from faulthandler import disable
from typing import Optional, cast, TYPE_CHECKING, NamedTuple
from typing import List, Dict, Set

import functools
import re
import pathlib
import importlib.resources
from importlib.metadata import metadata as setup_metadata
import asyncio
import pkg_resources
from multiprocessing import JoinableQueue
import os.path
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler

import launch.logging
from launch.actions import OpaqueFunction, OpaqueCoroutine
from launch.event import Event
from launch.event_handler import EventHandler
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from launch.utilities import is_a_subclass

from ..events import ExecutionPlugin
from ...plugin.process import PluginProcess
from ...db.web import ROS2WebDB, ROS2WebDBException

from ...utilities import open_yaml_file

if TYPE_CHECKING:
    from .system_service import SystemService


class Plugin(NamedTuple):
    id: str
    type: str
    name: str
    package_name: str
    class_name: str
    module_name: str
    location: str
    config: Dict
    disable: bool
    summary: str
    version: str


class PluginManager(OpaqueCoroutine):

    def __init__(self) -> None:
        super().__init__(coroutine=self.__coroutine)

        self.__logger = launch.logging.get_logger('PluginManager')

        self.__plugin_process_dict: Dict[str, PluginProcess] = {}
        self.__plugin_dict: Dict[str, Plugin] = {}

        self.__send_queue = None
        self.__receive_queue: asyncio.Queue = asyncio.Queue()
        self.__is_running = False
        self.__receive_plugin_queue: JoinableQueue = JoinableQueue()
        self.__receive_loop_task = None
        self.__launch_context = None
        self.__db: ROS2WebDB = None
        self.__loop = asyncio.get_event_loop()

        self.__observer = Observer()
        self.__plugin_file_watcher: Dict[str, Observer] = {}

    def __on_shutdown(self, event: Event, context: LaunchContext):
        self.__is_running = False
        self.__receive_plugin_queue.put(None)

        if self.__receive_loop_task is not None:
            self.__receive_loop_task.cancel()

        if len(self.__plugin_file_watcher) > 0:
            self.__observer.stop()
            self.__observer.join()

        for process in self.__plugin_process_dict.values():
            process.shutdown()

    async def __coroutine(self, context: LaunchContext):
        
        self.__launch_context = context
        self.__db = cast(ROS2WebDB, context.get_locals_as_dict().get("db"))

        event_handlers = [
            OnShutdown(on_shutdown=self.__on_shutdown),
            EventHandler(
                matcher=lambda event: is_a_subclass(
                    event, ExecutionPlugin),
                entities=OpaqueFunction(
                    function=lambda context: [
                        cast(ExecutionPlugin, context.locals.event).plugin]
                ),
            )
        ]
        for event_handler in event_handlers:
            context.register_event_handler(event_handler)

        self.__is_running = True
        self.__loop.run_in_executor(None, self.__receive_plugin_queue_loop)
        self.__receive_loop_task = self.__loop.create_task(
            self.__receive_loop())

        
        service: SystemService = context.get_locals_as_dict().get("system_service")
        service.init(plugin_manager=self)

        self.__load_plugin('extension')
        self.__load_plugin('package')
        self.__activation_plugins(service)

        if len(self.__plugin_file_watcher) > 0:
            self.__observer.start()

    def __load_plugin(self, plugin_type: str):
        table = self.__db.db('ros2web').table('plugin')

        for entry_point in pkg_resources.iter_entry_points(f'ros2web.{plugin_type}'):
            try:

                location = entry_point.dist.location
                package_name = os.path.basename(location)
                plugin_name = entry_point.name
                plugin_id = f'{plugin_type}.{plugin_name}'

                if plugin_id in self.__plugin_dict:
                    continue

                plugin_summary = 'unknown'
                plugin_version = 'unknown'

                metadata = setup_metadata(package_name)
                if metadata:
                    plugin_summary = metadata['Summary']
                    plugin_version = metadata['Version']

                module_name = entry_point.module_name
                config = open_yaml_file(package_name, 'config.yml')
                class_name = None
                if len(entry_point.attrs) > 0:
                    class_name = entry_point.attrs[0]

                disable = False
                plugin_doc = self.__db.get(table, {
                    'id': plugin_id
                })
                
                if plugin_doc:
                    disable = plugin_doc.get('disable', disable)

                self.__plugin_dict[plugin_id] = Plugin(
                    id=plugin_id,
                    type=plugin_type,
                    name=plugin_name,
                    package_name=package_name,
                    class_name=class_name,
                    module_name=module_name,
                    location=location,
                    config=config,
                    disable=disable,
                    summary=plugin_summary,
                    version=plugin_version
                )
            except Exception as e:
                self.__logger.error("[{}]:{}".format(package_name, e))

    def __set_hot_reload(self, plugin: Plugin):

        hot_reload = plugin.config.get('hot_reload', None)
        if hot_reload is not None:
            sym_link = os.path.join(plugin.location, plugin.package_name)
            path = os.path.realpath(sym_link)
            patterns = hot_reload.get('patterns', ['*.py'])
            recursive = hot_reload.get('recursive', True)

            event_handler = PatternMatchingEventHandler(
                patterns=patterns)
            event_handler.on_modified = functools.partial(
                self.__modified_event_handler, plugin)
            observed_watch = self.__observer.schedule(
                event_handler,
                path,
                recursive=recursive
            )
            self.__plugin_file_watcher[plugin.id] = observed_watch

    def __activation_plugins(self, service: 'SystemService'):

        for name, plugin in self.__plugin_dict.items():
            try:
                self.__set_hot_reload(plugin)

                process = PluginProcess(
                    plugin=plugin,
                    service=service,
                    receive_queue=self.__receive_plugin_queue,
                )
                self.__plugin_process_dict[plugin.id] = process

                if plugin.disable is False:
                    process.start()
            except Exception as e:
                self.__logger.error(
                    "Failed to activate '{}': {}".format(name, e))

    def __modified_event_handler(self, plugin: Plugin, event):
        if plugin.disable is False:
            file_name = os.path.basename(event.src_path)
            if 'config.yml' == file_name:
                config = open_yaml_file(plugin.package_name, 'config.yml')
                plugin = plugin._replace(config=config)
                self.__plugin_dict[plugin.id] = plugin
            
            process = self.__plugin_process_dict.get(plugin.id)
            if process is not None:
                process.restart(plugin)

    def __receive_plugin_queue_loop(self):
        try:
            while self.__is_running:
                try:
                    data = self.__receive_plugin_queue.get()
                    if self.__send_queue is not None and self.__is_running:
                        # self.__logger.info("send queue:{}".format(data))
                        asyncio.run_coroutine_threadsafe(
                            self.__send_queue.put(data), self.__loop)
                except Exception as e:
                    self.__logger.error(e)
                finally:
                    if self.__receive_plugin_queue.qsize() > 0:
                        self.__receive_plugin_queue.task_done()
        finally:
            self.__receive_plugin_queue.close()
            self.__receive_plugin_queue.join_thread()

    async def __receive_loop(self):
        try:
            while True:
                try:
                    data = await self.__receive_queue.get()
                    plugin_name = data.get('webPackageName')
                    plugin_id = f'package.{plugin_name}'
                    process = self.__plugin_process_dict.get(plugin_id)
                    if process:
                        process.send_data(data)
                except Exception as e:
                    self.__logger.error(e)
                finally:
                    if self.__receive_queue.qsize() > 0:
                        self.__receive_queue.task_done()
        except asyncio.CancelledError:
            pass
        finally:
            pass

    @property
    def receive_queue(self):
        return self.__receive_queue

    def set_send_queue(self, queue: asyncio.Queue):
        self.__send_queue = queue

    # API
    def emit_system_event(self, plugin_id, event):
        data = {
            'operation': 'system_event',
            'event': event
        }
        process = self.__plugin_process_dict.get(plugin_id)
        if process:
            process.send_data(data)
    
    def get_plugins(self) -> List:
        plugins = []
        for plugin in self.__plugin_dict.values():
            plugins.append(plugin.id)
        return sorted(plugins)
    
    def get_plugin(self, plugin_id: str)->Optional[Plugin]:
        
        plugin = self.__plugin_dict.get(plugin_id)
        if plugin:
            return plugin._asdict()
        else:
            return None
        
    def enable_plugin(self, plugin_id: str):
        
        table = self.__db.db('ros2web').table('plugin')
        process = self.__plugin_process_dict.get(plugin_id)
        if process is not None:
            plugin = self.__plugin_dict[plugin_id]
            
            process.start()
            plugin = plugin._replace(disable=False)
            self.__plugin_dict[plugin_id] = plugin
            self.__db.upsert(table, {
                'id': plugin_id,
                'disable': False
            }, {'id': plugin_id})

            return plugin._asdict()
        else:
            return None

    def disable_plugin(self, plugin_id: str):
        table = self.__db.db('ros2web').table('plugin')
        process = self.__plugin_process_dict.get(plugin_id)
        if process is not None:
            plugin = self.__plugin_dict[plugin_id]
            
            process.shutdown()
            plugin = plugin._replace(disable=True)
            self.__plugin_dict[plugin_id] = plugin
            self.__db.upsert(table, {
                'id': plugin_id,
                'disable': True
            }, {'id': plugin_id})
            
            return plugin._asdict()
        else:
            return None

    def get_plugin_process(self, plugin_type: str, plugin_name: str):
        plugin_id = f"{plugin_type}.{plugin_name}"
        process = self.__plugin_process_dict.get(plugin_id)
        return process

    def get_plugin_files(self, *, plugin_type: str, plugin_name: str, directory: str, file_name: str) -> Optional[str]:
        file_path = None
        plugin_id = f"{plugin_type}.{plugin_name}"
        plugin = self.__plugin_dict.get(plugin_id)
        if plugin:
            if plugin.disable is False:
                with importlib.resources.path(plugin.package_name, "data") as path:
                    path = path.joinpath(directory, file_name)
                    if os.path.exists(path):
                        file_path = path
        return file_path

    def __get_init_props(self, widget):

        init_props = {}
        init_grid = {}

        widget_name = widget['name']
        m = re.fullmatch(r'^(.+)\.(.+)$', widget_name)
        if m:
            plugin_name = m.group(1)
            widget_name = m.group(2)
            widget_key = f'extension.{plugin_name}'
            plugin = self.__plugin_dict.get(widget_key)
            if plugin:
                widget_doc = plugin.config.get(widget_name, {})
                init_props = widget_doc.get("props", {})
                init_grid = widget_doc.get("grid", {})

        return init_props, init_grid

    def init_widgets(self, widgets: List[Dict]) -> Dict:
        
        layout = {}
        if widgets is not None:
            for widget in widgets:
                init_props, init_grid = self.__get_init_props(widget)
                props = widget.get('props', {})
                props = {} if props is None else props
                widget_key = widget['key']
                grid =  widget.get('grid', {})
                layout[widget_key] = {**init_grid, **grid}
                widget['props'] = {**init_props, **props}

        return layout
