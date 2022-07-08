from typing import Dict, Set, TYPE_CHECKING, cast
from typing import NoReturn

import os
import asyncio
import secrets
import aiohttp
from aiohttp import web
import importlib.resources
import launch.logging
from launch import LaunchContext
from launch.actions import OpaqueCoroutine
from ros2web.plugin.process import PluginProcess

from ...utilities import get_ip_address

with importlib.resources.path("ros2web", "data") as path:
    PUBLIC_FILE_PATH = path.joinpath("public")

if TYPE_CHECKING:
    from .plugin_manager import PluginManager

logger = launch.logging.get_logger('HTTPServer')
routes = web.RouteTableDef()


def get_plugin_process(plugin_type: str, request: web.Request):
    plugin_name = request.match_info['plugin_name']
    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    process = plugin_manager.get_plugin_process(plugin_type, plugin_name)
    if process is None:
        raise web.HTTPNotFound()
    return process


@routes.get('/api/ros2web/plugins')
async def get_plugins(request: web.Request) -> web.StreamResponse:
    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    plugins = plugin_manager.get_plugins()
    return web.json_response(plugins)


@routes.get('/api/ros2web/plugin/enable')
async def enable_plugin(request: web.Request) -> web.StreamResponse:
    search_params = request.rel_url.query
    plugin_id = search_params.get('id')
    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    plugin = plugin_manager.enable_plugin(plugin_id)

    if plugin is None:
        raise web.HTTPBadRequest()

    return web.json_response(plugin)


@routes.get('/api/ros2web/plugin/disable')
async def disable_plugin(request: web.Request) -> web.StreamResponse:
    search_params = request.rel_url.query
    plugin_id = search_params.get('id')
    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    plugin = plugin_manager.disable_plugin(plugin_id)

    if plugin is None:
        raise web.HTTPBadRequest()

    return web.json_response(plugin)


@routes.get('/api/ros2web/plugin/{plugin_id}')
async def get_plugin(request: web.Request) -> web.StreamResponse:
    plugin_id = request.match_info['plugin_id']
    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    plugin = plugin_manager.get_plugin(plugin_id)

    if plugin is None:
        raise web.HTTPNotFound()

    return web.json_response(plugin)


@routes.get('/api/ros2web/page/{plugin_name}/{tail:.*}')
async def get_package_page(request: web.Request) -> web.StreamResponse:
    plugin_name = request.match_info['plugin_name']
    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    plugin_process = cast(
        PluginProcess, plugin_manager.get_plugin_process('package', plugin_name))

    if plugin_process is None:
        raise web.HTTPNotFound()

    path = request.match_info['tail']
    search_params = request.rel_url.query
    page_data = await plugin_process.call_api(
        request_method='page',
        path=path,
        search_params=dict(search_params),
    )
    widgets = page_data.get('widgets', [])
    layout = plugin_manager.init_widgets(widgets)
    page_data['layout'] = layout

    return web.json_response(page_data)


@routes.get('/api/ros2web/state/{plugin_name}/{key}')
async def get_package_state_key(request: web.Request) -> web.StreamResponse:
    plugin_process = get_plugin_process('package', request)
    key = request.match_info['key']
    state = await plugin_process.get_state()
    return web.json_response(state.get(key))


@routes.get('/api/ros2web/{plugin_name}/{tail:.*}')
async def extension_get_api(request: web.Request) -> web.StreamResponse:
    plugin_process = get_plugin_process('extension', request)
    tail = request.match_info['tail']
    search_params = request.rel_url.query
    result = await plugin_process.call_api(
        request_method='get',
        path=tail,
        search_params=dict(search_params),
    )

    if result is None:
        raise web.HTTPNotFound()

    return web.json_response(result)


@routes.post('/api/ros2web/{plugin_name}/{tail:.*}')
async def extension_post_api(request: web.Request) -> web.StreamResponse:

    plugin_process = get_plugin_process('extension', request)
    tail = request.match_info['tail']
    # byte_payload = await request.read() # returns bytes object with body content.
    json_payload = await request.json()  # Read request body decoded as json
    # text_payload = await request.text() # Returns str with body content.

    result = await plugin_process.call_api(
        request_method='post',
        path=tail,
        json_payload=dict(json_payload)
    )

    if result is None:
        raise web.HTTPNotFound()

    return web.json_response(result)


@routes.get('/api/{plugin_name}/{tail:.*}')
async def package_plugin_get_api(request: web.Request) -> web.StreamResponse:
    plugin_process = get_plugin_process('package', request)

    tail = request.match_info['tail']
    search_params = request.rel_url.query
    result = await plugin_process.call_api(
        request_method='get',
        path=tail,
        search_params=dict(search_params),
    )
    if result is None:
        raise web.HTTPNotFound()

    return web.json_response(result)


@routes.post('/api/{plugin_name}/{tail:.*}')
async def package_plugin_post_api(request: web.Request) -> web.StreamResponse:
    plugin_process = get_plugin_process('package', request)
    tail = request.match_info['tail']

    # byte_payload = await request.read() # returns bytes object with body content.
    json_payload = await request.json()  # Read request body decoded as json
    # text_payload = await request.text() # Returns str with body content.

    result = await plugin_process.call_api(
        request_method='post',
        path=tail,
        json_payload=dict(json_payload)
    )

    if result is None:
        raise web.HTTPNotFound()

    return web.json_response(result)


@routes.get("/subscription/state")
async def websocket_handler(request: web.Request) -> web.WebSocketResponse:

    ws = web.WebSocketResponse()
    await ws.prepare(request)

    async with request.app['lock']:
        request.app["connected"].add(ws)

    send_queue = request.app["send_queue"]

    try:
        async for msg in ws:
            data = msg.json()
            await send_queue.put(data)
    except RuntimeError as e:
        logger.error(f"RuntimeError: {e}")
    except ValueError as e:
        logger.error(f"ValueError: {e}")
    except TypeError as e:
        logger.error(f"TypeError: {e}")
    finally:
        async with request.app['lock']:
            request.app["connected"].remove(ws)

    return ws


# @routes.get("/{file_path:.*}")
# async def ros2web(request: web.Request) -> web.StreamResponse:

#     file_path = request.match_info['file_path']

#     base, ext = os.path.splitext(file_path)
#     if ext == '':
#         file_path = 'index.html'

#     file_name = os.path.basename(file_path)

#     file_path = os.path.join(PUBLIC_FILE_PATH, file_name)
#     if os.path.exists(file_path):
#         return web.FileResponse(file_path)

#     raise web.HTTPNotFound()

@routes.get("/")
async def root_handler(request: web.Request) -> NoReturn:
    exc = web.HTTPFound(location="/ros2web")
    raise exc


@routes.get('/ros2web/widget/{plugin_name}/{file_name}')
async def get_widget_file(request: web.Request) -> web.StreamResponse:
    plugin_name = request.match_info['plugin_name']
    file_name = request.match_info['file_name']

    # async with aiohttp.ClientSession() as session:
    #     async with session.get(f"http://localhost:3001/{file_name}") as r:
    #         text = await r.text()
    # return web.Response(text=text)

    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    file_path = plugin_manager.get_plugin_files(plugin_type='extension',
                                                plugin_name=plugin_name,
                                                directory='widgets',
                                                file_name=file_name)

    if file_path is None:
        raise web.HTTPNotFound()
    return web.FileResponse(file_path)


@routes.get("/ros2web")
async def index_handler(request: web.Request) -> web.StreamResponse:
    response = web.FileResponse(os.path.join(PUBLIC_FILE_PATH, 'index.html'))
    # response.set_cookie('')
    return response


@routes.get("/ros2web/{file_path:.*}")
async def ros2web(request: web.Request) -> web.StreamResponse:
    file_path = request.match_info['file_path']
    base, ext = os.path.splitext(file_path)
    if ext == '':
        file_path = 'index.html'

    file_path = os.path.join(PUBLIC_FILE_PATH, file_path)
    if os.path.exists(file_path):
        return web.FileResponse(file_path)

    raise web.HTTPNotFound()


@routes.get('/{plugin_name}')
async def plugin_index(request: web.Request) -> web.StreamResponse:
    plugin_name = request.match_info['plugin_name']
    plugin_manager: 'PluginManager' = request.app["plugin_manager"]

    file_path = plugin_manager.get_plugin_files(plugin_type='package',
                                                plugin_name=plugin_name,
                                                directory='public',
                                                file_name='index.html')
    if file_path is None:
        raise web.HTTPNotFound()
    return web.FileResponse(file_path)


@routes.get('/{plugin_name}/{file_path:.*}')
async def plugin_file(request: web.Request) -> web.StreamResponse:
    file_path = request.match_info['file_path']
    plugin_name = request.match_info['plugin_name']
    base, ext = os.path.splitext(file_path)

    directory = 'public'
    if ext == '':
        file_name = 'index.html'
    else:
        file_name = os.path.basename(file_path)

    d = file_path.split('/')
    if '..' in d:
        raise web.HTTPNotFound()

    if len(d) > 1:
        directory = os.path.join('public', *d[:-1])

    plugin_manager: 'PluginManager' = request.app["plugin_manager"]
    file_path = plugin_manager.get_plugin_files(plugin_type='package',
                                                plugin_name=plugin_name,
                                                directory=directory,
                                                file_name=file_name)
    if file_path is None:
        raise web.HTTPNotFound()
    return web.FileResponse(file_path)


async def queue_listener(app: web.Application) -> None:
    receive_queue: asyncio.Queue = app["receive_queue"]
    connected: Set[web.WebSocketResponse] = app["connected"]

    try:
        while True:
            data = await receive_queue.get()
            try:
                if not connected:
                    continue
                # logger.info("send queue:{}".format(data))
                await asyncio.wait([client.send_json(data)
                                    for client in connected if client.closed is False])
            except ConnectionResetError as e:
                logger.error(e)
            except RuntimeError as e:
                logger.error(f"RuntimeError: {e}")
            except ValueError as e:
                logger.error(f"ValueError: {e}")
            except TypeError as e:
                logger.error(f"TypeError: {e}")
            finally:
                receive_queue.task_done()
    except asyncio.CancelledError:
        pass
    finally:
        pass


@web.middleware
async def token_auth(request: web.Request, handler):
    _token = request.app["token"]

    if request.path == '/':
        search_params = request.rel_url.query
        token_param = search_params.get("token", request.cookies.get("AUTH"))
        if _token != token_param:
            raise web.HTTPUnauthorized()
        else:
            exc = web.HTTPFound(location="/ros2web")
            exc.set_cookie("AUTH", _token)
            raise exc
    else:
        token = request.cookies.get("AUTH")
        if _token != token:
            raise web.HTTPUnauthorized()

    response = await handler(request)

    return response


async def on_startup(app: web.Application) -> None:
    app["queue_listener"] = asyncio.create_task(queue_listener(app))


async def on_shutdown(app: web.Application) -> None:
    receive_queue = app["receive_queue"]
    await receive_queue.join()

    app["queue_listener"].cancel()
    await app["queue_listener"]

    connected: Set[web.WebSocketResponse] = app["connected"]
    if len(connected) > 0:
        await asyncio.wait([client.close(code=1006, message="Server shutdown")
                            for client in connected if client.closed is False])


async def on_cleanup(app: web.Application) -> None:
    pass


def init_web_app(*, token, context: LaunchContext) -> web.Application:
    app = web.Application(middlewares=[token_auth])
    # app = web.Application()

    app['launch_context'] = context
    app['connected'] = set()
    app['token'] = token

    plugin_manager: 'PluginManager' = context.get_locals_as_dict().get("plugin_manager")
    app['plugin_manager'] = plugin_manager
    app['lock'] = asyncio.Lock()
    receive_queue = asyncio.Queue()
    app['receive_queue'] = receive_queue
    app['send_queue'] = plugin_manager.receive_queue
    plugin_manager.set_send_queue(receive_queue)

    app.router.add_routes(routes)

    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.on_cleanup.append(on_cleanup)

    return app


class HTTPServer(OpaqueCoroutine):

    def __init__(self, *, host, port):
        self.__host = host
        self.__port = port
        self.__runner = None
        super().__init__(coroutine=self.__coroutine)

    async def __start_server(self, context: LaunchContext):
        try:
            token = secrets.token_hex(16)
            self.__runner = web.AppRunner(init_web_app(token=token,
                                                       context=context))
            await self.__runner.setup()

            site = web.TCPSite(self.__runner, self.__host, self.__port)
            await site.start()

            ip_address = self.__host
            if self.__host is None:
                ip_address = get_ip_address()

            url1 = f'http://localhost:{self.__port}?token=' + token
            url2 = f'http://{ip_address}:{self.__port}?token=' + token

            print(
                "------------------------------------------------------------------------\n"
                "{}\n"
                "{}\n"
                "------------------------------------------------------------------------\n"
                "(Press CTRL+C to quit)\n".format(url1, url2)
            )
            completed_future = asyncio.get_event_loop().create_future()
            await completed_future
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(e)
        finally:
            await self.__runner.cleanup()

    async def __coroutine(self, context: LaunchContext):
        await self.__start_server(context)
