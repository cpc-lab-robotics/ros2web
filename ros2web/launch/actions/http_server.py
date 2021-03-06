from typing import Dict, Set

import os
import asyncio
import functools
import secrets
import aiohttp
from aiohttp import web
import importlib.resources

import launch.logging
from launch import LaunchContext
from launch.actions import OpaqueCoroutine

from .web_package_manager import WebPackageManager
from ...utilities import get_ip_address

with importlib.resources.path("ros2web", "data") as path:
    UI_FILE_PATH = path.joinpath("ui")


logger = launch.logging.get_logger('HTTPServer')
routes = web.RouteTableDef()


def get_web_packge_process(request: web.Request):
    web_package_name = request.match_info['web_package_name']
    web_package_manager: WebPackageManager = request.app["web_package_manager"]
    wp_process = web_package_manager.get_web_package_process(web_package_name)
    if wp_process is None:
        raise web.HTTPNotFound()
    return wp_process


@routes.get("/")
async def index_handler(request: web.Request) -> web.StreamResponse:
    return web.FileResponse(os.path.join(UI_FILE_PATH, 'index.html'))


@routes.get("/ros2web")
async def ros2web(request: web.Request) -> web.StreamResponse:
    return web.FileResponse(os.path.join(UI_FILE_PATH, 'index.html'))


@routes.get("/ros2web/{web_package_name}")
async def ros2web_package(request: web.Request) -> web.StreamResponse:
    get_web_packge_process(request)
    return web.FileResponse(os.path.join(UI_FILE_PATH, 'index.html'))


@routes.get("/ros2web/{web_package_name}/{tail:.*}")
async def ros2web_package_tail(request: web.Request) -> web.StreamResponse:
    get_web_packge_process(request)
    return web.FileResponse(os.path.join(UI_FILE_PATH, 'index.html'))


@routes.get('/api/ros2web/page/{web_package_name}/{tail:.*}')
async def web_package_info(request: web.Request) -> web.StreamResponse:
    wp_process = get_web_packge_process(request)
    path = request.match_info['tail']
    search_params = request.rel_url.query
    
    page_data = await wp_process.call_api(
        request_method='page',
        path=path,
        search_params=dict(search_params),
    )
    return web.json_response(page_data)


@routes.get('/api/ros2web/state/{web_package_name}')
async def web_package_state(request: web.Request) -> web.StreamResponse:
    wp_process = get_web_packge_process(request)
    state = await wp_process.get_state()
    return web.json_response(state)


@routes.get('/api/ros2web/state/{web_package_name}/{key}')
async def web_package_state_key(request: web.Request) -> web.StreamResponse:
    wp_process = get_web_packge_process(request)
    key = request.match_info['key']
    state = await wp_process.get_state()
    return web.json_response(state.get(key))


@routes.get('/api/{web_package_name}/{tail:.*}')
async def web_package_extension_get_api(request: web.Request) -> web.StreamResponse:
    wp_process = get_web_packge_process(request)
    
    tail = request.match_info['tail']
    search_params = request.rel_url.query
    result = await wp_process.call_api(
        request_method='get',
        path=tail,
        search_params=dict(search_params),
    )
    if result is None:
        raise web.HTTPNotFound()

    return web.json_response(result)


@routes.post('/api/{web_package_name}/{tail:.*}')
async def web_package_extension_post_api(request: web.Request) -> web.StreamResponse:
    wp_process = get_web_packge_process(request)
    tail = request.match_info['tail']
    
    # TODO: Taking the defined API and settings.
    # byte_payload = await request.read() # returns bytes object with body content.
    json_payload = await request.json() # Read request body decoded as json
    # text_payload = await request.text() # Returns str with body content.
    
    result = await wp_process.call_api(
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


@routes.get("/{web_package_name}")
async def web_package_handler(request: web.Request) -> web.StreamResponse:
    web_package_name = request.match_info['web_package_name']
    wp_process = get_web_packge_process(request)

    # TODO: Handling package resources
    return web.Response(text=web_package_name)


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


def init_web_app(*, token, context) -> web.Application:
    app = web.Application()
    app['launch_context'] = context
    app['connected'] = set()
    app['token'] = token

    web_package_manager: WebPackageManager = context.\
        get_locals_as_dict().get("web_package_manager")

    app['web_package_manager'] = web_package_manager
    app['lock'] = asyncio.Lock()

    receive_queue = asyncio.Queue()
    app['receive_queue'] = receive_queue
    app['send_queue'] = web_package_manager.receive_queue
    web_package_manager.set_send_queue(receive_queue)

    app.router.add_routes(routes)

    # TODO: web package????????????????????????
    app.router.add_static("/", UI_FILE_PATH, follow_symlinks=True)

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
            token = secrets.token_urlsafe()
            self.__runner = web.AppRunner(init_web_app(token=token,
                                                       context=context))
            await self.__runner.setup()

            site = web.TCPSite(self.__runner, self.__host, self.__port)
            await site.start()

            ip_address = self.__host
            if self.__host is None:
                ip_address = get_ip_address()

            # url = f'http://{self._host}:{self._port}?token=' + token
            url = f'http://0.0.0.0:{self.__port}'
            # url = f'http://{ip_address}:{self.__port}'

            print(
                "------------------------------------------------------------------------\n"
                "{}\n"
                "------------------------------------------------------------------------\n"
                "(Press CTRL+C to quit)\n".format(url)
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
