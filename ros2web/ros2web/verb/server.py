import os

import asyncio
import launch.logging
from launch import LaunchService, LaunchDescription
from ros2cli.verb import VerbExtension

from ros2web.api.create import create_config_directory
from ..db.web import ROS2WebDB, ROS2WebDBException
from ..launch.actions import SystemService
from ..launch.actions import HTTPServer
from ..launch.actions import WebPackageManager
from ..utilities import logging_screen_handler_remover, logging_logfile_handler_remover


class ServerVerb(VerbExtension):
    """Start the server."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--bind',
            help='Specify alternate bind address [default: all interfaces]')
        parser.add_argument(
            '--port',
            default='8080',
            help='Specify alternate port [default: 8080]')

        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the config directory')

    def main(self, *, args):
        self.__logger = launch.logging.get_logger('server')

        try:
            # config_directory = create_config_directory(
            #     args.destination_directory)
            # db = ROS2WebDB(config_directory)
            system_service = SystemService()
            web_package_manager = WebPackageManager()
            http_server = HTTPServer(host=args.bind, port=args.port)

            launch_service = LaunchService(argv=[], debug=False)
            launch_service.context.extend_globals({
                # 'db': db,
                'system_service': system_service,
                'web_package_manager': web_package_manager
            })
            # logging_screen_handler_remover(launch_service._LaunchService__logger)
            # logging_logfile_handler_remover(launch_service._LaunchService__logger)
            launch_description = LaunchDescription([
                system_service,
                web_package_manager,
                http_server
            ])
            launch_service.include_launch_description(launch_description)
            launch_service.run()
        except Exception as e:
            self.__logger.error(e)
        finally:
            pass
