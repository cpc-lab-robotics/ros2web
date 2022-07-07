import os

from argparse import ArgumentParser
from ros2cli.verb import VerbExtension

from ros2web.api.create_boilerplate import create_package


class CreateVerb(VerbExtension):
    """Create a new package api plugin."""

    def add_arguments(self, parser: ArgumentParser, cli_name):

        parser.add_argument('-e', '--extension', action='store_true',
                            help='Create extension package.')
        
        parser.add_argument('-a', '--all', action='store_true',
                            help='Create extension and web package.')
        

        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the package directory')

        parser.add_argument(
            'plugin_name',
            help='Name of the plugin package')

    def main(self, *, args):

        package_path = os.path.join(
            args.destination_directory, args.plugin_name)

        if os.path.exists(package_path):
            return '\nAborted!\nThe directory already exists: ' + package_path + '\n' + \
                'Either remove the directory or ' + \
                'choose a different destination directory or web package name'

        if args.all:
            types = {
                'package': True,
                'extension': True,
            }
        else:
            types = {
                'package': args.extension is False,
                'extension': args.extension,
            }
        
        create_package(args.plugin_name, args.destination_directory, types)
