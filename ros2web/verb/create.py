import os

from ros2cli.verb import VerbExtension

from ros2web.api.create import create_web_package


class CreateVerb(VerbExtension):
    """Create a new package api plugin."""

    def add_arguments(self, parser, cli_name):

        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the web package directory')

        parser.add_argument(
            'web_package_name',
            help='Name of the Web package')

    def main(self, *, args):

        package_path = os.path.join(
            args.destination_directory, args.web_package_name)

        if os.path.exists(package_path):
            return '\nAborted!\nThe directory already exists: ' + package_path + '\n' + \
                'Either remove the directory or ' + \
                'choose a different destination directory or web package name'

        create_web_package(args.web_package_name,
                           args.destination_directory)
        return
