# Copyright (C) 2024, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import sys
import subprocess
from jtop import jtop, JtopException

from nanosaur import __version__
from nanosaur.utilities import Params
from nanosaur import installation
from nanosaur import simulation


# Define default parameters
DEFAULT_PARAMS = {
    'nanosaur_workspace_name': 'nanosaur_ws',
    'nanosaur_branch': 'nanosaur2',
}


def info(platform, params: Params, args):
    """Print version information."""
    print(f"Nanosaur package version {__version__}")
    # Print configuration parameters
    print("\nConfiguration:")
    for key, value in params.items():
        if value:  # Only print if value is not empty
            print(f"  {key}: {value}")
    # Print device information
    print("\nPlatform Information:")
    for key, value in platform.items():
        print(f"  {key}: {value}")


def main():
    # Load the parameters
    params = Params.load(DEFAULT_PARAMS, params_file='nanosaur.yaml')

    # Extract device information with jtop
    try:
        with jtop() as device:
            if device.ok():
                platform = device.board['platform']
    except JtopException as e:
        print(f"Error: {e}")
        if subprocess.check_output("id -u", shell=True).strip() != b'0':
            print(
                "To automatically fix this error, this script must be run as root. Please use 'sudo'.")
            sys.exit(1)
        print("Attempting to update the jtop package...")
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "--upgrade", "jtop"])
        sys.exit(1)
    # Determine the device type
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"

    # Create the argument parser
    parser = argparse.ArgumentParser(
        description="Nanosaur CLI - A command-line interface for the Nanosaur package.")

    # Define subcommands
    subparsers = parser.add_subparsers(
        dest='command', help="Available commands")
    # Subcommand: info
    parser_info = subparsers.add_parser(
        'info', help="Show version information")
    parser_info.set_defaults(func=info)

    # Subcommand: install (with a sub-menu for installation types)
    parser_install = subparsers.add_parser(
        'install', help="Run the installation process")
    install_subparsers = parser_install.add_subparsers(
        dest='install_type', help="Installation types")

    parser_install_developer = install_subparsers.add_parser(
        'update', help="Update the installation")
    parser_install_developer.add_argument(
        '--force',
        action='store_true',
        help="Force the installation")
    parser_install_developer.set_defaults(func=installation.update)

    # Subcommand: install basic
    parser_install_basic = install_subparsers.add_parser(
        'basic', help="Perform a basic installation")
    parser_install_basic.add_argument(
        '--force',
        action='store_true',
        help="Force the installation")
    parser_install_basic.set_defaults(func=installation.install_basic)

    # Subcommand: install developer
    parser_install_developer = install_subparsers.add_parser(
        'developer', help="Perform the developer installation")
    parser_install_developer.add_argument(
        '--force',
        action='store_true',
        help="Force the installation")
    parser_install_developer.set_defaults(func=installation.install_developer)

    # Subcommand: install simulation
    if device_type == 'desktop':
        parser_install_simulation = install_subparsers.add_parser(
            'simulation', help="Install the simulation tools")
        parser_install_simulation.add_argument(
            '--force', action='store_true', help="Force the installation")
        parser_install_simulation.set_defaults(
            func=installation.install_simulation)

    # Subcommand: update (with a sub-menu for installation types)
    if device_type == 'desktop':
        parser_simulation = subparsers.add_parser(
            'simulation', help="Update all nanosaur devices")
        simulation_subparsers = parser_simulation.add_subparsers(
            dest='simulation_type', help="Simulation types")
        parser_simulation_start = simulation_subparsers.add_parser(
            'start', help="Start the simulation selected")
        parser_simulation_start.set_defaults(func=simulation.simulation_start)
        parser_simulation_set = simulation_subparsers.add_parser(
            'set', help="Select the simulator you want to use")
        parser_simulation_set.set_defaults(func=simulation.simulation_set)

    # Parse the arguments
    args = parser.parse_args()

    # Handle install subcommand without an install_type
    if args.command == 'install' and args.install_type is None:
        parser_install.print_help()
        sys.exit(1)

    # Handle install subcommand without an install_type
    if args.command == 'simulation' and args.simulation_type is None:
        parser_simulation.print_help()
        sys.exit(1)

    # Execute the corresponding function based on the subcommand
    if hasattr(args, 'func'):
        args.func(platform, params, args)
    else:
        # If no command is provided, display a custom help message without the
        # list of commands
        parser.print_help()


if __name__ == "__main__":
    main()
# EOF
