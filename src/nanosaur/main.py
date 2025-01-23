# PYTHON_ARGCOMPLETE_OK
# Copyright (C) 2025, Raffaello Bonghi <raffaello@rnext.it>
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
import argcomplete
import sys
import inquirer
from inquirer.themes import GreenPassion
from jtop import jtop, JtopException

from nanosaur.docker import create_simple
from nanosaur.workspace import workspaces_info, parser_workspace_menu, create_developer_workspace, create_maintainer_workspace, get_workspaces_path
from nanosaur.robot import parser_robot_menu, robot_start, robot_stop
from nanosaur.simulation import parser_simulation_menu
from nanosaur.swarm import parser_swarm_menu
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList, package_info


NANOSAUR_INSTALL_OPTIONS = ['Simple', 'Developer', 'Maintainer']

# Define default parameters
DEFAULT_PARAMS = {}


def info(platform, params: Params, args):
    """Print version information."""
    # Print version information
    package_info(params, args.verbose)
    # Print mode if it exists in params
    if 'mode' in params:
        if params['mode'] == 'Developer':
            mode_string = TerminalFormatter.color_text(f"Mode: {params['mode']}", bg_color='green', bold=True)
            print(f"\n{mode_string}")
        elif params['mode'] == 'Maintainer':
            mode_string = TerminalFormatter.color_text(f"Mode: {params['mode']}", bg_color='red', bold=True)
            print(f"\n{mode_string}")
        elif params['mode'] == 'Raffo':
            mode_string = TerminalFormatter.color_text(f"Mode: {params['mode']}", bg_color='cyan', bold=True)
            print(f"\n{mode_string}")

    robot_list = RobotList.load(params)
    # Print current robot configuration
    robot_data = RobotList.get_robot(params)
    print()
    robot_data.verbose()
    # Print other robots if they exist
    if len(robot_list.robots) > 1 or args.verbose:
        print()
        robot_list.print_all_robots(params.get('robot_idx', 0))
    # Print simulation tools if they exist
    if 'simulation_tool' in params:
        print(f"\n{TerminalFormatter.color_text('Simulation Tool:', bold=True)} {params['simulation_tool']}")
    # Print installed workspaces
    workspaces_info(params, args.verbose)
    # Print all robot configurations
    if args.verbose:
        # Print device information
        print(TerminalFormatter.color_text("\nPlatform Information:", bold=True))
        for key, value in platform.items():
            print(f"  {key}: {value}")


def install(platform, params: Params, args):
    # Questions to ask the user
    questions = [
        inquirer.List(
            'choice',
            message="What would you like to install?",
            choices=NANOSAUR_INSTALL_OPTIONS,
        ),
        inquirer.Confirm(
            'confirm',
            message="Are you sure you want to install this?",
            default=False
        )
    ]
    # Ask the user to select an install type
    answers = inquirer.prompt(questions, theme=GreenPassion())
    if answers is None:
        return False
    # Check if the user wants to continue
    if answers['confirm'] is False:
        print(TerminalFormatter.color_text("Installation cancelled", color='red'))
        return False
    # Get the selected install type
    install_type = answers['choice']
    print(f"Installing {install_type} workspace...")

    install_functions = {
        'Simple': create_simple,
        'Developer': create_developer_workspace,
        'Maintainer': create_maintainer_workspace
    }

    if not install_functions[install_type](platform, params, args):
        return False
    # Set params in maintainer mode
    current_mode = params.get('mode', 'Simple')
    if (install_type == 'Developer' and current_mode == 'Simple') or \
       (install_type == 'Maintainer' and current_mode in ['Simple', 'Developer']) or \
       (install_type == 'Raffo' and current_mode in NANOSAUR_INSTALL_OPTIONS):
        params['mode'] = install_type


def robot_control(params, subparsers):
    robot = RobotList.get_robot(params).name
    robot_name = TerminalFormatter.color_text(robot, color='green', bold=True)
    parser_wakeup = subparsers.add_parser('wake-up', help=f"Start {robot_name} (same as 'nanosaur robot start')")
    parser_wakeup.set_defaults(func=robot_start)
    # Subcommand: shutdown
    parser_shutdown = subparsers.add_parser('shutdown', help="Shutdown the robot (same as 'nanosaur robot stop')")
    parser_shutdown.set_defaults(func=robot_stop)


def main():
    # Load the parameters
    params = Params.load(DEFAULT_PARAMS)

    # Extract device information with jtop
    try:
        with jtop() as device:
            if device.ok():
                platform = device.board['platform']
    except JtopException as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Determine the device type
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"

    # Create the argument parser
    parser = argparse.ArgumentParser(
        description="Nanosaur CLI - A command-line interface for the Nanosaur package.")
    
    parser.add_argument('--mode', type=str, help="Specify the mode of operation")
    # Define subcommands
    subparsers = parser.add_subparsers(dest='command', help="Available commands")

    # Subcommand: info
    parser_info = subparsers.add_parser('info', help="Show version information")
    parser_info.add_argument('--verbose', '-v', action='store_true', help="Show detailed information")
    parser_info.set_defaults(func=info)

    # Subcommand: install (hidden if workspace already exists)
    if 'mode' not in params:
        parser_install = subparsers.add_parser('install', help=f"Install nanosaur on your {device_type}")
    else:
        parser_install = subparsers.add_parser('install')
    # Add simulation install subcommand
    parser_install.add_argument('--force', action='store_true', help="Force the update")
    parser_install.add_argument('--all', action='store_true', help="Install for all platforms")
    parser_install.set_defaults(func=install)

    # Subcommand: workspace (with a sub-menu for workspace operations)
    if get_workspaces_path(params):
        # Add workspace subcommand
        parser_workspace = parser_workspace_menu(subparsers)

    # Subcommand: simulation (with a sub-menu for simulation types)
    if device_type == 'desktop' and 'mode' in params:
        # Add simulation subcommand
        parser_simulation = parser_simulation_menu(subparsers, params)

    # Add robot subcommand
    parser_robot, parser_config = parser_robot_menu(subparsers, params)

    if device_type == 'desktop':
        # Subcommand: swarm (with a sub-menu for swarm operations)
        parser_swarm = parser_swarm_menu(subparsers, params)

    # Subcommand: wakeup (with a sub-menu for wakeup operations)
    if 'mode' in params:
        robot_control(params, subparsers)

    # Enable tab completion
    argcomplete.autocomplete(parser)

    # Parse the arguments
    args = parser.parse_args()
    # Override mode if provided as an argument
    if args.mode:
        params.set('mode', args.mode, save=False)

    # Handle subcommands without a specific type
    if args.command in ['workspace', 'ws'] and not args.workspace_type:
        parser_workspace.print_help()
    elif args.command in ['simulation', 'sim'] and not args.simulation_type:
        parser_simulation.print_help()
    elif args.command == 'robot' and not args.robot_type:
        parser_robot.print_help()
    elif args.command == 'robot' and args.robot_type == 'config' and not args.config_type:
        parser_config.print_help()
    elif args.command == 'swarm' and not args.swarm_type:
        parser_swarm.print_help()
    elif hasattr(args, 'func'):
        # Execute the corresponding function based on the subcommand
        args.func(platform, params, args)
    else:
        # If no command is provided, display the help message
        parser.print_help()


if __name__ == "__main__":
    main()
# EOF
