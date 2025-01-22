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
from jtop import jtop, JtopException

from nanosaur import __version__
from nanosaur.utilities import Params, get_nanosaur_home
from nanosaur import workspace
from nanosaur.robot import parser_robot_menu
from nanosaur.simulation import parser_simulation_menu
from nanosaur.swarm import parser_swarm_menu
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import RobotList
import inquirer

NANOSAUR_CONFIG_FILE_NAME = 'nanosaur.yaml'
NANOSAUR_HOME_NAME = 'nanosaur_test'

# Define default parameters
DEFAULT_PARAMS = {
    'nanosaur_branch': 'nanosaur2',
    'nanosaur_home': NANOSAUR_HOME_NAME,
    'nanosaur_raw_github_repo': 'https://raw.githubusercontent.com/rnanosaur/nanosaur',
    'ws_developer_name': 'ros_ws',
    'ws_perception_name': 'perception_ws',
    'ws_robot_name': 'robot_ws',
    'ws_simulation_name': 'simulation_ws',
}


def info(platform, params: Params, args):
    """Print version information."""
    # Print mode if it exists in params
    if 'mode' in params:
        print(TerminalFormatter.color_text(f"Mode: {params['mode']}", bg_color='red', bold=True))
        print()

    robot_list = RobotList.load(params)
    # Print current robot configuration
    robot_data = RobotList.get_robot(params)
    robot_data.verbose()
    # Print other robots if they exist
    if len(robot_list.robots) > 1 or args.verbose:
        print()
        robot_list.print_all_robots(params.get('robot_idx', 0))
    # Print simulation tools if they exist
    if 'simulation_tool' in params:
        print(f"\n{TerminalFormatter.color_text('Simulation Tool:', bold=True)} {params['simulation_tool']}")

    # Print all workspaces installed
    print(TerminalFormatter.color_text("\nInstalled Workspaces:", bold=True))
    for ws_name in ['ws_perception_name', 'ws_robot_name', 'ws_simulation_name']:
        # Get the workspace path if it exists
        if ws_path := workspace.get_workspace_path(
            params, params.get(ws_name)
        ):
            ws_name_split = ws_name.split('_')[1]  # Split and get the middle part
            print(f"  {TerminalFormatter.color_text(ws_name_split, bold=True)}: {ws_path}")

    # Print all robot configurations
    if args.verbose:
        # Print device information
        print(TerminalFormatter.color_text("\nPlatform Information:", bold=True))
        for key, value in platform.items():
            print(f"  {key}: {value}")
    # Print version information
    print(TerminalFormatter.color_text("\nVersion Information:", bold=True))
    print(f"  {TerminalFormatter.color_text('Nanosaur package:', bold=True)} {__version__}")
    print(f"  {TerminalFormatter.color_text('Nanosaur version (branch):', bold=True)} {params['nanosaur_branch']}")
    print(f"  {TerminalFormatter.color_text('Nanosaur home:', bold=True)} {get_nanosaur_home(params['nanosaur_home'])}")
    config_file_path = Params.get_params_file(params['nanosaur_home'], NANOSAUR_CONFIG_FILE_NAME)
    print(f"  {TerminalFormatter.color_text('Nanosaur config file:', bold=True)} {config_file_path}")


def install_old(platform, params: Params, args, password=None):
    if args.developer:
        workspace.create_developer_workspace(platform, params, args)
    else:
        print(TerminalFormatter.color_text("Not implemented yet", color='red'))



def install(platform, params: Params, args, password=None):
    questions = [
        inquirer.List(
            'choice',
            message="What would you like to install?",
            choices=['Developer Workspace', 'Simulation Tools', 'Robot Configuration'],
        ),
    ]

    answers = inquirer.prompt(questions)

    if answers['choice'] == 'Developer Workspace':
        workspace.create_developer_workspace(platform, params, args)
    elif answers['choice'] == 'Simulation Tools':
        print(TerminalFormatter.color_text("Simulation Tools installation is not implemented yet", color='red'))
    elif answers['choice'] == 'Robot Configuration':
        print(TerminalFormatter.color_text("Robot Configuration installation is not implemented yet", color='red'))



def main():
    # Load the parameters
    params = Params.load(DEFAULT_PARAMS, home_folder=NANOSAUR_HOME_NAME, params_file_name=NANOSAUR_CONFIG_FILE_NAME)

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

    # Define subcommands
    subparsers = parser.add_subparsers(dest='command', help="Available commands")

    # Subcommand: info
    parser_info = subparsers.add_parser('info', help="Show version information")
    parser_info.add_argument('--verbose', '-v', action='store_true', help="Show detailed information")
    parser_info.set_defaults(func=info)

    # Subcommand: install (hidden if workspace already exists)
    #if 'mode' in params and params['mode'] in ['developer', 'maintainer']:
    parser_install = subparsers.add_parser('install', help=f"Install nanosaur on your {device_type}")
    #else:
    #    parser_install = subparsers.add_parser('install')
    # Add simulation install subcommand
    parser_install.add_argument('--developer', '--dev', action='store_true', help="Install developer workspace")
    parser_install.add_argument('--force', action='store_true', help="Force the update")
    parser_install.add_argument('--all-platforms', action='store_true', help="Install for all platforms")
    parser_install.set_defaults(func=install)

    # Subcommand: workspace (with a sub-menu for workspace operations)
    if workspace.get_workspaces_path(params):
        # Add workspace subcommand
        parser_workspace = workspace.parser_workspace_menu(subparsers)

    # Subcommand: simulation (with a sub-menu for simulation types)
    if device_type == 'desktop':
        # Add simulation subcommand
        parser_simulation = parser_simulation_menu(subparsers, params)

    # Add robot subcommand
    parser_robot, parser_config = parser_robot_menu(subparsers, params)

    if device_type == 'desktop':
        # Subcommand: swarm (with a sub-menu for swarm operations)
        parser_swarm = parser_swarm_menu(subparsers, params)

    # Enable tab completion
    argcomplete.autocomplete(parser)

    # Parse the arguments
    args = parser.parse_args()

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
