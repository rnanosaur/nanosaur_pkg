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
from nanosaur.utilities import Params
from nanosaur import workspace
from nanosaur import simulation
from nanosaur import robot
from nanosaur.prompt_colors import TerminalFormatter

NANOSAUR_CONFIG_FILE_NAME = 'nanosaur.yaml'
NANOSAUR_HOME_NAME = 'nanosaur'

# Define default parameters
DEFAULT_PARAMS = {
    'nanosaur_branch': 'nanosaur2',
    'nanosaur_home': NANOSAUR_HOME_NAME,
    'nanosaur_raw_github_repo': 'https://raw.githubusercontent.com/rnanosaur/nanosaur',
    'ws_perception_name': 'perception_ws',
    'ws_robot_name': 'robot_ws',
    'ws_simulation_name': 'simulation_ws',
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


def install(platform, params: Params, args, password=None):
    if args.developer:
        workspace.create_developer_workspace(platform, params, args)
    else:
        print(TerminalFormatter.color_text("Not implemented yet", color='red'))


def parser_workspace_menu(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser_workspace = subparsers.add_parser(
        'workspace', aliases=["ws"], help="Manage the Nanosaur workspace")
    workspace_subparsers = parser_workspace.add_subparsers(
        dest='workspace_type', help="Workspace types")
    # Add workspace clean subcommand

    def add_workspace_subcommand(name, help_text, func):
        parser = workspace_subparsers.add_parser(name, help=help_text)
        parser.add_argument('workspace', type=str, nargs='?', help="Specify the workspace to clean")
        parser.add_argument('--force', action='store_true', help="Force the workspace clean")
        parser.add_argument('--all-platforms', '--all', action='store_true', help="Clean all workspaces")
        parser.set_defaults(func=func)
        return parser
    # Add workspace clean subcommand
    parser_workspace_clean = add_workspace_subcommand('clean', "Clean the workspace", workspace.clean)
    parser_workspace_clean.add_argument('--perception', action='store_true', help="Clean the perception workspace")
    add_workspace_subcommand('update', "Update the workspace", workspace.update)
    # Add workspace perception subcommand
    parser_workspace_perception = workspace_subparsers.add_parser(
        'perception', help="Start the Isaac ROS docker container")
    parser_workspace_perception.set_defaults(func=workspace.run_dev_script)
    return parser_workspace


def parser_simulation_menu(subparsers: argparse._SubParsersAction, params: Params) -> argparse.ArgumentParser:
    # Get the simulation tool from the parameters
    simulation_type = params.get('simulation_tool', "NOT SELECTED")
    # Add simulation subcommand
    parser_simulation = subparsers.add_parser(
        'simulation', aliases=["sim"], help=f"Work with simulation tools [{simulation_type}]")
    simulation_subparsers = parser_simulation.add_subparsers(
        dest='simulation_type', help="Simulation types")

    # Add simulation start subcommand
    parser_simulation_start = simulation_subparsers.add_parser(
        'start', help="Start the selected simulation")
    parser_simulation_start.set_defaults(func=simulation.simulation_start)

    # Add simulation set subcommand
    parser_simulation_set = simulation_subparsers.add_parser(
        'set', help="Select the simulator you want to use")
    parser_simulation_set.set_defaults(func=simulation.simulation_set)
    return parser_simulation


def parser_swarm_menu(subparsers: argparse._SubParsersAction, params: Params) -> argparse.ArgumentParser:
    # Get the robot index from the parameters
    idx_swarm = params.get('robot_idx', 0)
    # Subcommand: swarm (with a sub-menu for swarm operations)
    parser_swarm = subparsers.add_parser('swarm', help="Manage swarm Nanosaur robots")
    swarm_subparsers = parser_swarm.add_subparsers(dest='swarm_type', help="Robot operations")
    # Add robot status subcommand
    parser_robot_new = swarm_subparsers.add_parser('new', help="Get a new robot to control")
    parser_robot_new.add_argument('name', type=str, help="New robot name")
    parser_robot_new.set_defaults(func=robot.robot_new)
    # Add robot set subcommand
    parser_robot_set = swarm_subparsers.add_parser('set', help=f"Set which robot to control [{idx_swarm}]")
    parser_robot_set.add_argument('robot_name', type=str, nargs='?', help="Name of the robot to control")
    parser_robot_set.set_defaults(func=robot.robot_idx_set)
    # Add robot list subcommand
    parser_robot_list = swarm_subparsers.add_parser('list', help="List all robots in the swarm")
    parser_robot_list.set_defaults(func=robot.robot_list)
    return parser_swarm


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
    parser_info.set_defaults(func=info)

    # Subcommand: install (hidden if workspace already exists)
    # if get_workspace_path(params['nanosaur_workspace_name']) is None:
    if 'developer_mode' not in params and not params['developer_mode']:
        parser_install = subparsers.add_parser('install', help="Install the Nanosaur workspace")
    else:
        parser_install = subparsers.add_parser('install')
    # Add simulation install subcommand
    parser_install.add_argument('--developer', '--dev', action='store_true', help="Install developer workspace")
    parser_install.add_argument('--force', action='store_true', help="Force the update")
    parser_install.add_argument('--all-platforms', action='store_true', help="Install for all platforms")
    parser_install.set_defaults(func=install)

    # Subcommand: workspace (with a sub-menu for workspace operations)
    if 'developer_mode' in params and params['developer_mode']:
        # Add workspace subcommand
        parser_workspace = parser_workspace_menu(subparsers)

    # Subcommand: simulation (with a sub-menu for simulation types)
    if device_type == 'desktop':
        # Add simulation subcommand
        parser_simulation = parser_simulation_menu(subparsers, params)

    # Subcommand: robot (with a sub-menu for robot operations)
    robot_data = robot.RobotList.get_robot(params)
    parser_robot = subparsers.add_parser('robot', help=f"Manage the Nanosaur robot [{robot_data.name}]")
    robot_subparsers = parser_robot.add_subparsers(dest='robot_type', help="Robot operations")

    # Add robot drive subcommand
    parser_robot_drive = robot_subparsers.add_parser('drive', help="Drive the robot")
    parser_robot_drive.set_defaults(func=robot.control_keyboard)
    # Add robot start subcommand
    parser_robot_start = robot_subparsers.add_parser('start', help="Start the robot")
    parser_robot_start.add_argument(
        '--container', action='store_true', help="Run from container")
    parser_robot_start.add_argument(
        '--build', action='store_true', help="Rebuild docker before starting")
    parser_robot_start.set_defaults(func=robot.robot_start)
    # Add robot stop subcommand
    parser_robot_stop = robot_subparsers.add_parser('stop', help="Stop the robot")
    parser_robot_stop.set_defaults(func=robot.robot_stop)

    # Add robot name subcommand
    parser_robot_name = robot_subparsers.add_parser('name', help=f"Set the robot name [{robot_data.name}]")
    parser_robot_name.add_argument('name', type=str, nargs='?', help="Name of the robot (default: nanosaur)")
    parser_robot_name.set_defaults(func=robot.robot_set_name)
    # Add robot domain id subcommand
    parser_robot_domain_id = robot_subparsers.add_parser('domain_id', help=f"Set the robot domain ID [{robot_data.domain_id}]")
    parser_robot_domain_id.add_argument('domain_id', type=int, nargs='?', help="Domain ID of the robot (default: 0)")
    parser_robot_domain_id.set_defaults(func=robot.robot_set_domain_id)
    # Add robot reset subcommand
    parser_robot_reset = robot_subparsers.add_parser('reset', help="Reset the robot configuration")
    parser_robot_reset.set_defaults(func=robot.robot_reset)

    if device_type == 'desktop':
        # Subcommand: swarm (with a sub-menu for swarm operations)
        parser_swarm = parser_swarm_menu(subparsers, params)

    # Enable tab completion
    argcomplete.autocomplete(parser)

    # Parse the arguments
    args = parser.parse_args()

    # Handle workspace subcommand without a workspace_type
    if args.command in ['workspace', 'ws'] and args.workspace_type is None:
        parser_workspace.print_help()
        sys.exit(1)

    # Handle install subcommand without an install_type
    if args.command in ['simulation', 'sim'] and args.simulation_type is None:
        parser_simulation.print_help()
        sys.exit(1)

    if args.command in ['robot'] and args.robot_type is None:
        parser_robot.print_help()
        sys.exit(1)

    if args.command in ['swarm'] and args.swarm_type is None:
        parser_swarm.print_help()
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
