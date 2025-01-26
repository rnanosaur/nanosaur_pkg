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

import os
import inquirer
from inquirer.themes import GreenPassion
import argparse
import subprocess
from nanosaur import workspace
from nanosaur.docker import docker_simulator_start
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList


# Dictionary of simulation tools and their commands
simulation_tools = {
    "isaac-sim": {
        "simulator": "ros2 launch isaac_sim_wrapper isaac_sim_server.launch.py",
        "robot": "ros2 launch nanosaur_isaac_sim nanosaur_bridge.launch.py"
    },
    "gazebo": {
        "simulator": "ros2 launch nanosaur_gazebo gazebo.launch.py",
        "robot": "ros2 launch nanosaur_gazebo nanosaur_bridge.launch.py"
    }
}


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
    parser_simulation_start.add_argument(
        '--debug', action='store_true', help="Start the simulation in debug mode")
    parser_simulation_start.set_defaults(func=simulation_start)

    # Add simulation set subcommand
    parser_simulation_set = simulation_subparsers.add_parser(
        'set', help="Select the simulator you want to use")
    parser_simulation_set.set_defaults(func=simulation_set)
    return parser_simulation


def simulation_robot_start_debug(params):
    nanosaur_ws_path = workspace.get_workspace_path(params, 'ws_simulation_name')
    bash_file = os.path.join(nanosaur_ws_path, 'install', 'setup.bash')
    # Check if the install folder exists
    if not os.path.exists(bash_file):
        print(TerminalFormatter.color_text("Workspace not built. Build before to debug", color='red'))
        return False
    # Check which simulation tool is selected
    if 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please select a simulator first.", color='red'))
        return False
    # Check if the simulation tool is valid and get the command
    command = simulation_tools[params['simulation_tool']]['robot']
    # Load the robot configuration
    robot = RobotList.get_robot(params)
    print(TerminalFormatter.color_text(f"Starting {robot}", color='green'))

    # Print the command to be run
    print(f"ROS_DOMAIN_ID={robot.domain_id} {command} {robot.config_to_ros()}")

    try:
        # Combine sourcing the bash file with running the command
        process = subprocess.Popen(
            f"source {bash_file} && ROS_DOMAIN_ID={robot.domain_id} {command} {robot.config_to_ros()}",
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Stream output live
        for line in process.stdout:
            # Decode and print stdout line-by-line
            print(line.decode('utf-8'), end="")

        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line.decode('utf-8'), color='red'), end="")  # Print stderr (errors) in red

        return process.returncode == 0
    except KeyboardInterrupt:
        return False
    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False


def simulation_start_debug(simulation_ws_path, simulation_tool):
    """Install the simulation tools."""

    bash_file = f'{simulation_ws_path}/install/setup.bash'
    # Check if the install folder exists
    if not os.path.exists(bash_file):
        print(TerminalFormatter.color_text("Workspace not built. Build before to debug", color='red'))
        return False

    command = simulation_tools[simulation_tool]['simulator']

    try:
        # Combine sourcing the bash file with running the command
        process = subprocess.Popen(
            f"source {bash_file} && {command}",
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Stream output live
        for line in process.stdout:
            # Decode and print stdout line-by-line
            print(line.decode('utf-8'), end="")

        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line.decode('utf-8'), color='red'), end="")  # Print stderr (errors) in red

        return process.returncode == 0
    except KeyboardInterrupt:
        return False
    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False


def simulation_start(platform, params: Params, args):
    # Check if debug mode is enabled
    debug_mode = None
    if 'debug' in params:
        debug_mode = params['debug']
        print(TerminalFormatter.color_text(f"Default debug mode: {debug_mode}", color='yellow'))

    # Check which simulation tool is selected
    if 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please run simulation set first.", color='red'))
        return False

    if params['simulation_tool'] not in simulation_tools:
        print(TerminalFormatter.color_text(f"Unknown simulation tool: {params['simulation_tool']}", color='red'))
        return False

    if debug_mode:
        nanosaur_ws_path = workspace.get_workspace_path(params, 'ws_simulation_name')
        simulator_tool = params['simulation_tool']
        return simulation_start_debug(nanosaur_ws_path, simulator_tool)
    # Run from docker container
    return docker_simulator_start(platform, params, args)


def simulation_set(platform, params: Params, args):
    """Set the simulation tools."""

    questions = [
        inquirer.List(
            'simulation_tool',
            message="Set the simulation tools:",
            choices=[tool.capitalize() for tool in simulation_tools.keys()],
            default=params.get('simulation_tool', None)
        )
    ]
    # Ask the user to select a simulation tool
    answers = inquirer.prompt(questions, theme=GreenPassion())
    if answers is None:
        return False
    # Save the selected simulation tool
    params['simulation_tool'] = answers['simulation_tool'].lower()
    print(TerminalFormatter.color_text(f"Selected {answers['simulation_tool']}", color='green'))
    return True
# EOF
