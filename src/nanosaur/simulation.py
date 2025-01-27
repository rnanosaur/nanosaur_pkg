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
import shutil
import inquirer
from inquirer.themes import GreenPassion
import argparse
import subprocess
import logging
from nanosaur import workspace
from nanosaur.ros import get_ros2_path
from nanosaur.docker import docker_simulator_start
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList

# Set up the logger
logger = logging.getLogger(__name__)

# Dictionary of simulation tools and their commands
simulation_tools = {
    "isaac-sim": {
        "simulator": "ros2 launch isaac_sim_wrapper isaac_sim_server.launch.py",
        "robot": "ros2 launch nanosaur_isaac-sim nanosaur_bridge.launch.py"
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

    # Add simulation headless subcommand
    parser_simulation_headless = simulation_subparsers.add_parser(
        'headless', help="Set the simulation in headless mode")
    parser_simulation_headless.set_defaults(func=simulation_set_headless)
    return parser_simulation


def find_all_isaac_sim():
    # Path where Isaac Sim is usually installed
    base_path = os.path.expanduser("~/.local/share/ov/pkg")
    isaac_sim_folders = {}

    if os.path.exists(base_path):
        # Look for directories that contain "isaac-sim" in their names
        for folder in os.listdir(base_path):
            if "isaac-sim" in folder:
                version = folder.split("isaac-sim-")[-1]  # Extract version after "isaac-sim-"
                full_path = os.path.join(base_path, folder)
                isaac_sim_folders[version] = full_path
    # Return a dictionary with the version as key and the full path as value
    return isaac_sim_folders


def is_gazebo_installed(folder="/usr/share/gazebo"):
    """
    Check if Gazebo is installed by verifying the existence of the Gazebo binary
    or the installation folder.

    :param folder: Path to the folder where Gazebo is typically installed (default: /usr/share/gazebo).
    :return: True if Gazebo is installed, False otherwise.
    """
    if shutil.which("gazebo") or shutil.which("gz"):
        return True
    # Check if the default Gazebo folder exists
    return bool(os.path.exists(folder) and os.path.isdir(folder))


def is_simulation_tool_installed():
    """
    Check if either Gazebo or Isaac Sim is installed.

    :return: A dictionary indicating the installation status of Gazebo and Isaac Sim.
    """
    return bool(find_all_isaac_sim()) or is_gazebo_installed()


def simulation_info(platform, params: Params, verbose):
    """
    Print information about the installed simulation tools.
    """

    def print_simulation_tool():
        isaac_sim_version = ""
        if 'isaac_sim_path' in params and params['simulation_tool'] == 'isaac-sim' and params['isaac_sim_path']:
            isaac_sim_version = params['isaac_sim_path'].split("isaac-sim-")[-1]  # Extract version after "isaac-sim-"
        text_message = f"{TerminalFormatter.color_text('   selected:', bold=True)} {params['simulation_tool']} {isaac_sim_version}"
        print(text_message)
        headless_md = params.get('simulation_headless', False)
        headless_string = TerminalFormatter.color_text('enabled', color='green') if headless_md else TerminalFormatter.color_text('disabled', color='red')
        print(f"{TerminalFormatter.color_text('   Headless mode:', bold=True)} {headless_string}")
        if headless_md and params['simulation_tool'] == 'isaac-sim':
            hostname = f"{os.uname()[1]}.local"
            link_webrtc = TerminalFormatter.clickable_link(f"http://{hostname}:8211/streaming/webrtc-client/")
            print(f"{TerminalFormatter.color_text('   WebRTC:', bold=True)} {link_webrtc}")

    # Check if any simulation tools are installed
    if not is_simulation_tool_installed():
        print(TerminalFormatter.color_text("No simulation tools installed", color='red'))
        return

    print(TerminalFormatter.color_text("Simulation:", bold=True))
    if 'simulation_tool' in params:
        print_simulation_tool()

    elif platform['Machine'] != 'aarch64':
        print(TerminalFormatter.color_text("   No simulation tool selected", color='red'))

    # Check if Isaac Sim is installed
    if verbose:
        if isaac_sim_list := find_all_isaac_sim():
            print(TerminalFormatter.color_text("   Isaac Sim installed:", bold=True))
            for version, path in isaac_sim_list.items():
                print(f"    - Isaac Sim {version}: {path}")
        # Check if Gazebo is installed
        if is_gazebo_installed():
            print(TerminalFormatter.color_text("   Gazebo is installed", bold=True))


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
    robot = RobotList.current_robot(params)
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


def simulation_start_debug(simulation_ws_path, simulation_tool, headless, isaac_sim_path=None):
    """Install the simulation tools."""

    bash_file = f'{simulation_ws_path}/install/setup.bash'
    # Check if the install folder exists
    if not os.path.exists(bash_file):
        print(TerminalFormatter.color_text("Workspace not built. Build before to debug", color='red'))
        return False

    command = simulation_tools[simulation_tool]['simulator']
    command += f" headless:={str(headless).lower()}"
    # add isaac_sim_path if available
    if isaac_sim_path:
        command = f"{command} isaac_sim_path:={isaac_sim_path}"
    logger.debug(command)
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
    if 'ws_debug' in params:
        debug_mode = params['ws_debug']
        print(TerminalFormatter.color_text(f"Default debug mode: {debug_mode}", color='yellow'))
    # Get the ROS 2 installation path if available
    ros2_installed = get_ros2_path(workspace.ROS_DISTRO)
    debug_mode = 'docker' if ros2_installed is None else debug_mode
    # Check which simulation tool is selected
    if 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please run simulation set first.", color='red'))
        return False
    # Check if the simulation tool is valid
    if params['simulation_tool'] not in simulation_tools:
        print(TerminalFormatter.color_text(f"Unknown simulation tool: {params['simulation_tool']}", color='red'))
        return False
    # Check if Isaac Sim is selected but no version is set
    if params['simulation_tool'] == 'isaac-sim' and 'isaac_sim_path' not in params:
        print(TerminalFormatter.color_text("No Isaac Sim version selected. Please run simulation set first.", color='red'))
        return False
    # Check if the debug mode is enabled
    if debug_mode == 'host':
        nanosaur_ws_path = workspace.get_workspace_path(params, 'ws_simulation_name')
        simulator_tool = params['simulation_tool']
        headless = params.get('simulation_headless', False)
        return simulation_start_debug(nanosaur_ws_path, simulator_tool, headless, isaac_sim_path=params.get('isaac_sim_path', None))
    elif debug_mode == 'docker':
        # Run from docker container
        return docker_simulator_start(platform, params, args)
    else:
        print(TerminalFormatter.color_text(f"Unknown debug mode: {debug_mode}", color='red'))
        return False


def simulation_set(platform, params: Params, args):
    """Set the simulation tools."""
    # Get the current simulation tool
    current_tool = params.get('simulation_tool', None)
    # Capitalize the current tool name
    if current_tool:
        current_tool = current_tool.capitalize()
    # Check if Gazebo is installed
    if not is_gazebo_installed():
        simulation_tools.pop('gazebo', None)
    # Find all installed Isaac Sim versions
    isaac_sim_list = find_all_isaac_sim()
    # Get the version of Isaac Sim if it is already set
    version = None
    if 'isaac_sim_path' in params:
        version = params['isaac_sim_path'].split("isaac-sim-")[-1]  # Extract version after "isaac-sim-"
    # Check if any simulation tools are available
    if not simulation_tools:
        print(TerminalFormatter.color_text("No simulation tools available. Please install a simulator first.", color='red'))
        return False
    # Ask the user to select a simulation tool
    questions = [
        inquirer.List(
            'simulation_tool',
            message="Set the simulation tools",
            choices=[tool.capitalize() for tool in simulation_tools.keys()],
            default=current_tool
        ),
        inquirer.List(
            'isaac-sim',
            message="Select Isaac Sim version for run on host",
            choices=list(isaac_sim_list.keys()),
            default=version,
            ignore=lambda answers: answers['simulation_tool'] != 'Isaac-sim' or not isaac_sim_list
        )
    ]
    # Ask the user to select a simulation tool
    answers = inquirer.prompt(questions, theme=GreenPassion())
    if answers is None:
        return False
    # Save the selected simulation tool
    params['simulation_tool'] = answers['simulation_tool'].lower()
    if params['simulation_tool'] == 'isaac-sim' and answers['isaac-sim'] is not None:
        params['isaac_sim_path'] = isaac_sim_list[answers['isaac-sim']]
        print(TerminalFormatter.color_text(f"Selected Isaac Sim version: {answers['isaac-sim']}", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Selected {answers['simulation_tool']}", color='green'))
    return True


def simulation_set_headless(platform, params: Params, args):
    # Get the current simulation tool
    headless_mode = params.get('simulation_headless', False)
    # Ask the user if they want to run in headless mode
    question = [
        inquirer.List(
            'headless',
            message="Select if you want run the simulation in headless mode",
            choices=['Yes', 'No'],
            default='Yes' if headless_mode else 'No'
        )
    ]
    # Get the user's answer
    answer = inquirer.prompt(question, theme=GreenPassion())
    if answer is None:
        return False
    # Save the headless mode setting
    params['simulation_headless'] = (answer['headless'] == 'Yes')
    print(TerminalFormatter.color_text(f"Headless mode set to: {answer['headless']}", color='green'))
    return True
