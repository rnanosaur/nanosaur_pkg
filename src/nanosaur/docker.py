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

# https://gabrieldemarmiesse.github.io/python-on-whales/
from python_on_whales import docker, DockerClient, DockerException
from nanosaur.utilities import Params, RobotList, get_nanosaur_raw_github_url, get_nanosaur_home, build_env_file, is_env_file
from nanosaur.prompt_colors import TerminalFormatter
import os
import requests


def download_docker_compose(url, folder_path, file_name, force=False) -> str:
    # Create the full file path
    file_path = os.path.join(folder_path, file_name)
    # Check if the file already exists
    if not force and os.path.exists(file_path):
        print(TerminalFormatter.color_text(f"File '{file_name}' already exists in '{folder_path}'. Skip download", color='yellow'))
        return file_path  # Cancel download

    # Send a request to download the file
    response = requests.get(url)

    if response.status_code == 200:
        # Save the file in the workspace folder
        file_path = os.path.join(folder_path, file_name)
        with open(file_path, 'wb') as file:
            file.write(response.content)
        print(TerminalFormatter.color_text(f"File '{file_name}' downloaded successfully to '{folder_path}'.", color='green'))
        return file_path
    else:
        print(TerminalFormatter.color_text(f"Failed to download file. Status code: {response.status_code}", color='red'))
        return None


def create_simple(platform, params: Params, args) -> bool:
    # Determine the device type
    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    # Get the Nanosaur home folder and branch
    nanosaur_raw_url = get_nanosaur_raw_github_url(params)
    url = f"{nanosaur_raw_url}/nanosaur/compose/{docker_compose}"
    nanosaur_home_path = get_nanosaur_home()
    download_docker_compose(url, nanosaur_home_path, docker_compose, force=args.force)
    return True


def docker_start(platform, params: Params, args):
    """Start the docker container."""

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    nanosaur_home_path = get_nanosaur_home()
    # Create the full file path
    docker_compose_path = os.path.join(nanosaur_home_path, docker_compose)
    robot = RobotList.get_robot(params)

    # Check which simulation tool is selected only if robot.simulation is true
    if robot.simulation and 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please run simulation set first.", color='red'))
        return False

    # Build env file
    if not is_env_file():
        print(TerminalFormatter.color_text("Creating the environment file...", color='green'))
        build_env_file(params)
    print(TerminalFormatter.color_text(f"robot {robot.name} starting", color='green'))
    # Create a DockerClient object with the docker-compose file
    nanosaur_compose = DockerClient(compose_files=[docker_compose_path])
    if args.build:
        print(TerminalFormatter.color_text("Building the Docker container...", color='green'))
        nanosaur_compose.compose.build()
    # Start the container in detached mode
    try:
        nanosaur_compose.compose.up(detach=args.detach)
    except DockerException as e:
        print(TerminalFormatter.color_text(f"Error starting the robot: {e}", color='red'))
        return False


def docker_simulator_start(platform, params: Params, args):
    """Start the simulation tools."""

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    nanosaur_home_path = get_nanosaur_home()
    # Create the full file path
    docker_compose_path = os.path.join(nanosaur_home_path, docker_compose)
    robot = RobotList.get_robot(params)
    
    # Check which simulation tool is selected
    if 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please run simulation set first.", color='red'))
        return False

    # Create a DockerClient object with the docker-compose file
    nanosaur_compose = DockerClient(compose_files=[docker_compose_path])
    if len(nanosaur_compose.compose.ps()) > 0:
        print(TerminalFormatter.color_text(f"The robot {robot.name} is already running.", color='red'))
        return False
    # Build env file
    if not is_env_file():
        print(TerminalFormatter.color_text("Creating the environment file...", color='green'))
        build_env_file(params)
    print(TerminalFormatter.color_text(f"robot {robot.name} starting", color='green'))
    # Start the container in detached mode
    simulation_tool = params['simulation_tool'].lower().replace(' ', '_')
    try:
        nanosaur_compose.compose.up(services=[f'nanosaur_{simulation_tool}'])
    except DockerException as e:
        print(TerminalFormatter.color_text(f"Error starting the simulation tool: {e}", color='red'))
        return False


def docker_stop(platform, params: Params, args):
    """Stop the docker container."""

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    nanosaur_home_path = get_nanosaur_home()
    # Create the full file path
    docker_compose_path = os.path.join(nanosaur_home_path, docker_compose)

    robot = RobotList.get_robot(params)
    # Create a DockerClient object with the docker-compose file
    nanosaur_compose = DockerClient(compose_files=[docker_compose_path])
    if len(nanosaur_compose.compose.ps()) > 0:
        nanosaur_compose.compose.down()
        print(TerminalFormatter.color_text(f"robot {robot.name} stopped", color='green'))
    else:
        print(TerminalFormatter.color_text(f"The robot {robot.name} is not running.", color='red'))
# EOF
