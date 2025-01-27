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
import os
import logging
from python_on_whales import docker, DockerClient, DockerException
from nanosaur.utilities import Params, RobotList, get_nanosaur_home, build_env_file
from nanosaur.prompt_colors import TerminalFormatter

# Set up the logger
logger = logging.getLogger(__name__)


def docker_service_run_command(platform, params: Params, service, command=None, name=None, volumes=None):
    """Run a command in the robot container."""

    if command is None:
        command = []
    if volumes is None:
        volumes = []

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    nanosaur_home_path = get_nanosaur_home()
    # Create the full file path
    robot = RobotList.current_robot(params)
    docker_compose_path = os.path.join(nanosaur_home_path, docker_compose)
    env_file_path = os.path.join(nanosaur_home_path, f'{robot.name}.env')

    # Build env file
    build_env_file(params)
    # Create a DockerClient object with the docker-compose file
    nanosaur_compose = DockerClient(compose_files=[docker_compose_path], compose_env_files=[env_file_path])
    print(TerminalFormatter.color_text(f"Running command in the robot {robot.name} container", color='green'))
    try:
        nanosaur_compose.compose.run(service=service, command=command, remove=True, tty=True, name=name, volumes=volumes)
    except DockerException as e:
        print(TerminalFormatter.color_text(f"Error running the command: {e}", color='red'))
        return False
    return True


def docker_robot_start(platform, params: Params, args):
    """Start the docker container."""

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    nanosaur_home_path = get_nanosaur_home()
    # Create the full file path
    robot = RobotList.current_robot(params)
    docker_compose_path = os.path.join(nanosaur_home_path, docker_compose)
    env_file_path = os.path.join(nanosaur_home_path, f'{robot.name}.env')

    # Check which simulation tool is selected only if robot.simulation is true
    if robot.simulation and 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please run simulation set first.", color='red'))
        return False

    # Build env file
    build_env_file(params)

    print(TerminalFormatter.color_text(f"robot {robot.name} starting", color='green'))

    compose_profiles = []
    if args.profile:
        print(TerminalFormatter.color_text(f"Starting with profile: {args.profile}", color='green'))
        compose_profiles = [args.profile]
    # Create a DockerClient object with the docker-compose file
    nanosaur_compose = DockerClient(compose_files=[docker_compose_path], compose_env_files=[env_file_path], compose_profiles=compose_profiles)
    # Start the container in detached mode
    try:
        nanosaur_compose.compose.up(detach=args.detach)
        if not args.detach:
            nanosaur_compose.compose.rm(volumes=True)
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
    robot = RobotList.current_robot(params)
    docker_compose_path = os.path.join(nanosaur_home_path, docker_compose)
    env_file_path = os.path.join(nanosaur_home_path, f'{robot.name}.env')

    # Start the container in detached mode
    simulation_tool = params['simulation_tool'].lower().replace(' ', '-')
    # Create a DockerClient object with the docker-compose file
    nanosaur_compose = DockerClient(compose_files=[docker_compose_path], compose_env_files=[env_file_path])

    # if len(nanosaur_compose.compose.ps()) > 0:
    #    print(TerminalFormatter.color_text(f"The robot {robot.name} is already running.", color='red'))
    #    return False

    # Build env file
    build_env_file(params)

    print(TerminalFormatter.color_text(f"Simulator {simulation_tool} starting", color='green'))
    try:
        nanosaur_compose.compose.up(services=[f'{simulation_tool}'], recreate=False)
        nanosaur_compose.compose.rm(services=[f'{simulation_tool}'], volumes=True)
    except DockerException as e:
        print(TerminalFormatter.color_text(f"Error starting the simulation tool: {e}", color='red'))
        return False


def docker_robot_stop(platform, params: Params, args):
    """Stop the docker container."""

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    nanosaur_home_path = get_nanosaur_home()
    # Create the full file path
    robot = RobotList.current_robot(params)
    docker_compose_path = os.path.join(nanosaur_home_path, docker_compose)
    env_file_path = os.path.join(nanosaur_home_path, f'{robot.name}.env')

    # Create a DockerClient object with the docker-compose file
    nanosaur_compose = DockerClient(compose_files=[docker_compose_path], compose_env_files=[env_file_path])
    if len(nanosaur_compose.compose.ps()) > 0:
        nanosaur_compose.compose.down(volumes=True)
        print(TerminalFormatter.color_text(f"robot {robot.name} stopped", color='green'))
    else:
        print(TerminalFormatter.color_text(f"The robot {robot.name} is not running.", color='red'))
# EOF
