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
from python_on_whales import docker, DockerClient
from nanosaur.utilities import Params
from nanosaur import workspace
from nanosaur.prompt_colors import TerminalFormatter
import os


def docker_start(platform, params: Params, args):
    """Start the docker container."""

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_path = workspace.get_workspace_path(params, 'ws_simulation_name')
    if workspace_path is not None:
        # Create a DockerClient object with the docker-compose file
        print(TerminalFormatter.color_text(f"Starting workspace src folder in {workspace_path}", color='yellow'))
        nanosaur_compose = DockerClient(compose_files=[f"{workspace_path}/src/nanosaur_simulations/docker-compose.yml"])
        env_path = f"{workspace_path}/src/nanosaur_simulations/.env"
    else:
        nanosaur_compose = docker
        env_path = ".env"

    uid = os.getuid()
    gid = os.getgid()
    # Create a .env file and save UID and GID
    with open(env_path, 'w') as env_file:
        env_file.write(f"USER_UID={uid}\n")
        env_file.write(f"USER_GID={gid}\n")
        # Check which simulation tool is selected and save it in the .env file
        simulation_tool = params['simulation_tool'].lower().replace(' ', '_')
        env_file.write(f"SIMULATION={simulation_tool}\n")

    print(TerminalFormatter.color_text(f"Starting the Docker container for {simulation_tool}", color='green'))

    if args.build:
        print(TerminalFormatter.color_text("Building the Docker container...", color='green'))
        nanosaur_compose.compose.build()
    # Start the container in detached mode
    nanosaur_compose.compose.up(detach=True)


def docker_stop(platform, params: Params, args):
    """Stop the docker container."""

    if not docker.compose.is_installed():
        print(TerminalFormatter.color_text("Please install Docker and Docker Compose before running the simulation.", color='red'))
        return False

    workspace_path = workspace.get_workspace_path(params, 'ws_simulation_name')
    if workspace_path is not None:
        # Create a DockerClient object with the docker-compose file
        print(TerminalFormatter.color_text(f"Starting docker from container {workspace_path}", color='yellow'))
        nanosaur_compose = DockerClient(compose_files=[f"{workspace_path}/src/nanosaur_simulations/docker-compose.yml"])
        env_path = f"{workspace_path}/src/nanosaur_simulations/.env"
    else:
        nanosaur_compose = docker
        env_path = ".env"

    if len(nanosaur_compose.compose.ps()) > 0:
        nanosaur_compose.compose.down()
    else:
        print(TerminalFormatter.color_text("The Docker container is not running.", color='red'))

    # Remove the .env file
    if os.path.exists(env_path):
        os.remove(env_path)
# EOF
