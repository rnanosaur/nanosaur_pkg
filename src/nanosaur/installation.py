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
import requests
import subprocess
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import run_colcon_build, run_rosdep, run_command
NANOSAUR_WS="nanosaur_ws"
NANOSAUR_BRANCH="nanosaur2"


def get_workspace_path(nanosaur_ws_name):
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    # Get the user's home directory
    user_home_dir = os.path.expanduser("~")
    
    # Create the full path for the workspace folder in the user's home directory
    workspace_path = os.path.join(user_home_dir, nanosaur_ws_name)
    
    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        return workspace_path
    else:
        return None


def create_workspace(nanosaur_ws_name):
    # Get the user's home directory
    user_home_dir = os.path.expanduser("~")
    
    # Create the full path for the workspace folder in the user's home directory
    workspace_path = os.path.join(user_home_dir, nanosaur_ws_name)
    workspace_path_src = os.path.join(workspace_path, "src")

    # Check if folder exists, if not, create it
    if not os.path.exists(workspace_path_src):
        os.makedirs(workspace_path_src)
        print(TerminalFormatter.color_text(f"Folder '{workspace_path_src}' created.", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Folder '{workspace_path_src}' already exists.", color='yellow'))
    
    return workspace_path


def download_rosinstall(url, folder_path, file_name):
    # Create the full file path
    file_path = os.path.join(folder_path, file_name)
    
    # Check if the file already exists
    if os.path.exists(file_path):
        print(TerminalFormatter.color_text(f"File '{file_name}' already exists in '{folder_path}'. Skip download", color='yellow'))
        return file_path # Cancel download
    
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
        return 


def install_basic(platform, args):
    """Perform a basic installation."""
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(f"Nanosaur installation on {device_type}")
    return True


def install_developer(platform, args):
    """Perform the developer installation."""
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"- Nanosaur installation on {device_type}", bold=True))
    # Create workspace
    workspace_path = create_workspace(NANOSAUR_WS)
    workspace_path_src = os.path.join(workspace_path, "src")
    # Download rosinstall for this device
    print(TerminalFormatter.color_text(f"- Download rosinstall", bold=True))
    branch = NANOSAUR_BRANCH
    url = f"https://raw.githubusercontent.com/rnanosaur/nanosaur/{branch}/nanosaur/rosinstall/{device_type}.rosinstall"
    rosinstall_path = download_rosinstall(url, workspace_path, f"{device_type}.rosinstall")
    # Import workspace
    print(TerminalFormatter.color_text(f"- Import workspace from {device_type}.rosinstall", bold=True))
    # run vcs import to sync the workspace
    run_command(f"vcs import {workspace_path_src} < {rosinstall_path}")
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    result = run_colcon_build(workspace_path)
    print(result)
    return True


def install_simulation(platform, args):
    """Install simulation tools"""
    force = args.force
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"Nanosaur simulation tools installation on {device_type}", bold=True))
    # Create workspace
    workspace_path = create_workspace(NANOSAUR_WS)
    workspace_path_src = os.path.join(workspace_path, "src")
    # Download rosinstall for this device
    print(TerminalFormatter.color_text(f"- Download rosinstall", bold=True))
    branch = NANOSAUR_BRANCH
    url = f"https://raw.githubusercontent.com/rnanosaur/nanosaur/{branch}/nanosaur/rosinstall/simulation.rosinstall"
    rosinstall_path = download_rosinstall(url, workspace_path, "simulation.rosinstall")
    # Import workspace
    print(TerminalFormatter.color_text("- Import workspace from simulation.rosinstall", bold=True))
    # run vcs import to sync the workspace
    if not run_command(f"vcs import {workspace_path_src} < {rosinstall_path}"):
        return False
    # rosdep workspace
    print(TerminalFormatter.color_text(f"- Install all dependencies on workspace {workspace_path}", bold=True))
    if not run_rosdep(workspace_path):
        return False
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    if not run_colcon_build(workspace_path):
        return False
    # All fine
    return True

def update(platform, args):
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"Nanosaur updating on {device_type}", bold=True))
    workspace_path = get_workspace_path(NANOSAUR_WS)
    if workspace_path is None:
        print(TerminalFormatter.color_text(f"There are no {NANOSAUR_WS} in this device!", color='red'))
        return False
    workspace_path_src = f"{workspace_path}/src"
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    if not run_colcon_build(workspace_path):
        return False
# EOF
