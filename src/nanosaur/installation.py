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
from nanosaur.utilities import run_command_live, run_command
NANOSAUR_WS="nanosaur_ws"
NANOSAUR_BRANCH="nanosaur2"


def create_nanosaur_workspace(nanosaur_ws_name):
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


def run_vcs_import(folder_path, file_path):
    # Construct the command to run
    #run_command_live(["vcs", "import", f"{folder_path} < {file_path}"])
    run_command(f"vcs import {folder_path} < {file_path}")


def run_colcon_build(folder_path):
    # Move to the folder_path and run the colcon build command
    try:
        os.chdir(folder_path)
        print(f"Changed directory to: {folder_path}")
        
        # Run the colcon build command with the necessary flags
        return run_command_live(["colcon", "build", "--symlink-install", "--merge-install"])
    
    except Exception as e:
        print(f"An error occurred while running the colcon build command: {e}")
        return False


def install_basic(platform, args):
    """Perform a basic installation."""
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(f"Nanosaur installation on {device_type}")


def install_developer(platform, args):
    """Perform the developer installation."""
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"- Nanosaur installation on {device_type}", bold=True))
    # Create workspace
    workspace_path = create_nanosaur_workspace(NANOSAUR_WS)
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


def install_simulation(platform, args):
    """Install simulaion tools"""
    force = args.force
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"- Nanosaur simulation tools installation on {device_type}", bold=True))
    # Create workspace
    workspace_path = create_nanosaur_workspace(NANOSAUR_WS)
    workspace_path_src = os.path.join(workspace_path, "src")
    # Download rosinstall for this device
    print(TerminalFormatter.color_text(f"- Download rosinstall", bold=True))
    branch = NANOSAUR_BRANCH
    url = f"https://raw.githubusercontent.com/rnanosaur/nanosaur/{branch}/nanosaur/rosinstall/simulation.rosinstall"
    rosinstall_path = download_rosinstall(url, workspace_path, "simulation.rosinstall")
    # Import workspace
    print(TerminalFormatter.color_text("- Import workspace from simulation.rosinstall", bold=True))
    # run vcs import to sync the workspace
    run_command(f"vcs import {workspace_path_src} < {rosinstall_path}")
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    result = run_colcon_build(workspace_path)
    print(result)
# EOF
