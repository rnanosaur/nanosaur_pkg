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
import sys
import requests
import pexpect
import subprocess
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, require_sudo_password
from nanosaur.workspace import get_workspace_path, create_workspace

def download_rosinstall(url, folder_path, file_name):
    # Create the full file path
    file_path = os.path.join(folder_path, file_name)

    # Check if the file already exists
    if os.path.exists(file_path):
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
        return


def run_vcs_import(workspace_path, rosinstall_path):
    try:
        # Run the command and stream the output live
        process = subprocess.Popen(
            f"vcs import {workspace_path}/src < {rosinstall_path}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Stream output live
        for line in process.stdout:
            print(line.decode('utf-8'), end="")  # Print stdout line-by-line

        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line.decode('utf-8'), color='red'), end="")  # Print stderr (errors) in red

        # Check the exit status of the command
        if process.returncode != 0:
            print(TerminalFormatter.color_text(process.returncode, color='red'))
        else:
            print(TerminalFormatter.color_text("Command completed successfully", color='green'))

        return process.returncode == 0

    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False


def run_rosdep(folder_path, password):
    if password is None:
        print(TerminalFormatter.color_text("Error: No password provided.", color='red'))
        return False
    result = False
    try:
        child = pexpect.spawn(
            f"rosdep install --from-paths {folder_path}/src --ignore-src -r -y",
            encoding='utf-8',
            timeout=None)
        # Stream all command output to the terminal in real time
        child.logfile = sys.stdout
        # Wait for password prompt with timeout
        index = child.expect(
            ['password for', pexpect.EOF, pexpect.TIMEOUT], timeout=30)
        if index == 0:
            child.sendline(password)
            # Wait for completion
            child.expect(pexpect.EOF, timeout=300)
            result = True
        elif index == 1:  # Command finished without password prompt
            print("Command finished without asking for a password.")
            result = True
        elif index == 2:  # Timeout
            print(TerminalFormatter.color_text("Error: Sudo prompt timed out. Please try again.", color='red'))
            result = False
    except pexpect.ExceptionPexpect as e:
        print(TerminalFormatter.color_text(f"Error running rosdep: {str(e)}", color='red'))
        result = False
    finally:
        # Ensure the process is closed
        if child.isalive():
            child.close()
    return result


def run_colcon_build(folder_path):
    
    ros2_distro = 'humble'
    ros2_sources = f'/opt/ros/{ros2_distro}/setup.bash'
    
    # Move to the folder_path and run the colcon build command
    try:
        os.chdir(folder_path)
        print(f"Changed directory to: {folder_path}")

        # Run the command and stream the output live
        process = subprocess.Popen(
            f"source {ros2_sources} && colcon build --symlink-install --merge-install",
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Stream output live
        for line in process.stdout:
            print(line.decode('utf-8'), end="")  # Print stdout line-by-line

        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line.decode('utf-8'), color='red'), end="")  # Print stderr (errors) in red

        # Check the exit status of the command
        if process.returncode != 0:
            print(TerminalFormatter.color_text(process.returncode, color='red'))
        else:
            print(TerminalFormatter.color_text("Command completed successfully", color='green'))

        return process.returncode == 0

    except Exception as e:
        print(f"An error occurred while running the colcon build command: {e}")
        return False


def install_basic(platform, params: Params, args, password=None):
    """Perform a basic installation."""
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(f"Nanosaur installation on {device_type}")
    return True


@require_sudo_password
def install_developer(platform, params: Params, args, password=None):
    """Perform the developer installation."""
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"- Nanosaur installation on {device_type}", bold=True))
    # Create workspace
    workspace_path = create_workspace(params['nanosaur_workspace_name'])
    # Download rosinstall for this device
    print(TerminalFormatter.color_text("- Download rosinstall", bold=True))
    branch = params['nanosaur_branch']
    url = f"https://raw.githubusercontent.com/rnanosaur/nanosaur/{branch}/nanosaur/rosinstall/{device_type}.rosinstall"
    rosinstall_path = download_rosinstall(
        url, workspace_path, f"{device_type}.rosinstall")
    # Import workspace
    print(TerminalFormatter.color_text(f"- Import workspace from {device_type}.rosinstall", bold=True))
    # run vcs import to sync the workspace
    vcs_status = run_vcs_import(workspace_path, rosinstall_path)
    if not vcs_status:
        print(TerminalFormatter.color_text("Failed to import workspace", color='red'))
        return False
    # rosdep workspace
    print(TerminalFormatter.color_text(f"- Install all dependencies on workspace {workspace_path}", bold=True))
    if not run_rosdep(workspace_path, password):
        print(TerminalFormatter.color_text("Failed to install dependencies", color='red'))
        return False
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    if not run_colcon_build(workspace_path):
        print(TerminalFormatter.color_text("Failed to build workspace", color='red'))
        return False
    return True


@require_sudo_password
def install_simulation(platform, params: Params, args, password=None):
    """Install simulation tools"""
    force = args.force
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"Nanosaur simulation tools installation on {device_type}", bold=True))
    # Create workspace
    workspace_path = create_workspace(params['nanosaur_workspace_name'])
    # Download rosinstall for this device
    print(TerminalFormatter.color_text("- Download rosinstall", bold=True))
    branch = params['nanosaur_branch']
    url = f"https://raw.githubusercontent.com/rnanosaur/nanosaur/{branch}/nanosaur/rosinstall/simulation.rosinstall"
    rosinstall_path = download_rosinstall(
        url, workspace_path, "simulation.rosinstall")
    # Import workspace
    print(TerminalFormatter.color_text("- Import workspace from simulation.rosinstall", bold=True))
    # run vcs import to sync the workspace
    vcs_status = run_vcs_import(workspace_path, rosinstall_path)
    if not vcs_status:
        print(TerminalFormatter.color_text("Failed to import workspace", color='red'))
        return False
    # rosdep workspace
    print(TerminalFormatter.color_text(f"- Install all dependencies on workspace {workspace_path}", bold=True))
    if not run_rosdep(workspace_path, password):
        print(TerminalFormatter.color_text("Failed to install dependencies", color='red'))
        return False
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    if not run_colcon_build(workspace_path):
        print(
            TerminalFormatter.color_text(
                "Failed to build workspace",
                color='red'))
        return False
    # All fine
    return True


def update(platform, params: Params, args, password=None):
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"Nanosaur updating on {device_type}", bold=True))
    workspace_path = get_workspace_path(params['nanosaur_workspace_name'])
    if workspace_path is None:
        print(TerminalFormatter.color_text(f"There are no {params['nanosaur_workspace_name']} in this device!", color='red'))
        return False
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    if not run_colcon_build(workspace_path):
        return False
# EOF
