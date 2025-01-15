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

import os
import sys
import pexpect
import requests
import subprocess
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, require_sudo_password

ros2_distro = 'humble'
ros2_sources = f'/opt/ros/{ros2_distro}/setup.bash'


def clean_workspace(nanosaur_ws_name, password):
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    # Check if the script is running with sudo
    if os.geteuid() == 0:
        # Get the original user's home directory
        user_home_dir = os.path.expanduser(f"~{os.getenv('SUDO_USER')}")
    else:
        # Get the current user's home directory
        user_home_dir = os.path.expanduser("~")

    # Create the full path for the workspace folder in the user's home
    # directory
    workspace_path = os.path.join(user_home_dir, nanosaur_ws_name)

    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' exists. Cleaning build, install and log folders", color='yellow'))
        result = False
        try:
            child = pexpect.spawn(f"sudo rm -rf {workspace_path}/build {workspace_path}/install {workspace_path}/log", encoding='utf-8', timeout=None)
            # Wait for password prompt with timeout
            index = child.expect(
                ['password for', pexpect.EOF, pexpect.TIMEOUT], timeout=30)
            if index == 0:
                child.logfile = None  # Disable logging to hide password
                child.sendline(password)
                child.logfile = sys.stdout  # Re-enable logging
                # Wait for completion
                child.expect(pexpect.EOF, timeout=300)
                result = True
            elif index == 1:  # Command finished without password prompt
                print("Command finished without asking for a password.")
                result = True
            elif index == 2:  # Timeout
                print(TerminalFormatter.color_text("Error: Sudo prompt timed out. Please try again.", color='red'))
                result = False

            # Stream all command output to the terminal in real time
            child.logfile = sys.stdout

        except pexpect.ExceptionPexpect as e:
            print(TerminalFormatter.color_text(f"Error running rm {str(e)}", color='red'))
            result = False
        finally:
            # Ensure the process is closed
            if child.isalive():
                child.close()
        print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' cleaned up.", color='green'))
        return result
    else:
        print(TerminalFormatter.color_text(f"Folder '{workspace_path}' does not exist.", color='yellow'))
    return False


def get_workspace_path(nanosaur_ws_name):
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    # Check if the script is running with sudo
    if os.geteuid() == 0:
        # Get the original user's home directory
        user_home_dir = os.path.expanduser(f"~{os.getenv('SUDO_USER')}")
    else:
        # Get the current user's home directory
        user_home_dir = os.path.expanduser("~")

    # Create the full path for the workspace folder in the user's home
    # directory
    workspace_path = os.path.join(user_home_dir, nanosaur_ws_name)

    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        return workspace_path
    else:
        return None


def create_workspace(nanosaur_ws_name):
    # Check if the script is running with sudo
    if os.geteuid() == 0:
        # Get the original user's home directory
        user_home_dir = os.path.expanduser(f"~{os.getenv('SUDO_USER')}")
    else:
        # Get the current user's home directory
        user_home_dir = os.path.expanduser("~")

    # Create the full path for the workspace folder in the user's home
    # directory
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
        child = pexpect.spawn(f"bash -c 'source {ros2_sources} && rosdep install --from-paths {folder_path}/src --ignore-src -r -y'", encoding='utf-8', timeout=None)
        # Stream all command output to the terminal in real time
        child.logfile = sys.stdout
        # Wait for password prompt with timeout
        index = child.expect(
            ['password for', pexpect.EOF, pexpect.TIMEOUT], timeout=30)
        if index == 0:
            child.logfile = None  # Disable logging to hide password
            child.sendline(password)
            child.logfile = sys.stdout  # Re-enable logging
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


@require_sudo_password
def clean(platform, params: Params, args, password=None):
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"Nanosaur cleaning on {device_type}", bold=True))
    # Check if the workspace exists
    workspace_path = get_workspace_path(params['nanosaur_workspace_name'])
    if workspace_path is None:
        print(TerminalFormatter.color_text(f"There are no {params['nanosaur_workspace_name']} in this device!", color='red'))
        return False
    # Clean workspace
    clean_workspace(workspace_path, password)
    return True


def update(platform, params: Params, args, password=None):
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    print(TerminalFormatter.color_text(f"Nanosaur updating on {device_type}", bold=True))
    # Check if the workspace exists
    workspace_path = get_workspace_path(params['nanosaur_workspace_name'])
    if workspace_path is None:
        print(TerminalFormatter.color_text(f"There are no {params['nanosaur_workspace_name']} in this device!", color='red'))
        return False
    # Clean workspace if force
    if args.force:
        print(TerminalFormatter.color_text("- Force update", bold=True))
        # Check if the workspace exists
        clean_workspace(workspace_path, password)
    # Build environment
    print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
    if not run_colcon_build(workspace_path):
        return False
# EOF
