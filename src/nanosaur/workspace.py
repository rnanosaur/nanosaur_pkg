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
from nanosaur.prompt_colors import TerminalFormatter


def clean_workspace(nanosaur_ws_name):
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
        os.system(f"sudo rm -rf {workspace_path}/build {workspace_path}/install {workspace_path}/log")
        print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' cleaned up.", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Folder '{workspace_path}' does not exist.", color='yellow'))


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
# EOF
