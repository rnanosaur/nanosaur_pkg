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
import yaml
import argparse
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur import ros
from nanosaur.utilities import Params, get_nanosaur_home, create_nanosaur_home, require_sudo_password, conditional_sudo_password


# Default colcon settings
COLCON_DEFAULTS = {
    'build': {
        'base-paths': ['src', '../shared_src'],
        'symlink-install': True,
    }
}


def parser_workspace_menu(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser_workspace = subparsers.add_parser(
        'workspace', aliases=["ws"], help="Manage the Nanosaur workspace")
    workspace_subparsers = parser_workspace.add_subparsers(
        dest='workspace_type', help="Workspace types")
    # Add workspace clean subcommand

    def add_workspace_subcommand(name, help_text, func):
        parser = workspace_subparsers.add_parser(name, help=help_text)
        parser.add_argument('workspace', type=str, nargs='?', help="Specify the workspace to clean")
        parser.add_argument('--force', action='store_true', help="Force the workspace clean")
        parser.add_argument('--all-platforms', '--all', action='store_true', help="Clean all workspaces")
        parser.set_defaults(func=func)
        return parser
    # Add workspace clean subcommand
    add_workspace_subcommand('clean', "Clean the workspace", clean)
    add_workspace_subcommand('update', "Update the workspace", update)
    add_workspace_subcommand('run', "Run the workspace", run_developer_workspace)
    add_workspace_subcommand('deploy', "Deploy workspace to docker image", deploy)
    return parser_workspace


def run_developer_workspace(platform, params: Params, args, password=None):
    if args.workspace is not None:
        workspace = args.workspace
    else:
        workspace = "robot" if platform['Machine'] == 'jetson' else "desktop"

    if workspace == 'perception':
        perception_ws_name = params['ws_perception_name']
        perception_ws_path = get_workspace_path(params, perception_ws_name)
        ros.run_dev_script(params, perception_ws_path)
    else:
        print(TerminalFormatter.color_text(f"I cannot run {workspace}", color='red'))


def get_workspaces_path(params: Params) -> bool:
    nanosaur_home_path = get_nanosaur_home(params['nanosaur_home'])
    # Add all workspaces that exist in the Nanosaur home folder
    return [
        workspace for workspace in [
            'ws_developer_name', 'ws_robot_name', 'ws_simulation_name', 'ws_perception_name'
        ] if os.path.exists(os.path.join(nanosaur_home_path, params[workspace]))
    ]


def get_workspace_path(params: Params, ws_name) -> str:
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home(params['nanosaur_home'])
    # Create the full path for the workspace folder in the user's home directory
    workspace_path = os.path.join(nanosaur_home_path, ws_name)

    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        return workspace_path
    else:
        return None


def create_workspace(nanosaur_home_path, ws_name, skip_create_colcon_setting=False) -> str:
    ws_name_path = os.path.join(nanosaur_home_path, ws_name)
    ws_name_path_src = os.path.join(ws_name_path, "src")
    # Check if folder exists, if not, create it
    if not os.path.exists(ws_name_path_src):
        os.makedirs(ws_name_path_src)
        print(TerminalFormatter.color_text(f"Workspace '{ws_name}' created in {nanosaur_home_path}.", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Workspace '{ws_name}' already exists.", color='yellow'))
    # Save the default colcon settings
    if not skip_create_colcon_setting:
        with open(f"{ws_name_path}/colcon_defaults.yaml", 'w') as file:
            yaml.dump(COLCON_DEFAULTS, file)
    return ws_name_path


def clean_workspace(nanosaur_ws_name, password) -> bool:
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    # Get the current user's home directory
    user_home_dir = os.path.expanduser("~")

    # Create the full path for the workspace folder in the user's home
    # directory
    workspace_path = os.path.join(user_home_dir, nanosaur_ws_name)

    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        subfolders = ['build', 'install', 'log']
        subfolders_exist = all(os.path.exists(os.path.join(workspace_path, subfolder)) for subfolder in subfolders)

        if subfolders_exist:
            print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' and subfolders exist. Cleaning build, install and log folders", color='yellow'))
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


def build_workspace(nanosaur_raw_github_repo, branch, workspace_path, rosinstall_name, password, skip_rosdep=False, skip_build=False) -> bool:
    # Download rosinstall for this device
    url = f"{nanosaur_raw_github_repo}/{branch}/nanosaur/rosinstall/{rosinstall_name}.rosinstall"
    rosinstall_path = ros.download_rosinstall(url, workspace_path, f"{rosinstall_name}.rosinstall")
    if rosinstall_path is not None:
        print(TerminalFormatter.color_text(f"- Fill {rosinstall_name} from {rosinstall_name}.rosinstall", bold=True))
    else:
        print(TerminalFormatter.color_text(f"Failed to download {rosinstall_name}.rosinstall Exiting...", color='red'))
        return False
    # Import workspace
    print(TerminalFormatter.color_text(f"- Import workspace from {rosinstall_name}.rosinstall", bold=True))
    # run vcs import to sync the workspace
    vcs_status = ros.run_vcs_import(workspace_path, rosinstall_path)
    if not vcs_status:
        print(TerminalFormatter.color_text("Failed to import workspace", color='red'))
        return False
    # rosdep workspace
    if not skip_rosdep:
        print(TerminalFormatter.color_text(f"- Install all dependencies on workspace {workspace_path}", bold=True))
        if not ros.run_rosdep(workspace_path, password):
            print(TerminalFormatter.color_text("Failed to install dependencies", color='red'))
            return False
    # Build environment
    if not skip_build:
        print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
        if not ros.run_colcon_build(workspace_path):
            print(TerminalFormatter.color_text("Failed to build workspace", color='red'))
            return False
    # All fine
    return True


def create_developer_workspace(platform, params: Params, args, password=None) -> bool:
    # Get the Nanosaur home folder and branch
    nanosaur_home = params['nanosaur_home']
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home(nanosaur_home)
    # Create developer workspace
    create_workspace(nanosaur_home_path, params['ws_developer_name'], skip_create_colcon_setting=True)
    return True


@require_sudo_password
def create_maintainer_workspace(platform, params: Params, args, password=None):
    # determine the device type
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    # Get the Nanosaur home folder and branch
    nanosaur_home = params['nanosaur_home']
    nanosaur_raw_github_repo = params['nanosaur_raw_github_repo']
    branch = params['nanosaur_branch']
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home(nanosaur_home)

    # Create the shared source folder
    nanosaur_shared_src = os.path.join(nanosaur_home_path, "shared_src")
    # Check if folder exists, if not, create it
    if not os.path.exists(nanosaur_shared_src):
        os.makedirs(nanosaur_shared_src)
        print(TerminalFormatter.color_text(f"Shared src folder created in {nanosaur_home_path}.", color='green'))
    # Download rosinstall for this device
    url = f"{nanosaur_raw_github_repo}/{branch}/nanosaur/rosinstall/shared.rosinstall"
    rosinstall_path = ros.download_rosinstall(url, nanosaur_shared_src, "shared.rosinstall")
    if rosinstall_path is not None:
        print(TerminalFormatter.color_text("- Fill shared src from shared.rosinstall", bold=True))
    else:
        print(TerminalFormatter.color_text("Failed to download rosinstall file. Exiting...", color='red'))
        return False
    # Import workspace
    print(TerminalFormatter.color_text("- Import workspace from shared.rosinstall", bold=True))
    # run vcs import to sync the workspace
    vcs_status = ros.run_vcs_import(nanosaur_home_path, rosinstall_path, src_folder="shared_src")
    if not vcs_status:
        print(TerminalFormatter.color_text("Failed to import workspace", color='red'))
        return False

    # Make the robot workspace
    if device_type == "robot" or args.all_platforms:
        # Make the robot workspace
        ws_name_path = create_workspace(nanosaur_home_path, params['ws_robot_name'])
        if not build_workspace(nanosaur_raw_github_repo, branch, ws_name_path, device_type, password):
            return False
    # Make the simulation workspace
    if device_type == "desktop" or args.all_platforms:
        # Make the simulation workspace
        ws_name_path = create_workspace(nanosaur_home_path, params['ws_simulation_name'])
        if not build_workspace(nanosaur_raw_github_repo, branch, ws_name_path, device_type, password):
            return False

    # Make the perception workspace
    ws_name_path = create_workspace(nanosaur_home_path, params['ws_perception_name'])
    return build_workspace(nanosaur_raw_github_repo, branch, ws_name_path, 'perception', password, skip_rosdep=True, skip_build=True)


@require_sudo_password
def clean(platform, params: Params, args, password=None):
    if args.workspace is not None:
        workspace = args.workspace
    else:
        workspace = "robot" if platform['Machine'] == 'jetson' else "desktop"

    if workspace == 'robot' or args.all_platforms:
        robot_ws_name = params['ws_robot_name']
        robot_ws_path = get_workspace_path(params, robot_ws_name)
        if robot_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Clean robot workspace {robot_ws_name}", bold=True))
            # Clean workspace
            clean_workspace(robot_ws_path, password)

    if workspace == 'desktop' or args.all_platforms:
        simulation_ws_name = params['ws_simulation_name']
        simulation_ws_path = get_workspace_path(params, simulation_ws_name)
        if simulation_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Clean simulation workspace {simulation_ws_name}", bold=True))
            # Clean workspace
            clean_workspace(simulation_ws_path, password)

    if workspace == 'perception' or args.all_platforms:
        perception_ws_name = params['ws_perception_name']
        perception_ws_path = get_workspace_path(params, perception_ws_name)
        if perception_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Clean perception workspace {perception_ws_name}", bold=True))
            # Clean workspace
            clean_workspace(perception_ws_path, password)

    return True


@conditional_sudo_password
def update(platform, params: Params, args, password=None):
    if args.workspace is not None:
        workspace = args.workspace
    else:
        workspace = "robot" if platform['Machine'] == 'jetson' else "desktop"

    def update_workspace(params, workspace_type, workspace_name_key, skip_build=False):
        workspace_name = params[workspace_name_key]
        workspace_path = get_workspace_path(params, workspace_name)
        # Check if the workspace folder exists, vcs update and build
        if workspace_path:
            rosinstall_path = os.path.join(workspace_path, f"{workspace_type}.rosinstall")
            if os.path.exists(rosinstall_path):
                print(TerminalFormatter.color_text(f"Found rosinstall file: {rosinstall_path}", bold=True))
                if not ros.run_vcs_import(workspace_path, rosinstall_path):
                    return False
            # rosdep workspace
            if not skip_build:
                print(TerminalFormatter.color_text(f"- Update {workspace_name} workspace", bold=True))
                if not ros.run_colcon_build(workspace_path):
                    return False
        return True

    # Update shared workspace
    nanosaur_home_path = get_nanosaur_home(params['nanosaur_home'])
    shared_src_path = os.path.join(nanosaur_home_path, "shared_src")
    rosinstall_path = os.path.join(shared_src_path, "shared.rosinstall")
    if os.path.exists(rosinstall_path):
        print(TerminalFormatter.color_text(f"Found rosinstall file: {rosinstall_path}", bold=True))
        if not ros.run_vcs_import(nanosaur_home_path, rosinstall_path, src_folder="shared_src"):
            return False

    # Update the robot workspace
    if workspace == 'robot' or args.all_platforms:
        update_workspace(params, 'robot', 'ws_robot_name')

    # Update the simulation workspace
    if workspace == 'desktop' or args.all_platforms:
        update_workspace(params, 'desktop', 'ws_simulation_name')

    if workspace == 'perception' or args.all_platforms:
        update_workspace(params, 'perception', 'ws_perception_name', skip_build=True)

    return True


def deploy(platform, params: Params, args, password=None):
    if args.workspace is not None:
        workspace = args.workspace
    else:
        workspace = "robot" if platform['Machine'] == 'jetson' else "desktop"

    if workspace == 'robot' or args.all_platforms:
        print(TerminalFormatter.color_text("Robot deploy. Not implemented", color='yellow'))

    if workspace == 'desktop' or args.all_platforms:
        print(TerminalFormatter.color_text("Desktop deploy. Not implemented", color='yellow'))

    # Call the function within the deploy function
    if workspace == 'perception' or args.all_platforms:
        perception_ws_name = params['ws_perception_name']
        perception_ws_path = get_workspace_path(params, perception_ws_name)
        ros.deploy_docker_perception(params, perception_ws_path)

    return True
# EOF
