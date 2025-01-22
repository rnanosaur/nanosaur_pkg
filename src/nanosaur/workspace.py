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
import yaml
import argparse
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur import ros
from nanosaur.simulation import start_robot_simulation
from nanosaur.utilities import Params, get_nanosaur_raw_github_url, get_nanosaur_home, create_nanosaur_home, require_sudo_password
import inquirer


# Default colcon settings
COLCON_DEFAULTS = {
    'build': {
        'base-paths': ['src', '../shared_src'],
        'symlink-install': True,
        'merge-install': True,
    }
}

DEFAULT_WORKSPACE_PERCEPTION = 'perception_ws'
DEFAULT_WORKSPACE_SIMULATION = 'simulation_ws'
DEFAULT_WORKSPACE_ROBOT = 'ros_ws'
DEFAULT_WORKSPACE_DEVELOPER = 'developer_ws'


def parser_workspace_menu(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser_workspace = subparsers.add_parser(
        'workspace', aliases=["ws"], help="Manage the Nanosaur workspace")
    workspace_subparsers = parser_workspace.add_subparsers(
        dest='workspace_type', help="Workspace types")
    # Add workspace clean subcommand

    def add_workspace_subcommand(name, help_text, func):
        parser = workspace_subparsers.add_parser(name, help=help_text)
        parser.add_argument('workspace', type=str, nargs='?', help="Specify the workspace")
        parser.add_argument('--force', action='store_true', help="Force operation")
        parser.add_argument('--all', action='store_true', help="Use all workspaces")
        parser.set_defaults(func=func)
        return parser
    # Add workspace clean subcommand
    add_workspace_subcommand('clean', "Clean the workspace", clean)
    add_workspace_subcommand('update', "Update the workspace", update)
    add_workspace_subcommand('build', "Build the workspace", build)
    add_workspace_subcommand('debug', "Debug the workspace", debug)
    add_workspace_subcommand('deploy', "Deploy workspace to docker image", deploy)
    return parser_workspace


def get_selected_workspace(params, workspace_actions, args):
    # Check if the workspace is provided as an argument
    if args.workspace is not None:
        return args.workspace
    # Get the workspaces
    workspaces = get_workspaces_path(params)
    workspaces = {k: v for k, v in workspaces.items() if k in workspace_actions}
    # Check if there are any workspaces
    if not workspaces:
        print(TerminalFormatter.color_text("No workspaces found.", color='red'))
        return None
    # Ask the user to select a workspace
    questions = [
        inquirer.List('workspace', message="Select a workspace", choices=workspaces),
    ]
    answers = inquirer.prompt(questions)
    return answers['workspace'] if answers else None


def clean(platform, params: Params, args):
    """ Clean the workspace """
    workspace_actions = {
        'developer': lambda: clean_workspace(params.get('ws_developer_name', DEFAULT_WORKSPACE_DEVELOPER)),
        'robot': lambda: clean_workspace(params.get('ws_robot_name', DEFAULT_WORKSPACE_ROBOT)),
        'simulation': lambda: clean_workspace(params.get('ws_simulation_name', DEFAULT_WORKSPACE_SIMULATION)),
        'perception': lambda: clean_workspace(params.get('ws_perception_name', DEFAULT_WORKSPACE_PERCEPTION))
    }
    if args.all:
        print(TerminalFormatter.color_text("Cleaning all workspaces", bold=True))
        results = [action() for action in workspace_actions.values()]
        return all(results)
    # Get the workspace
    workspace = get_selected_workspace(params, workspace_actions, args)
    if workspace is None:
        return False
    # Clean the workspace
    print(TerminalFormatter.color_text(f"Cleaning {workspace}", bold=True))
    if action := workspace_actions.get(workspace):
        return action()
    print(TerminalFormatter.color_text(f"I cannot clean this {workspace}", color='red'))
    return False


def update(platform, params: Params, args):
    """ Update the workspace """
    # Get the Nanosaur home folder and branch
    nanosaur_raw_url = get_nanosaur_raw_github_url(params)
    # Update shared workspace

    def update_shared_workspace(force):
        nanosaur_home_path = get_nanosaur_home()
        shared_src_path = os.path.join(nanosaur_home_path, "shared_src")
        rosinstall_path = os.path.join(shared_src_path, "shared.rosinstall")
        workspace_type = 'shared'
        # Download rosinstall for this device
        url = f"{nanosaur_raw_url}/nanosaur/rosinstall/{workspace_type}.rosinstall"
        rosinstall_path = ros.download_rosinstall(url, shared_src_path, f"{workspace_type}.rosinstall", force)
        if rosinstall_path is not None:
            print(TerminalFormatter.color_text(f"Update {workspace_type}.rosinstall", bold=True))
        else:
            print(TerminalFormatter.color_text(f"Failed to download {workspace_type}.rosinstall", color='red'))
        if os.path.exists(rosinstall_path):
            print(TerminalFormatter.color_text(f"Found rosinstall file: {rosinstall_path}", bold=True))
            if not ros.run_vcs_import(nanosaur_home_path, rosinstall_path, src_folder="shared_src"):
                return False
    # Update rosinstall file and run vcs import

    def update_workspace(params, workspace_type, workspace_name_key, force, skip_rosinstall_update=False):
        workspace_path = get_workspace_path(params, workspace_name_key)
        if not workspace_path:
            return False
        rosinstall_path = os.path.join(workspace_path, f"{workspace_type}.rosinstall")
        if not skip_rosinstall_update:
            # Download rosinstall for this device
            url = f"{nanosaur_raw_url}/nanosaur/rosinstall/{workspace_type}.rosinstall"
            rosinstall_path = ros.download_rosinstall(url, workspace_path, f"{workspace_type}.rosinstall", force)
            if rosinstall_path is not None:
                print(TerminalFormatter.color_text(f"Update {workspace_type}.rosinstall", bold=True))
            else:
                print(TerminalFormatter.color_text(f"Failed to download {workspace_type}.rosinstall", color='red'))
        # run vcs import to sync the workspace
        if os.path.exists(rosinstall_path):
            print(TerminalFormatter.color_text(f"Found rosinstall file: {rosinstall_path}", bold=True))
            if not ros.run_vcs_import(workspace_path, rosinstall_path):
                return False
        return True

    workspace_actions = {
        'developer': lambda: update_workspace(params, 'developer', 'ws_developer_name', args.force, skip_rosinstall_update=True),
        'robot': lambda: update_workspace(params, 'robot', 'ws_robot_name', args.force),
        'simulation': lambda: update_workspace(params, 'simulation', 'ws_simulation_name', args.force),
        'perception': lambda: update_workspace(params, 'perception', 'ws_perception_name', args.force),
    }
    if args.all:
        print(TerminalFormatter.color_text("Updating all workspaces", bold=True))
        update_shared_workspace(args.force)
        results = [action() for action in workspace_actions.values()]
        return all(results)
    # Get the workspace
    workspace = get_selected_workspace(params, workspace_actions, args)
    if workspace is None:
        return False
    # Update the workspace
    print(TerminalFormatter.color_text(f"Updating {workspace}", bold=True))
    if action := workspace_actions.get(workspace):
        update_shared_workspace(args.force)
        return action()
    print(TerminalFormatter.color_text(f"Workspace {workspace} not found", color='red'))
    return False


@require_sudo_password
def build(platform, params: Params, args, password=None):
    """ Build the workspace """
    # Get the build action
    def get_build_action(workspace_name_key):
        workspace_path = get_workspace_path(params, workspace_name_key)
        if not workspace_path:
            return False
        print(TerminalFormatter.color_text(f"- Install all dependencies on workspace {workspace_path}", bold=True))
        if not ros.run_rosdep(workspace_path, password):
            print(TerminalFormatter.color_text("Failed to install dependencies", color='red'))
            return False
        print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
        if not ros.run_colcon_build(workspace_path):
            print(TerminalFormatter.color_text(f"Failed to build workspace {workspace_path}", color='red'))
            return False
        return True
    # Build the workspace
    workspace_actions = {
        'robot': lambda: get_build_action('ws_robot_name'),
        'simulation': lambda: get_build_action('ws_simulation_name'),
    }
    if args.all:
        print(TerminalFormatter.color_text("Building all workspaces", bold=True))
        results = [action() for action in workspace_actions.values()]
        return all(results)
    # Get the workspace
    workspace = get_selected_workspace(params, workspace_actions, args)
    if workspace is None:
        return False
    # Build the workspace
    print(TerminalFormatter.color_text(f"Building {workspace}", bold=True))
    if action := workspace_actions.get(workspace):
        return action()
    print(TerminalFormatter.color_text(f"Workspace {workspace} not found", color='red'))
    return False


def debug(platform, params: Params, args):
    """ Debug the workspace """
    workspace_actions = {
        'developer': lambda: ros.run_dev_script(params, get_workspace_path(params, 'ws_developer_name'), params.get('ws_developer_name', DEFAULT_WORKSPACE_DEVELOPER)),
        'simulation': lambda: start_robot_simulation(params),
        'perception': lambda: ros.run_dev_script(params, get_workspace_path(params, 'ws_perception_name'), params.get('ws_perception_name', DEFAULT_WORKSPACE_PERCEPTION))
    }
    workspace = get_selected_workspace(params, workspace_actions, args)
    if workspace is None:
        return False
    # Debug the workspace
    print(TerminalFormatter.color_text(f"Debugging {workspace}", bold=True))
    if action := workspace_actions.get(workspace):
        return action()
    print(TerminalFormatter.color_text(f"I cannot debug this {workspace}", color='red'))
    return False


def deploy(platform, params: Params, args):
    """ Deploy the workspace """
    workspace_actions = {
        'perception': lambda: ros.deploy_docker_perception(params, get_workspace_path(params, 'ws_perception_name'))
    }
    if args.all:
        print(TerminalFormatter.color_text("Deploying all workspaces", bold=True))
        results = [action() for action in workspace_actions.values()]
        return all(results)
    # Get the workspace
    workspace = get_selected_workspace(params, workspace_actions, args)
    if workspace is None:
        return False
    # Deploy the workspace
    print(TerminalFormatter.color_text(f"Deploying {workspace}", bold=True))
    if action := workspace_actions.get(workspace):
        return action()
    print(TerminalFormatter.color_text(f"I cannot deploy this {workspace}", color='red'))
    return False


def get_workspaces_path(params: Params) -> dict:
    nanosaur_home_path = get_nanosaur_home()
    # Add all workspaces that exist in the Nanosaur home folder
    workspaces = {
        'ws_developer_name': params.get('ws_developer_name', DEFAULT_WORKSPACE_DEVELOPER),
        'ws_robot_name': params.get('ws_robot_name', DEFAULT_WORKSPACE_ROBOT),
        'ws_simulation_name': params.get('ws_simulation_name', DEFAULT_WORKSPACE_SIMULATION),
        'ws_perception_name': params.get('ws_perception_name', DEFAULT_WORKSPACE_PERCEPTION)
    }
    return {
        name.split('_')[1]: os.path.join(nanosaur_home_path, path)
        for name, path in workspaces.items()
        if os.path.exists(os.path.join(nanosaur_home_path, path))
    }


def get_workspace_path(params: Params, ws_name) -> str:
    workspaces = {
        'ws_developer_name': params.get('ws_developer_name', DEFAULT_WORKSPACE_DEVELOPER),
        'ws_robot_name': params.get('ws_robot_name', DEFAULT_WORKSPACE_ROBOT),
        'ws_simulation_name': params.get('ws_simulation_name', DEFAULT_WORKSPACE_SIMULATION),
        'ws_perception_name': params.get('ws_perception_name', DEFAULT_WORKSPACE_PERCEPTION)
    }
    if ws_name not in workspaces:
        return None
    ws_name_folder = workspaces[ws_name]
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home()
    # Create the full path for the workspace folder in the user's home directory
    workspace_path = os.path.join(nanosaur_home_path, ws_name_folder)

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


def clean_workspace(nanosaur_ws_name) -> bool:
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    nanosaur_home_path = get_nanosaur_home()
    # Create the full path for the workspace folder in the user's home directory
    workspace_path = os.path.join(nanosaur_home_path, nanosaur_ws_name)

    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        subfolders = ['build', 'install', 'log']
        subfolders_exist = all(os.path.exists(os.path.join(workspace_path, subfolder)) for subfolder in subfolders)

        if subfolders_exist:
            print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' and subfolders exist. Cleaning build, install and log folders", color='yellow'))
            try:
                os.system(f"rm -Rf {workspace_path}/build {workspace_path}/install {workspace_path}/log")
                print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' cleaned up.", color='green'))
                return True
            except Exception as e:
                print(TerminalFormatter.color_text(f"Error running rm {str(e)}", color='red'))
                return False
        else:
            print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' does not contain build, install and log folders.", color='yellow'))
            return False
    return False


def create_developer_workspace(platform, params: Params, args, password=None) -> bool:
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home()
    # Create developer workspace
    create_workspace(nanosaur_home_path, params.get('ws_developer_name', DEFAULT_WORKSPACE_DEVELOPER), skip_create_colcon_setting=True)
    return True


def create_maintainer_workspace(platform, params: Params, args, password=None):
    # determine the device type
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home()
    # Create the shared source folder
    nanosaur_shared_src = os.path.join(nanosaur_home_path, "shared_src")
    # Check if folder exists, if not, create it
    if not os.path.exists(nanosaur_shared_src):
        os.makedirs(nanosaur_shared_src)
    if device_type == "robot" or args.all:
        # Make the robot workspace
        create_workspace(nanosaur_home_path, params.get('ws_robot_name', DEFAULT_WORKSPACE_ROBOT))
    # Make the simulation workspace
    if device_type == "desktop" or args.all:
        # Make the simulation workspace
        create_workspace(nanosaur_home_path, params.get('ws_simulation_name', DEFAULT_WORKSPACE_SIMULATION))
    # Make the perception workspace
    create_workspace(nanosaur_home_path, params.get('ws_perception_name', DEFAULT_WORKSPACE_PERCEPTION))
    # set all workspaces to be updated
    args.all = True
    if args.force:
        clean(platform, params, args)
    # Update all workspaces
    update(platform, params, args)
    # Build all workspaces
    build(platform, params, args)
    return True
# EOF
