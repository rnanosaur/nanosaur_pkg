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
import logging
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur import ros
from nanosaur.docker import docker_service_run_command
from nanosaur.simulation import simulation_robot_start_debug, simulation_start_debug
from nanosaur import utilities
import inquirer

# Set up the logger
logger = logging.getLogger(__name__)

# Default colcon settings
COLCON_DEFAULTS = {
    'build': {
        'base-paths': ['src', '../shared_src'],
        'symlink-install': True,
        'merge-install': True,
    }
}

ROS_DISTRO = "humble"
ISAAC_ROS_RELEASE = "release-3.2"
ISAAC_ROS_DISTRO_SUFFIX = "ros2_humble"
ISAAC_ROS_DOCKER_CAMERA_LIST = ["realsense", "zed"]
NANOSAUR_DOCKERFILE_SUFFIX = "nanosaur"

DEFAULT_WORKSPACE_PERCEPTION = 'perception_ws'
DEFAULT_WORKSPACE_SIMULATION = 'simulation_ws'
DEFAULT_WORKSPACE_ROBOT = 'robot_ws'
DEFAULT_WORKSPACE_DEVELOPER = 'ros_ws'

NANOSAUR_DOCKER_PACKAGE_ROBOT = "nanosaur"
NANOSAUR_DOCKER_PACKAGE_SIMULATION = "simulation"
NANOSAUR_DOCKER_PACKAGE_PERCEPTION = "perception"


def get_starting_location(params: utilities.Params) -> str:
    """Prompt the user to select the location to run the command."""
    debug_mode = None
    if 'ws_debug' in params:
        debug_mode = params['ws_debug']
        print(TerminalFormatter.color_text(f"Default debug mode: {debug_mode}", color='yellow'))
    # Get the ROS 2 installation path if available
    ros2_installed = ros.get_ros2_path(ROS_DISTRO)
    debug_mode = 'docker' if ros2_installed is None else debug_mode
    # Ask the user to select the location
    questions = [
        inquirer.List(
            'location',
            message="Run locally or on docker?",
            choices=['host', 'docker'],
            ignore=lambda answers: debug_mode,
        ),
    ]
    answers = inquirer.prompt(questions)
    if answers is None:
        return None
    return answers['location'] if debug_mode is None else debug_mode


def workspaces_info(params: utilities.Params, verbose: bool):
    """Print information about the workspaces."""

    def ros_info(params):
        # Print ROS 2 installation path
        ros2_path = ros.get_ros2_path(ROS_DISTRO)
        ros2_version_color = TerminalFormatter.color_text(ROS_DISTRO.capitalize(), color='blue', bold=True)
        ros2_string = TerminalFormatter.color_text(f"ROS 2 {ros2_version_color}:", bold=True)
        print(f"{ros2_string} {TerminalFormatter.clickable_link(ros2_path)}")
        # Print Isaac ROS installation path
        isaac_ros_version = params.get('isaac_ros_branch', ISAAC_ROS_RELEASE)
        isaac_ros_string = TerminalFormatter.color_text("Isaac ROS:", bold=True)
        print(f"{isaac_ros_string} {isaac_ros_version}")

    # Print installed workspaces
    workspaces = get_workspaces_path(params)
    print()
    if workspaces:
        print(TerminalFormatter.color_text("Installed Workspaces:", bold=True))
        for ws_name, ws_path in workspaces.items():
            # Get the workspace path if it exists
            print(f"  {TerminalFormatter.color_text(ws_name, bold=True)}: {TerminalFormatter.clickable_link(ws_path)}")
    elif verbose:
        print(TerminalFormatter.color_text("No workspaces installed", bold=True))
    # Print ROS 2 and Isaac ROS information
    if verbose:
        ros_info(params)


def parser_workspace_menu(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    # Check if ROS 2 is installed
    ros2_installed = ros.get_ros2_path(ROS_DISTRO)
    # Add workspace subcommand
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
    if ros2_installed is not None:
        add_workspace_subcommand('build', "Build the workspace", build)
    parser_debug = add_workspace_subcommand('debug', "Debug the workspace", debug)
    parser_deploy = add_workspace_subcommand('deploy', "Deploy workspace to docker image", deploy)
    parser_deploy.add_argument('image_name', type=str, nargs='?', help="Specify the image name")
    parser_debug.add_argument('image_name', type=str, nargs='?', help="Specify the image name")
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
    if len(workspaces) > 1:
        questions = [
            inquirer.List('workspace', message="Select a workspace", choices=workspaces),
        ]
        answers = inquirer.prompt(questions)
        return answers['workspace'] if answers else None
    return list(workspaces.keys())[0]


def clean(platform, params: utilities.Params, args):
    """ Clean the workspace """
    workspace_actions = {
        'developer': lambda: clean_workspace(params.get('ws_developer_name', DEFAULT_WORKSPACE_DEVELOPER)),
        'robot': lambda: clean_workspace(params.get('ws_robot_name', DEFAULT_WORKSPACE_ROBOT)),
        'simulation': lambda: clean_workspace(params.get('ws_simulation_name', DEFAULT_WORKSPACE_SIMULATION)),
        'perception': lambda: clean_workspace(params.get('ws_perception_name', DEFAULT_WORKSPACE_PERCEPTION))
    }
    if args.all:
        workspaces = get_workspaces_path(params)
        workspace_actions = {k: v for k, v in workspace_actions.items() if k in workspaces}
        print(TerminalFormatter.color_text("Cleaning all workspaces", bold=True))
        return all(action() for action in workspace_actions.values())
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


def update(platform, params: utilities.Params, args):
    """ Update the workspace """
    # Get the Nanosaur home folder
    nanosaur_home_path = utilities.get_nanosaur_home()
    # Get the Nanosaur home folder and branch
    nanosaur_raw_url = utilities.get_nanosaur_raw_github_url(params)
    # Update shared workspace

    def update_shared_workspace(force):
        shared_src_path = os.path.join(nanosaur_home_path, "shared_src")
        rosinstall_path = os.path.join(shared_src_path, "shared.rosinstall")
        workspace_type = 'shared'
        # Download rosinstall for this device
        url = f"{nanosaur_raw_url}/nanosaur/rosinstall/{workspace_type}.rosinstall"
        rosinstall_path = utilities.download_file(url, shared_src_path, f"{workspace_type}.rosinstall", force)
        if rosinstall_path is not None:
            print(TerminalFormatter.color_text(f"Update {workspace_type}.rosinstall", bold=True))
        else:
            print(TerminalFormatter.color_text(f"Failed to download {workspace_type}.rosinstall", color='red'))
        if os.path.exists(rosinstall_path):
            print(TerminalFormatter.color_text(f"Found rosinstall file: {rosinstall_path}", bold=True))
            if not ros.run_vcs_import(nanosaur_home_path, rosinstall_path, src_folder="shared_src"):
                return False
        return True
    # Update rosinstall file and run vcs import

    def update_workspace(params, workspace_type, workspace_name_key, force, skip_rosinstall_update=False):
        workspace_path = get_workspace_path(params, workspace_name_key)
        if not workspace_path:
            print(TerminalFormatter.color_text(f"Workspace {workspace_type} not found", color='red'))
            return False
        rosinstall_path = os.path.join(workspace_path, f"{workspace_type}.rosinstall")
        if not skip_rosinstall_update:
            # Download rosinstall for this device
            url = f"{nanosaur_raw_url}/nanosaur/rosinstall/{workspace_type}.rosinstall"
            rosinstall_path = utilities.download_file(url, workspace_path, f"{workspace_type}.rosinstall", force)
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
        print(TerminalFormatter.color_text("Updating isaac_ros_common repository", bold=True))
        isaac_ros_branch = params['isaac_ros_branch']
        ros.manage_isaac_ros_common_repo(nanosaur_home_path, isaac_ros_branch, args.force)
        print(TerminalFormatter.color_text("Updating all workspaces", bold=True))
        update_shared_workspace(args.force)
        workspaces = get_workspaces_path(params)
        workspace_actions = {k: v for k, v in workspace_actions.items() if k in workspaces}
        return all(action() for action in workspace_actions.values())
    # Get the workspace
    workspace = get_selected_workspace(params, workspace_actions, args)
    if workspace is None:
        return False
    # Update the workspace
    print(TerminalFormatter.color_text("Updating isaac_ros_common repository", bold=True))
    isaac_ros_branch = params['isaac_ros_branch']
    ros.manage_isaac_ros_common_repo(nanosaur_home_path, isaac_ros_branch, args.force)
    print(TerminalFormatter.color_text(f"Updating {workspace}", bold=True))
    if action := workspace_actions.get(workspace):
        update_shared_workspace(args.force)
        return action()
    print(TerminalFormatter.color_text(f"Workspace {workspace} not found", color='red'))
    return False


@utilities.require_sudo_password
def build(platform, params: utilities.Params, args, password=None):
    """ Build the workspace """
    # Get the build action
    def get_build_action(workspace_name_key):
        workspace_path = get_workspace_path(params, workspace_name_key)
        if not workspace_path:
            return False
        print(TerminalFormatter.color_text(f"- Install all dependencies on workspace {workspace_path}", bold=True))
        ros2_path = ros.get_ros2_path(ROS_DISTRO)
        if not ros.run_rosdep(ros2_path, workspace_path, password):
            print(TerminalFormatter.color_text("Failed to install dependencies", color='red'))
            return False
        print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
        if not ros.run_colcon_build(ros2_path, workspace_path):
            print(TerminalFormatter.color_text(f"Failed to build workspace {workspace_path}", color='red'))
            return False
        return True
    # Build the workspace
    workspace_actions = {
        'simulation': lambda: get_build_action('ws_simulation_name'),
    }
    if args.all:
        workspaces = get_workspaces_path(params)
        workspace_actions = {k: v for k, v in workspace_actions.items() if k in workspaces}
        print(TerminalFormatter.color_text("Building all workspaces", bold=True))
        return all(action() for action in workspace_actions.values())
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


def debug(platform, params: utilities.Params, args):
    """ Debug the workspace """
    # Get the debug mode
    debug_mode = None
    if 'ws_debug' in params:
        debug_mode = params['ws_debug']
        print(TerminalFormatter.color_text(f"Default debug mode: {debug_mode}", color='yellow'))
    # Get the ROS 2 installation path if available
    ros2_installed = ros.get_ros2_path(ROS_DISTRO)
    debug_mode = 'docker' if ros2_installed is None else debug_mode

    def debug_simulation(params: utilities.Params, args):
        # Get the selected launcher
        questions = [
            inquirer.List(
                'launcher',
                message="Select a launcher to debug",
                choices=['gazebo', 'isaac-sim', 'robot'],
                ignore=lambda answers: args.image_name is not None,
            ),
            inquirer.List(
                'location',
                message="Select where you want to debug",
                choices=['host', 'docker'],
                ignore=lambda answers: debug_mode,
            ),
        ]
        # Ask the user to select a launcher
        answers = inquirer.prompt(questions)
        if answers is None:
            return False
        selected_launcher = answers['launcher'] if args.image_name is None else args.image_name
        selected_location = answers['location'] if debug_mode is None else debug_mode
        # Create the Nanosaur home folder
        nanosaur_home_path = utilities.get_nanosaur_home()
        nanosaur_shared_src = os.path.join(nanosaur_home_path, "shared_src")
        simulation_ws_path = get_workspace_path(params, 'ws_simulation_name')

        print(TerminalFormatter.color_text(f"Debugging {selected_launcher} in {selected_location}", bold=True))
        # Debug the simulation workspace
        if selected_location == 'host':
            # Debug locally
            if selected_launcher == 'robot':
                return simulation_robot_start_debug(params)
            isaac_sim_path = params.get('isaac_sim_path', None)
            return simulation_start_debug(simulation_ws_path, selected_launcher, isaac_sim_path=isaac_sim_path)
        elif selected_location == 'docker':
            # Set the volumes
            volumes = [
                (simulation_ws_path, '/ros_ws'),
                (nanosaur_shared_src, '/shared_src'),
            ]
            # Get the robot object
            robot = utilities.RobotList.current_robot(params)
            core_tag = "simulation" if robot.simulation else "robot"
            container_name = f"{robot.name}-{core_tag}-debug" if selected_launcher == 'robot' else f"{selected_launcher}-debug"
            # Debug in Docker container
            return docker_service_run_command(
                platform, params, selected_launcher, command=['bash'],
                name=container_name, volumes=volumes
            )

        return False

    workspace_actions = {
        'developer': lambda: ros.run_docker_isaac_ros(get_workspace_path(params, 'ws_developer_name')),
        'simulation': lambda: debug_simulation(params, args),
        'perception': lambda: ros.run_docker_isaac_ros(get_workspace_path(params, 'ws_perception_name'))
    }
    workspace = get_selected_workspace(params, workspace_actions, args)
    if workspace is None:
        return False
    # Debug the workspace
    # print(TerminalFormatter.color_text(f"Debugging {workspace}", bold=True))
    if action := workspace_actions.get(workspace):
        return action()
    print(TerminalFormatter.color_text(f"I cannot debug this {workspace}", color='red'))
    return False


def deploy(platform, params: utilities.Params, args):
    """ Deploy the workspace """
    # Get the Nanosaur docker user
    nanosaur_docker_user = utilities.get_nanosaur_docker_user(params)
    # Get the Isaac ROS distro name
    isaac_ros_distro_name = params.get('isaac_ros_distro', ISAAC_ROS_DISTRO_SUFFIX)

    def deploy_simulation(image_name):
        """ Deploy the simulation workspace """
        simulation_ws_path = get_workspace_path(params, 'ws_simulation_name')
        # Get the path to the nanosaur_simulations package
        nanosaur_simulations_path = os.path.join(simulation_ws_path, 'src', 'nanosaur_simulations')

        # Build Gazebo sim docker
        if image_name == 'gazebo' or image_name is None:
            tag_image = f"{nanosaur_docker_user}/{NANOSAUR_DOCKER_PACKAGE_SIMULATION}:gazebo"
            dockerfile_path = f"{nanosaur_simulations_path}/Dockerfile.gazebo"
            if not ros.deploy_docker_image(dockerfile_path, tag_image):
                return False
        # Build Isaac Sim docker
        if image_name == 'isaac-sim' or image_name is None:
            tag_image = f"{nanosaur_docker_user}/{NANOSAUR_DOCKER_PACKAGE_SIMULATION}:isaac-sim"
            dockerfile_path = f"{nanosaur_simulations_path}/Dockerfile.isaac-sim"
            if not ros.deploy_docker_image(dockerfile_path, tag_image):
                return False
        # Build the Docker image for nanosaur bridge
        if image_name == 'robot' or image_name is None:
            tag_image = f"{nanosaur_docker_user}/{NANOSAUR_DOCKER_PACKAGE_ROBOT}:simulation"
            dockerfile_path = f"{nanosaur_simulations_path}/Dockerfile.nanosaur"
            if not ros.deploy_docker_image(dockerfile_path, tag_image):
                return False
        return True

    def deploy_perception(image_name):
        """ Deploy the perception workspace """
        # determine the device type
        device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
        # Get the path to the perception workspace
        perception_ws_path = get_workspace_path(params, 'ws_perception_name')
        # Get the release tag name
        release_tag_name = f"{nanosaur_docker_user}/{NANOSAUR_DOCKER_PACKAGE_PERCEPTION}"
        # Deploy the perception workspace
        if device_type == "robot":
            # Deploy the perception workspace for each camera
            cameras = ISAAC_ROS_DOCKER_CAMERA_LIST if args.all or image_name is None else [image_name]
            for camera in cameras:
                # Create the tags for the docker image
                tags = [isaac_ros_distro_name, camera, NANOSAUR_DOCKERFILE_SUFFIX]
                # Deploy the perception workspace for each camera
                if not ros.deploy_docker_isaac_ros(perception_ws_path, tags, f"{release_tag_name}:{camera}"):
                    return False
        if device_type == "desktop":
            # Deploy the perception workspace for simulation
            tags = [isaac_ros_distro_name, NANOSAUR_DOCKERFILE_SUFFIX]
            if not ros.deploy_docker_isaac_ros(perception_ws_path, tags, f"{release_tag_name}:simulation"):
                return False
        return True

    # Get the deploy action
    workspace_actions = {
        'developer': lambda: ros.deploy_docker_isaac_ros(
            get_workspace_path(params, 'ws_developer_name'),
            [isaac_ros_distro_name, NANOSAUR_DOCKERFILE_SUFFIX],
            f'{nanosaur_docker_user}/developer'
        ),
        'simulation': lambda: deploy_simulation(args.image_name),
        'perception': lambda: deploy_perception(args.image_name),
    }
    if args.all:
        workspaces = get_workspaces_path(params)
        workspace_actions = {k: v for k, v in workspace_actions.items() if k in workspaces}
        print(TerminalFormatter.color_text("Deploying all workspaces", bold=True))
        return all(action() for action in workspace_actions.values())
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


def get_workspaces_path(params: utilities.Params) -> dict:
    nanosaur_home_path = utilities.get_nanosaur_home()
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


def get_workspace_path(params: utilities.Params, ws_name) -> str:
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
    nanosaur_home_path = utilities.create_nanosaur_home()
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


def create_shared_workspace() -> str:
    # Create the Nanosaur home folder
    nanosaur_home_path = utilities.create_nanosaur_home()
    # Create the shared source folder
    nanosaur_shared_src = os.path.join(nanosaur_home_path, "shared_src")
    # Check if folder exists, if not, create it
    if not os.path.exists(nanosaur_shared_src):
        os.makedirs(nanosaur_shared_src)
    return nanosaur_shared_src


def clean_workspace(nanosaur_ws_name) -> bool:
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    nanosaur_home_path = utilities.get_nanosaur_home()
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
            except Exception as e:
                print(TerminalFormatter.color_text(f"Error running rm {str(e)}", color='red'))
                return False
        else:
            print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' does not contain build, install and log folders.", color='yellow'))
    return True


def create_simple(platform, params: utilities.Params, args) -> bool:
    # Create the Nanosaur home folder
    nanosaur_home_path = utilities.create_nanosaur_home()
    # Store nanosaur distro and Isaac ROS distro
    params['nanosaur_branch'] = params.get('nanosaur_branch', utilities.NANOSAUR_MAIN_BRANCH)
    # Determine the device type
    workspace_type = "robot" if platform['Machine'] == 'jetson' else "simulation"
    docker_compose = f"docker-compose.{workspace_type}.yml"
    # Get the Nanosaur home folder and branch
    nanosaur_raw_url = utilities.get_nanosaur_raw_github_url(params)
    url = f"{nanosaur_raw_url}/nanosaur/compose/{docker_compose}"
    # Download the docker-compose file
    utilities.download_file(url, nanosaur_home_path, docker_compose, force=args.force)
    return True


def create_developer_workspace(platform, params: utilities.Params, args, password=None) -> bool:
    # Create the Nanosaur home folder
    create_simple(platform, params, args)
    # Create the Nanosaur home folder
    nanosaur_home_path = utilities.create_nanosaur_home()
    # Store nanosaur distro and Isaac ROS distro
    params['isaac_ros_branch'] = params.get('isaac_ros_branch', ISAAC_ROS_RELEASE)
    # Create the shared source folder
    create_shared_workspace()
    # Create developer workspace
    create_workspace(nanosaur_home_path, params.get('ws_developer_name', DEFAULT_WORKSPACE_DEVELOPER))
    # set all workspaces to be updated
    args.all = True
    if args.force:
        clean(platform, params, args)
    # Update all workspaces
    update(platform, params, args)
    return True


def create_maintainer_workspace(platform, params: utilities.Params, args, password=None) -> bool:
    # Check if ROS 2 is installed
    ros2_installed = ros.get_ros2_path(ROS_DISTRO)
    # determine the device type
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    # Create the Nanosaur home folder
    nanosaur_home_path = utilities.create_nanosaur_home()
    # Store nanosaur distro and Isaac ROS distro
    params['nanosaur_branch'] = params.get('nanosaur_branch', utilities.NANOSAUR_MAIN_BRANCH)
    params['isaac_ros_branch'] = params.get('isaac_ros_branch', ISAAC_ROS_RELEASE)
    # Create the shared source folder
    create_shared_workspace()
    if device_type == "robot" or args.all:
        # Make the robot workspace
        create_workspace(nanosaur_home_path, params.get('ws_robot_name', DEFAULT_WORKSPACE_ROBOT))
    # Make the simulation workspace
    if device_type == "desktop" or args.all:
        # Make the simulation workspace
        create_workspace(nanosaur_home_path, params.get('ws_simulation_name', DEFAULT_WORKSPACE_SIMULATION))
    # Make the perception workspace
    create_workspace(nanosaur_home_path, params.get('ws_perception_name', DEFAULT_WORKSPACE_PERCEPTION))

    # Check if docker-compose files exist
    def handle_docker_compose_file(docker_compose_file):
        # Check if the docker-compose file exists
        docker_compose_path = os.path.join(nanosaur_home_path, docker_compose_file)
        if os.path.exists(docker_compose_path) and not os.path.islink(docker_compose_path):
            old_path = f"{docker_compose_path}.old"
            os.rename(docker_compose_path, old_path)
            print(TerminalFormatter.color_text(f"Renamed existing {docker_compose_file} to {old_path}", color='yellow'))
        # Create a symlink to the new docker-compose file
        new_path = os.path.join(nanosaur_home_path, 'shared_src', 'nanosaur', 'nanosaur', 'compose', docker_compose_file)
        if os.path.exists(docker_compose_path):
            os.remove(docker_compose_path)
        os.symlink(new_path, docker_compose_path)
        print(TerminalFormatter.color_text(f"Created symlink for {docker_compose_file} to {new_path}", color='green'))

    # Check if docker-compose files exist
    if device_type == "robot" or args.all:
        handle_docker_compose_file('docker-compose.robot.yml')
    if device_type == "desktop" or args.all:
        handle_docker_compose_file('docker-compose.simulation.yml')

    # set all workspaces to be updated
    args.all = True
    if args.force:
        clean(platform, params, args)
    # Update all workspaces
    update(platform, params, args)
    # Build all workspaces
    if ros2_installed is not None:
        build(platform, params, args)
    return True
# EOF
