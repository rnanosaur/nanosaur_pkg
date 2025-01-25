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

import inquirer
from inquirer.themes import GreenPassion
import argparse
import subprocess
import shlex
from nanosaur import workspace
from nanosaur import docker
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList
from nanosaur.utilities import ENGINES_CHOICES, CAMERA_CHOICES, LIDAR_CHOICES


def add_robot_config_subcommands(subparsers: argparse._SubParsersAction, params: Params) -> argparse.ArgumentParser:
    robot_data = RobotList.get_robot(params)
    parser_robot_config = subparsers.add_parser('config', help=f"Configure the robot settings [{robot_data.name}]")
    config_subparsers = parser_robot_config.add_subparsers(dest='config_type', help="Configuration options")
    # Add robot name subcommand
    parser_robot_name = config_subparsers.add_parser('name', help=f"Change the robot name [{robot_data.name}]")
    parser_robot_name.set_defaults(func=robot_set_name)
    # Add robot domain id subcommand
    parser_robot_domain_id = config_subparsers.add_parser('domain_id', help=f"Change the robot domain ID [{robot_data.domain_id}]")
    parser_robot_domain_id.set_defaults(func=robot_set_domain_id)
    # Add robot camera subcommand
    parser_robot_camera = config_subparsers.add_parser('camera', help=f"Change the robot camera type [{robot_data.camera_type}]")
    parser_robot_camera.add_argument('--new', type=str, help=f"Specify the new camera type (options: {', '.join(CAMERA_CHOICES)})")
    parser_robot_camera.set_defaults(func=robot_set_camera)
    # Add robot lidar subcommand
    parser_robot_lidar = config_subparsers.add_parser('lidar', help=f"Change the robot lidar type [{robot_data.lidar_type}]")
    parser_robot_lidar.add_argument('--new', type=str, choices=LIDAR_CHOICES, help=f"Specify the new lidar type (options: {', '.join(LIDAR_CHOICES)})")
    parser_robot_lidar.set_defaults(func=robot_set_lidar)
    # Add robot engines subcommand
    parser_robot_engines = config_subparsers.add_parser('engines', help=f"Configure the robot engines [{', '.join(robot_data.engines)}]")
    parser_robot_engines.add_argument('--new', type=str, help="Specify the new engine configuration")
    parser_robot_engines.set_defaults(func=robot_configure_engines)
    # Add robot reset subcommand
    parser_robot_reset = config_subparsers.add_parser('reset', help="Restore the robot configuration to default")
    parser_robot_reset.set_defaults(func=robot_reset)

    return parser_robot_config


def parser_robot_menu(subparsers: argparse._SubParsersAction, params: Params) -> argparse.ArgumentParser:
    robot_data = RobotList.get_robot(params)
    parser_robot = subparsers.add_parser('robot', help=f"Manage the Nanosaur robot [{robot_data.name}]")
    robot_subparsers = parser_robot.add_subparsers(dest='robot_type', help="Robot operations")
    # Add robot display subcommand
    parser_robot_display = robot_subparsers.add_parser('display', help="Show the robot")
    parser_robot_display.set_defaults(func=robot_display)
    # Add robot drive subcommand
    parser_robot_drive = robot_subparsers.add_parser('drive', help="Control the robot")
    parser_robot_drive.set_defaults(func=control_keyboard)
    # Add robot start subcommand
    parser_robot_start = robot_subparsers.add_parser('start', help="Activate the robot")
    parser_robot_start.add_argument(
        '--profile', type=str, help="Specify the profile name to use")
    parser_robot_start.add_argument(
        '--detach', action='store_true', help="Run the robot in detached mode")
    parser_robot_start.set_defaults(func=docker.docker_robot_start)
    # Add robot stop subcommand
    parser_robot_stop = robot_subparsers.add_parser('stop', help="Deactivate the robot")
    parser_robot_stop.set_defaults(func=docker.docker_robot_stop)

    # Add robot config subcommand
    parser_config = add_robot_config_subcommands(robot_subparsers, params)

    return parser_robot, parser_config


def robot_set_name(platform, params: Params, args):
    """Configure the robot name."""
    robot = RobotList.get_robot(params)

    def validate_name(_, x):
        if not x.isalnum():
            raise inquirer.errors.ValidationError("", reason="Name must contain only letters and numbers")
        return True

    question = [
        inquirer.Text(
            'name',
            message="Enter the new robot name",
            default=robot.name,
            validate=validate_name
        )
    ]

    answers = inquirer.prompt(question, theme=GreenPassion())
    if answers is None:
        return False
    if not answers['name']:
        print(TerminalFormatter.color_text("No name provided", color='red'))
        return False
    new_name = answers['name']
    if new_name != robot.name:
        robot.name = new_name
        RobotList.update_robot(params, robot)
        print(TerminalFormatter.color_text(f"Robot name set to: {robot.name}", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Robot name {new_name} is already set", color='yellow'))


def robot_set_domain_id(platform, params: Params, args):
    """Configure the domain ID."""
    robot = RobotList.get_robot(params)

    def validate_domain_id(_, x):
        if not x.isdigit():
            raise inquirer.errors.ValidationError("", reason="Domain ID must be a number")
        return True

    question = [
        inquirer.Text(
            'domain_id',
            message="Enter the new domain ID",
            default=str(robot.domain_id),
            validate=validate_domain_id
        )
    ]

    answers = inquirer.prompt(question, theme=GreenPassion())
    if answers is None:
        return False
    new_domain_id = int(answers['domain_id'])
    if new_domain_id != robot.domain_id:
        robot.domain_id = new_domain_id
        RobotList.update_robot(params, robot)
        print(TerminalFormatter.color_text(f"Domain ID set to: {robot.domain_id}", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Domain ID {new_domain_id} is already set", color='yellow'))


def robot_set_camera(platform, params: Params, args):
    """Configure the camera."""
    robot = RobotList.get_robot(params)

    all_cameras = sorted(set(CAMERA_CHOICES + [robot.camera_type]))

    if args.new is not None:
        if args.new not in all_cameras:
            robot.camera_type = args.new
            RobotList.update_robot(params, robot)
            print(TerminalFormatter.color_text(f"New camera {args.new} selected", color='green'))
        else:
            print(TerminalFormatter.color_text(f"Camera {args.new} is already exist", color='yellow'))
        return True

    options = [
        inquirer.List(
            'camera',
            message="Select a camera",
            choices=[camera or 'No camera' for camera in all_cameras],
            default=robot.camera_type
        )
    ]

    answers = inquirer.prompt(options, theme=GreenPassion())
    if answers is None:
        return False
    selected_camera = answers['camera'].replace(" (selected)", "")

    selected_camera = '' if selected_camera == 'No camera' else selected_camera
    if selected_camera != robot.camera_type:
        robot.camera_type = selected_camera
        RobotList.update_robot(params, robot)
        print(TerminalFormatter.color_text(f"Camera set to: {robot.camera_type or 'No camera'}", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Camera {selected_camera or 'No camera'} is already selected", color='yellow'))


def robot_set_lidar(platform, params: Params, args):
    """Configure the lidar."""
    robot = RobotList.get_robot(params)

    all_lidars = sorted(set(LIDAR_CHOICES + [robot.lidar_type]))

    if args.new is not None:
        if args.new not in all_lidars:
            robot.lidar_type = args.new
            RobotList.update_robot(params, robot)
            print(TerminalFormatter.color_text(f"New lidar {args.new} selected", color='green'))
        else:
            print(TerminalFormatter.color_text(f"Lidar {args.new} is already exist", color='yellow'))
        return True

    options = [
        inquirer.List(
            'lidar',
            message="Select a lidar",
            choices=[lidar or 'No lidar' for lidar in all_lidars],
            default=robot.lidar_type
        )
    ]

    answers = inquirer.prompt(options, theme=GreenPassion())
    if answers is None:
        return False
    selected_lidar = answers['lidar'].replace(" (selected)", "")

    selected_lidar = '' if selected_lidar == 'No lidar' else selected_lidar
    if selected_lidar != robot.lidar_type:
        robot.lidar_type = selected_lidar
        RobotList.update_robot(params, robot)
        print(TerminalFormatter.color_text(f"Lidar set to: {robot.lidar_type or 'No lidar'}", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Lidar {selected_lidar or 'No lidar'} is already selected", color='yellow'))


def robot_configure_engines(platform, params: Params, args):
    """Configure the robot engines."""
    robot = RobotList.get_robot(params)

    if args.new is not None:
        if args.new not in robot.engines:
            robot.engines.append(args.new)
            RobotList.update_robot(params, robot)
            print(TerminalFormatter.color_text(f"New engine {args.new} added", color='green'))
        else:
            print(TerminalFormatter.color_text(f"Engine {args.new} is already enabled", color='yellow'))
        return True

    engine_choices = [
        inquirer.Checkbox(
            'engines',
            message="Select engines to enable/disable",
            choices=list(sorted(list(set(robot.engines + ENGINES_CHOICES)))),
            default=robot.engines,
        )
    ]

    answers = inquirer.prompt(engine_choices, theme=GreenPassion())
    if answers is None:
        return False
    robot.engines = answers['engines']
    RobotList.update_robot(params, robot)
    if robot.engines:
        print(TerminalFormatter.color_text(f"Engines updated: {', '.join(robot.engines)}", color='green'))
    else:
        print(TerminalFormatter.color_text("No engines selected", color='yellow'))


def robot_reset(platform, params: Params, args):
    """Reset the robot configuration."""
    # Reset the robot configuration
    RobotList.remove_robot(params)
    print(TerminalFormatter.color_text("Robot configuration reset", color='green'))
    return True


def control_keyboard(platform, params: Params, args):
    """Control the robot using the keyboard."""
    robot = RobotList.get_robot(params)
    command = f"ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/{robot.name}/key_vel"
    # Run from local machine
    if params.get('mode', '') in ['maintainer', 'Raffo']:
        nanosaur_ws_path = workspace.get_workspace_path(params, 'ws_simulation_name')
        bash_file = f'{nanosaur_ws_path}/install/setup.bash'
        # Read the robot name
        print(TerminalFormatter.color_text(f"Control the robot {robot.name} using the keyboard", color='green'))
        subprocess.run(f'source {bash_file} && {command}', shell=True, executable='/bin/bash')
        return True
    # Run from docker container
    docker.docker_robot_run_command(platform, params, shlex.split(command), name=f"{robot.name}-keyboard")
    return True


def robot_display(platform, params: Params, args):
    """Display the robot configuration."""
    robot = RobotList.get_robot(params)
    command = f"ros2 launch nanosaur_visualization robot_display.launch.py robot_name:={robot.name}"
    # Run from local machine
    if params.get('mode', '') in ['maintainer', 'Raffo']:
        nanosaur_ws_path = workspace.get_workspace_path(params, 'ws_simulation_name')
        bash_file = f'{nanosaur_ws_path}/install/setup.bash'
        print(TerminalFormatter.color_text(f"Display the robot {robot.name}", color='green'))
        subprocess.run(f'source {bash_file} && {command}', shell=True, executable='/bin/bash')
        return True
    # Run from docker container
    docker.docker_robot_run_command(platform, params, shlex.split(command), name=f"{robot.name}-rviz")
    return True
# EOF
