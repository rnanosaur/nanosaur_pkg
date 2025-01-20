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

import subprocess
from nanosaur import workspace
from nanosaur import docker
from nanosaur import simulation
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList
from nanosaur.utilities import ENGINES_CHOICES


def robot_start(platform, params: Params, args):
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    # Check the device type
    if device_type == "desktop":
        # Start the robot simulation
        if args.container:
            docker.docker_start(platform, params, args)
        else:
            simulation.start_robot_simulation(params)
    elif device_type == "robot":
        print(TerminalFormatter.color_text("Not yet implemented", color='yellow'))
    else:
        print(TerminalFormatter.color_text("Unknown device type", color='red'))
    return True


def robot_stop(platform, params: Params, args):
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    # Check the device type
    if device_type == "desktop":
        docker.docker_stop(platform, params, args)
    elif device_type == "robot":
        print(TerminalFormatter.color_text("Not yet implemented", color='yellow'))
    else:
        print(TerminalFormatter.color_text("Unknown device type", color='red'))
    return True


def robot_set_name(platform, params: Params, args):
    """Configure the robot name."""
    # Check if the robot name is provided
    robot = RobotList.get_robot(params)
    if not args.name:
        print(f"Current robot name: {robot.name}")
        return True
    # Update the robot name
    robot.name = args.name
    RobotList.update_robot(params, robot)
    print(TerminalFormatter.color_text(f"Robot name set to: {robot.name}", color='green'))
    return True


def robot_set_domain_id(platform, params: Params, args):
    """Configure the domain ID."""
    # Check if the domain ID is provided
    robot = RobotList.get_robot(params)
    if not args.domain_id:
        print(f"Current robot domain_id: {robot.domain_id}")
        return True

    # Update the domain ID
    robot.domain_id = args.domain_id
    RobotList.update_robot(params, robot)
    print(TerminalFormatter.color_text(f"Domain ID set to: {robot.domain_id}", color='green'))
    return True


def robot_set_camera(platform, params: Params, args):
    """Configure the camera."""
    # Check if the camera is provided
    robot = RobotList.get_robot(params)
    if not args.camera_type:
        if not robot.camera:
            print("Robot camera is not set")
        else:
            print(f"Current robot camera: {robot.camera}")
        return True
    # Update the camera
    robot.camera = args.camera_type
    RobotList.update_robot(params, robot)
    print(TerminalFormatter.color_text(f"Camera set to: {robot.camera}", color='green'))
    return True


def robot_set_lidar(platform, params: Params, args):
    """Configure the lidar."""
    # Check if the lidar is provided
    robot = RobotList.get_robot(params)
    if not args.lidar_type:
        if not robot.lidar:
            print("Robot lidar is not set")
        else:
            print(f"Current robot lidar: {robot.lidar}")
        return True
    # Update the lidar
    robot.lidar = args.lidar_type
    RobotList.update_robot(params, robot)
    print(TerminalFormatter.color_text(f"Lidar set to: {robot.lidar}", color='green'))
    return True


def robot_configure_engines(platform, params: Params, args):
    """Configure the robot engines."""
    def print_options(robot):
        all_engines = sorted(list(set(robot.engines + ENGINES_CHOICES)))
        options = []
        for i, engine in enumerate(all_engines):
            if engine in robot.engines:
                options.append(TerminalFormatter.color_text(f"{i + 1}. Engine {engine} (enabled)", color='green'))
            else:
                options.append(f"{i + 1}. Enable engine {engine}")
        options.append(f"{len(all_engines) + 1}. Exit")
        for option in options:
            print(option)

    def toggle_engine(robot, engine):
        if not robot.engines:
            robot.engines = []
        if engine in robot.engines:
            robot.engines.remove(engine)
        else:
            robot.engines.append(engine)
        RobotList.update_robot(params, robot)

    robot = RobotList.get_robot(params)
    if args.new_engine is not None:
        if args.new_engine not in robot.engines:
            robot.engines.append(args.new_engine)
            RobotList.update_robot(params, robot)
            print(TerminalFormatter.color_text(f"New engine {args.new_engine} added", color='green'))
        else:
            print(TerminalFormatter.color_text(f"Engine {args.new_engine} is already enabled", color='yellow'))
        return True

    try:
        while True:
            robot = RobotList.get_robot(params)
            print_options(robot)
            choice = input("Select an option: ")
            all_engines = sorted(list(set(robot.engines + ENGINES_CHOICES)))
            if choice.isdigit() and 1 <= int(choice) <= len(all_engines):
                toggle_engine(robot, all_engines[int(choice) - 1])
            elif choice == str(len(all_engines) + 1):
                break
            else:
                print(TerminalFormatter.color_text("Invalid option, please try again", color='red'))
    except KeyboardInterrupt:
        print(TerminalFormatter.color_text("Process interrupted by user", color='yellow'))
        return False


def robot_reset(platform, params: Params, args):
    """Reset the robot configuration."""
    # Reset the robot configuration
    RobotList.remove_robot(params)
    print(TerminalFormatter.color_text("Robot configuration reset", color='green'))
    return True


def control_keyboard(platform, params: Params, args):
    """Control the robot using the keyboard."""
    nanosaur_ws_path = workspace.get_workspace_path(params, params['ws_simulation_name'])
    bash_file = f'{nanosaur_ws_path}/install/setup.bash'
    # Read the robot name
    robot = RobotList.get_robot(params)
    print(TerminalFormatter.color_text(f"Control the robot {robot.name} using the keyboard", color='green'))
    subprocess.run(f'source {bash_file} && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/{robot.name}/key_vel',
                   shell=True, executable='/bin/bash')


def robot_display(platform, params: Params, args):
    """Display the robot configuration."""
    nanosaur_ws_path = workspace.get_workspace_path(params, params['ws_simulation_name'])
    bash_file = f'{nanosaur_ws_path}/install/setup.bash'
    # Read the robot name
    robot = RobotList.get_robot(params)
    print(TerminalFormatter.color_text(f"Display the robot {robot.name}", color='green'))
    try:
        subprocess.run(f'source {bash_file} && ros2 launch nanosaur_visualization robot_display.launch.py robot_name:={robot.name}',
                       shell=True, executable='/bin/bash')
    except KeyboardInterrupt:
        print(TerminalFormatter.color_text("Process interrupted by user", color='yellow'))
    return True
# EOF
