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

import subprocess
from nanosaur import workspace
from nanosaur import docker
from nanosaur import simulation
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList, Robot


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


def robot_reset(platform, params: Params, args):
    """Reset the robot configuration."""
    # Reset the robot configuration
    RobotList.remove_robot(params)
    print(TerminalFormatter.color_text("Robot configuration reset", color='green'))
    return True


def robot_new(platform, params: Params, args):
    """Add a new robot configuration."""
    # Create a new robot configuration
    robot = Robot(name=args.name)
    if RobotList.add_robot(params, robot):
        print(TerminalFormatter.color_text("New robot configuration added", color='green'))
        return True
    print(TerminalFormatter.color_text("Robot configuration already exists", color='red'))


def robot_idx_set(platform, params: Params, args):
    """Set the robot index."""
    if args.robot_name is not None:
        idx = RobotList.get_idx_by_name(params, args.robot_name)
        if idx is not None:
            params['robot_idx'] = idx
            print(TerminalFormatter.color_text(f"Robot index set to: {idx}", color='green'))
            return True
        print(TerminalFormatter.color_text("Robot not found", color='red'))
    else:
        robot = RobotList.load(params)._get_robot_by_idx(params.get('robot_idx', 0))
        print(f"Current robot index: {params.get('robot_idx', 0)} name: {robot.name}")


def robot_list(platform, params: Params, args):
    """List the robot configurations."""
    robot_list = RobotList.load(params)
    for i, robot in enumerate(robot_list.robots):
        if i == params.get('robot_idx', 0):
            print(TerminalFormatter.color_text(f"{i}. {robot}", color='green'))
        else:
            print(f"{i}. {robot}")
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
