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
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params
import copy


simulation_tools = {
    "Isaac Sim": "ros2 launch nanosaur_isaac_sim nanosaur_bridge.launch.py",
    "Gazebo": "ros2 launch nanosaur_gazebo nanosaur_bridge.launch.py"
}

DEFAULT_ROBOT_CONFIG = {
    'name': 'nanosaur',
    'domain_id': 0
}


class Robot:

    @classmethod
    def load(cls, params):
        return cls(params['robot']) if 'robot' in params and params['robot'] else cls()

    def __init__(self, robot_config=None, name=None):
        if robot_config is None:
            robot_config = copy.deepcopy(DEFAULT_ROBOT_CONFIG)
        if name is not None:
            robot_config['name'] = name
        # Load the robot configuration
        for key, value in robot_config.items():
            setattr(self, key, value)

    def __repr__(self):
        return f"Robot(name={self.name}, domain_id={self.domain_id})"

    def to_dict(self):
        return self.__dict__


class RobotList:

    @classmethod
    def get_idx_by_name(cls, params, robot_name):
        return cls.load(params)._get_idx_by_name(robot_name)

    @classmethod
    def add_robot(cls, params, robot):
        robot_list = cls.load(params)
        if robot_list._add_robot(robot):
            params['robots'] = robot_list.to_dict()
            params['robot_idx'] = params.get('robot_idx', 0) + 1
            return True
        return False

    @classmethod
    def remove_robot(cls, params):
        robot_list = cls.load(params)
        idx = params.get('robot_idx', 0)
        if idx == 0:
            if 'robots' in params:
                del params['robots']
            if 'robot_idx' in params:
                del params['robot_idx']
        else:
            robot_list._remove_robot(idx)
            params['robots'] = robot_list.to_dict()
            if 'robot_idx' in params and params['robot_idx'] > 0:
                params['robot_idx'] -= 1

    @classmethod
    def update_robot(cls, params, robot):
        robot_list = cls.load(params)
        idx = params.get('robot_idx', 0)
        if robot_list._update_robot(robot, idx):
            params['robots'] = robot_list.to_dict()
            return True
        return False

    @classmethod
    def get_robot(cls, params, idx=None):
        if idx is None:
            idx = params.get('robot_idx', 0)
        return cls.load(params)._get_robot_by_idx(idx)

    @classmethod
    def load(cls, params):
        return cls() if 'robots' not in params else cls(params['robots'])

    def __init__(self, robots=None):
        if robots is None:
            self.robots = [Robot()]
        else:
            self.robots = [Robot(robot) for robot in robots]

    def _add_robot(self, robot):
        def is_robot(robot):
            for r in self.robots:
                if r.name == robot.name:
                    return False
            return True

        if is_robot(robot):
            self.robots.append(robot)
            return True
        return False

    def _remove_robot(self, idx):
        if idx < len(self.robots):
            del self.robots[idx]
            return True
        return False

    def _get_idx_by_name(self, name):
        return next((i for i, robot in enumerate(self.robots) if robot.name == name), None)

    def _get_robot_by_idx(self, idx):
        return self.robots[idx]

    def _get_robot_by_name(self, name):
        return next((robot for robot in self.robots if robot.name == name), None)

    def _update_robot(self, robot, idx):
        if idx < len(self.robots):
            self.robots[idx] = robot
            return True
        return False

    def __repr__(self):
        return f"RobotList({self.robots})"

    def to_dict(self):
        return [robot.to_dict() for robot in self.robots]


def start_robot_simulation(params):
    nanosaur_ws_path = workspace.get_workspace_path(params['nanosaur_workspace_name'])
    bash_file = f'{nanosaur_ws_path}/install/setup.bash'
    # Check which simulation tool is selected
    if 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please run simulation set first.", color='red'))
        return False
    # Check if the simulation tool is valid and get the command
    command = simulation_tools[params['simulation_tool']]
    # Load the robot configuration
    robot = RobotList.get_robot(params)
    print(TerminalFormatter.color_text(f"Starting {robot.name} ID={robot.domain_id}", color='green'))
    try:
        # Combine sourcing the bash file with running the command
        process = subprocess.Popen(
            f"source {bash_file} && {command} namespace:={robot.name}",
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Stream output live
        for line in process.stdout:
            # Decode and print stdout line-by-line
            print(line.decode('utf-8'), end="")

        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line.decode('utf-8'), color='red'), end="")  # Print stderr (errors) in red

        return process.returncode == 0
    except KeyboardInterrupt:
        return False
    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False


def robot_start(platform, params: Params, args):
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    # Check the device type
    if device_type == "desktop":
        # Start the robot simulation
        start_robot_simulation(params)
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
    workspace_path = workspace.get_workspace_path(params['nanosaur_workspace_name'])
    bash_file = f'{workspace_path}/install/setup.bash'
    # Read the robot name
    robot = RobotList.get_robot(params)
    print(TerminalFormatter.color_text(f"Control the robot {robot.name} using the keyboard", color='green'))
    subprocess.run(f'source {bash_file} && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/{robot.name}/key_vel',
                   shell=True, executable='/bin/bash')
# EOF
