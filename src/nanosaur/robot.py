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

    def __init__(self, robot_config=None):
        if robot_config is None:
            robot_config = DEFAULT_ROBOT_CONFIG
        # Load the robot configuration
        for key, value in robot_config.items():
            setattr(self, key, value)

    def __repr__(self):
        return f"Robot(name={self.name}, domain_id={self.domain_id})"

    def to_dict(self):
        return self.__dict__

class RobotList:
    
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
    
    def add_robot(self, robot):
        self.robots.append(robot)
    
    def remove_robot(self, robot):
        self.robots.remove(robot)
        
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
    del params['robot']
    print(TerminalFormatter.color_text("Robot configuration reset", color='green'))
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
