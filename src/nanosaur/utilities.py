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

import os
import copy
import yaml
import pexpect
import getpass
from nanosaur.prompt_colors import TerminalFormatter

DEFAULT_ROBOT_CONFIG = {
    'name': 'nanosaur',
    'domain_id': 0,
    'camera': '',
    'lidar': '',
    'engines': [],
}

CAMERA_CHOICES = ['', 'realsense', 'zed']
LIDAR_CHOICES = ['', 'rplidar']
ENGINES_CHOICES = ['vlslam', 'nvblox', 'apriltag']


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
        # "Nanosaur[DID=](cover=AAA, )"
        attributes = ', '.join(f"{key}={value}" for key, value in self.__dict__.items() if key not in ['name', 'domain_id'] and value)
        return f"{self.name}[DID={self.domain_id}] {attributes}"

    def to_dict(self) -> dict:
        return self.__dict__

    def verbose(self):
        """Print the robot configuration."""
        print(TerminalFormatter.color_text("Robot:", bold=True))
        print(f"  {TerminalFormatter.color_text('Name:', bold=True)} {self.name}")
        print(f"  {TerminalFormatter.color_text('Domain ID:', bold=True)} {self.domain_id}")
        print(f"  {TerminalFormatter.color_text('Camera:', bold=True)} {self.camera or 'not set'}")
        print(f"  {TerminalFormatter.color_text('Lidar:', bold=True)} {self.lidar or 'not set'}")
        print(f"  {TerminalFormatter.color_text('Engines:', bold=True)} {', '.join(self.engines) if self.engines else 'not set'}")
        # Print other attributes
        if other_attributes := {
            key: value
            for key, value in self.__dict__.items()
            if key not in ['name', 'domain_id', 'camera', 'lidar', 'engines']
        }:
            print(f"  {TerminalFormatter.color_text('Other attributes:', bold=True)}")
            for key, value in other_attributes.items():
                print(f"    {TerminalFormatter.color_text(f'{key}:', bold=True)} {value}")


class RobotList:

    @classmethod
    def get_idx_by_name(cls, params, robot_name) -> int:
        return cls.load(params)._get_idx_by_name(robot_name)

    @classmethod
    def add_robot(cls, params, robot) -> bool:
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
    def update_robot(cls, params, robot) -> bool:
        robot_list = cls.load(params)
        idx = params.get('robot_idx', 0)
        if robot_list._update_robot(robot, idx):
            params['robots'] = robot_list.to_dict()
            return True
        return False

    @classmethod
    def get_robot(cls, params, idx=None) -> Robot:
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

    def _add_robot(self, robot) -> bool:
        def is_robot(robot):
            for r in self.robots:
                if r.name == robot.name:
                    return False
            return True

        if is_robot(robot):
            self.robots.append(robot)
            return True
        return False

    def _remove_robot(self, idx) -> bool:
        if idx < len(self.robots):
            del self.robots[idx]
            return True
        return False

    def _get_idx_by_name(self, name) -> int:
        return next((i for i, robot in enumerate(self.robots) if robot.name == name), None)

    def _get_robot_by_idx(self, idx) -> Robot:
        return self.robots[idx]

    def _get_robot_by_name(self, name) -> Robot:
        return next((robot for robot in self.robots if robot.name == name), None)

    def _update_robot(self, robot, idx) -> bool:
        if idx < len(self.robots):
            self.robots[idx] = robot
            return True
        return False

    def __repr__(self):
        return f"RobotList({self.robots})"

    def to_dict(self) -> list:
        return [robot.to_dict() for robot in self.robots]

    def print_all_robots(self):
        for robot in self.robots:
            print(robot)

class Params:

    @classmethod
    def load(cls, default_params, home_folder, params_file_name):
        params_file = Params.get_params_file(home_folder, params_file_name)
        # Load parameters from YAML file if it exists
        if os.path.exists(params_file):
            with open(params_file, 'r') as file:
                params_dict = yaml.safe_load(file)
        else:
            params_dict = default_params

        return cls(params_dict, home_folder, params_file)

    def __init__(self, params_dict, home_folder, params_file_name):
        self._params_dict = params_dict
        self._default_params = copy.deepcopy(params_dict)
        self.home_folder = home_folder
        self.params_file_name = params_file_name
        for key, value in params_dict.items():
            setattr(self, key, value)

    def __getitem__(self, key):
        return self._params_dict[key]

    def __setitem__(self, key, value):
        self._params_dict[key] = value
        setattr(self, key, value)
        # save the new value in the file
        self.save()

    def __delitem__(self, key):
        del self._params_dict[key]
        delattr(self, key)
        # save the new value in the file
        self.save()

    def __contains__(self, key):
        return key in self._params_dict

    def __repr__(self):
        return str(self._params_dict)

    def save(self):
        params_file = Params.get_params_file(self.home_folder, self.params_file_name)
        # Save the parameters to the file if they are different from the default
        if params_file and self._params_dict != self._default_params:
            # Get the current nanosaur's home directory
            create_nanosaur_home(self.home_folder)
            # Save the parameters to the file
            print(TerminalFormatter.color_text(f"Saving parameters to {self.params_file_name}", color='yellow'))
            with open(params_file, 'w') as file:
                yaml.dump(self._params_dict, file)

    @staticmethod
    def get_params_file(home_folder, params_file_name) -> str:
        return os.path.join(get_nanosaur_home(home_folder), params_file_name)

    def get(self, key, default=None):
        return getattr(self, key, default)

    def set(self, key, value):
        setattr(self, key, value)
        # save the new value in the file
        self.save()
        return value

    def items(self):
        return self._params_dict.items()


def create_nanosaur_home(nanosaur_home) -> str:
    # Get the current user's home directory
    user_home_dir = os.path.expanduser("~")
    # Create the full path for the workspace folder in the user's home directory
    nanosaur_home_path = os.path.join(user_home_dir, nanosaur_home)
    # Check if folder exists, if not, create it
    if not os.path.exists(nanosaur_home_path):
        os.makedirs(nanosaur_home_path)
        print(TerminalFormatter.color_text(f"Folder '{nanosaur_home_path}' created.", color='green'))
    return nanosaur_home_path


def get_nanosaur_home(nanosaur_home) -> str:
    # Get the current user's home directory
    user_home_dir = os.path.expanduser("~")
    return os.path.join(user_home_dir, nanosaur_home)


def require_sudo(func):
    def wrapper(*args, **kwargs):
        if os.geteuid() != 0:
            print(
                TerminalFormatter.color_text(
                    "This script must be run as root. Please use 'sudo'.",
                    color='red'))
            return False
        return func(*args, **kwargs)
    return wrapper


def require_sudo_password(func):
    def wrapper(*args, **kwargs):
        child = None
        try:
            # Get password
            print(TerminalFormatter.color_text("This function require user password to be executed.", color='yellow'))
            # Get the username
            username = os.getlogin()
            # Get the password
            password = getpass.getpass(prompt=f'{TerminalFormatter.color_text("[sudo]", bold=True)} password for {username}: ')
            # Test if the sudo password is valid
            child = pexpect.spawn("sudo -v")
            child.expect("password for")
            child.sendline(password)
            index = child.expect([pexpect.EOF, pexpect.TIMEOUT], timeout=10)
            if index != 0:  # Password not accepted
                print("Error: Incorrect sudo password. Please try again.")
                return False
            # Execute function with password
            return func(*args, password=password, **kwargs)
        except Exception as e:
            print(f"Validation error: {e}")
            return False
        except KeyboardInterrupt:
            print(TerminalFormatter.color_text("Exiting...", color='yellow'))
            return False
        finally:
            if child is not None:
                child.close()
    return wrapper


def conditional_sudo_password(func):
    def wrapper(platform, params, args):
        if args.force:
            return require_sudo_password(func)(platform, params, args)
        else:
            return func(platform, params, args)
    return wrapper
# EOF
