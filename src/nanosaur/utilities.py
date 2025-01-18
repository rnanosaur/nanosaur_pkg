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
    def get_params_file(home_folder, params_file_name):
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


def create_nanosaur_home(nanosaur_home):
    # Get the current user's home directory
    user_home_dir = os.path.expanduser("~")
    # Create the full path for the workspace folder in the user's home directory
    nanosaur_home_path = os.path.join(user_home_dir, nanosaur_home)
    # Check if folder exists, if not, create it
    if not os.path.exists(nanosaur_home_path):
        os.makedirs(nanosaur_home_path)
        print(TerminalFormatter.color_text(f"Folder '{nanosaur_home_path}' created.", color='green'))
    return nanosaur_home_path


def get_nanosaur_home(nanosaur_home):
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
