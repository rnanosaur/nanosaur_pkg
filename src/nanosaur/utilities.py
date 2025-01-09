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
import yaml
from nanosaur.prompt_colors import TerminalFormatter


class Params:
    
    @classmethod
    def load(cls, default_params, params_file=None):
        # Load parameters from YAML file if it exists
        if os.path.exists(params_file):
            with open(params_file, 'r') as file:
                params_dict = yaml.safe_load(file)
        else:
            params_dict = default_params

        return cls(params_dict, params_file)
    
    def __init__(self, params_dict, params_file=None):
        self._params_dict = params_dict
        self.params_file = params_file
        for key, value in params_dict.items():
            setattr(self, key, value)

    def __getitem__(self, key):
        return self._params_dict[key]

    def __setitem__(self, key, value):
        self._params_dict[key] = value
        setattr(self, key, value)
        # save the new value in the file
        if self.params_file:
            self.save(self.params_file)

    def __delitem__(self, key):
        del self._params_dict[key]
        delattr(self, key)

    def __contains__(self, key):
        return key in self._params_dict

    def save(self, file_path):
        with open(file_path, 'w') as file:
            yaml.dump(self._params_dict, file)

    def get(self, key, default=None):
        return getattr(self, key, default)
    
    def set(self, key, value):
        setattr(self, key, value)
        # save the new value in the file
        if self.params_file:
            self.save(self.params_file)
        return value

    def items(self):
        return self._params_dict.items()


def require_sudo(func):
    def wrapper(*args, **kwargs):
        if os.geteuid() != 0:
            print(TerminalFormatter.color_text("This script must be run as root. Please use 'sudo'.", color='red'))
            return False
        return func(*args, **kwargs)
    return wrapper
# EOF
