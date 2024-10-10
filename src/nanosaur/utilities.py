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
import shlex
import subprocess
from nanosaur.prompt_colors import TerminalFormatter


def run_command(command):    
    try:
        # Run the command and capture the output
        result = subprocess.run(command, shell=True, capture_output=True, text=True)

        # Check if there is any output or error message
        if result.stdout:
            print(TerminalFormatter.color_text(result.stdout, color='green'))
        if result.stderr:
            print(TerminalFormatter.color_text(result.stderr, color='red'))
            return False
    except Exception as e:
        print(f"An error occurred while running the vcs import command: {e}")
        return False
    
    return True


def run_command_live(command):
    """
    General function to execute a command and stream output live.
    :param command: List of command arguments (as needed by subprocess.Popen).
    """
    try:
        command = shlex.split(command)
        # Run the command and stream the output live
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, stdin=open(os.devnull), preexec_fn=os.setsid)

        # Stream output live
        for line in process.stdout:
            print(line, end="")  # Print stdout line-by-line
        
        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line, color='red'), end="")  # Print stderr (errors) in red

        # Check the exit status of the command
        if process.returncode != 0:
            print(TerminalFormatter.color_text(process.returncode, color='red'))
        else:
            print(TerminalFormatter.color_text("Command completed successfully", color='green'))

        return process.returncode == 0
    
    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False


def run_rosdep(folder_path):
    return run_command_live(f"rosdep install --from-paths {folder_path}/src --ignore-src -r -y")


def run_colcon_build(folder_path):
    # Move to the folder_path and run the colcon build command
    try:
        os.chdir(folder_path)
        print(f"Changed directory to: {folder_path}")
        
        # Run the colcon build command with the necessary flags
        return run_command_live("colcon build --symlink-install --merge-install")
    
    except Exception as e:
        print(f"An error occurred while running the colcon build command: {e}")
        return False
# EOF
