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
import sys
import pexpect
import requests
import subprocess
import pty
import select
import termios
import tty
import signal
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, get_nanosaur_home

ros2_distro = 'humble'
ros2_sources = f'/opt/ros/{ros2_distro}/setup.bash'


ISAAC_ROS_DISTRO_SUFFIX = "ros2_humble"
NANOSAUR_DOCKERFILE_SUFFIX = "nanosaur"


def run_dev_script(params, host_workspace_path, workspace_path):
    isaac_ros_common_path = os.path.join(host_workspace_path, 'src', 'isaac_ros_common')
    # Get the path to the Isaac ROS common package
    os.chdir(isaac_ros_common_path)
    print(f"Changed directory to: {isaac_ros_common_path}")

    nanosaur_home_path = get_nanosaur_home()

    # Path to the script you want to run
    command = "./scripts/run_dev.sh"

    # Build the command arguments
    args = ["-d", nanosaur_home_path]

    # Optional: Commands to run automatically after the script starts
    auto_commands = [f"cd {workspace_path}"]

    # Save the original terminal settings
    original_termios = termios.tcgetattr(sys.stdin)

    def set_raw_mode():
        """Set terminal to raw mode to handle special characters."""
        tty.setraw(sys.stdin.fileno())

    def restore_terminal():
        """Restore original terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSANOW, original_termios)

    def handle_signal(signum, frame):
        """Forward terminal signals to the subprocess."""
        os.kill(child_pid, signum)

    # Open a pseudo-terminal
    master_fd, slave_fd = pty.openpty()

    # Prepare the command and arguments
    cmd = [command] + (args or [])

    # Fork the process
    child_pid = os.fork()
    if child_pid == 0:  # Child process
        os.close(master_fd)  # Close master in the child process
        os.dup2(slave_fd, sys.stdin.fileno())  # Use slave as stdin
        os.dup2(slave_fd, sys.stdout.fileno())  # Use slave as stdout
        os.dup2(slave_fd, sys.stderr.fileno())  # Use slave as stderr
        os.execvp(cmd[0], cmd)  # Execute the command
    else:  # Parent process
        os.close(slave_fd)  # Close slave in the parent process

    try:
        set_raw_mode()

        # Forward terminal signals to the subprocess
        for sig in (signal.SIGINT, signal.SIGTSTP, signal.SIGQUIT):
            signal.signal(sig, handle_signal)

        # Automatically send commands if specified
        if auto_commands:
            for command in auto_commands:
                os.write(master_fd, (command + '\n').encode())

        while True:
            # Wait for input from the user or output from the subprocess
            rlist, _, _ = select.select([sys.stdin, master_fd], [], [])

            if sys.stdin in rlist:  # User input
                user_input = os.read(sys.stdin.fileno(), 1024)
                os.write(master_fd, user_input)  # Forward input to the subprocess

            if master_fd in rlist:  # Subprocess output
                output = os.read(master_fd, 1024)
                if not output:  # If the subprocess exits, stop the loop
                    break
                # Filter and render subprocess output to avoid cursor resets
                filtered_output = output.replace(b"\x1b[2J", b"")  # Remove clear screen sequences
                sys.stdout.buffer.write(filtered_output)
                sys.stdout.buffer.flush()
    finally:
        restore_terminal()  # Restore the original terminal settings
        os.close(master_fd)

    print(TerminalFormatter.color_text("Dev script finished", color='green'))


def download_rosinstall(url, folder_path, file_name, force=False) -> str:
    # Create the full file path
    file_path = os.path.join(folder_path, file_name)

    # Check if the file already exists
    if not force and os.path.exists(file_path):
        print(TerminalFormatter.color_text(f"File '{file_name}' already exists in '{folder_path}'. Skip download", color='yellow'))
        return file_path  # Cancel download

    # Send a request to download the file
    response = requests.get(url)

    if response.status_code == 200:
        # Save the file in the workspace folder
        file_path = os.path.join(folder_path, file_name)
        with open(file_path, 'wb') as file:
            file.write(response.content)
        print(TerminalFormatter.color_text(f"File '{file_name}' downloaded successfully to '{folder_path}'.", color='green'))
        return file_path
    else:
        print(TerminalFormatter.color_text(f"Failed to download file. Status code: {response.status_code}", color='red'))
        return None


def run_vcs_import(workspace_path, rosinstall_path, src_folder="src") -> bool:
    try:
        # Run the command and stream the output live
        process = subprocess.Popen(
            f"vcs import {workspace_path}/{src_folder} < {rosinstall_path}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Stream output live
        for line in process.stdout:
            print(line.decode('utf-8'), end="")  # Print stdout line-by-line

        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line.decode('utf-8'), color='red'), end="")  # Print stderr (errors) in red

        # Check the exit status of the command
        if process.returncode != 0:
            print(TerminalFormatter.color_text(process.returncode, color='red'))
        else:
            print(TerminalFormatter.color_text("Command completed successfully", color='green'))

        return process.returncode == 0

    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False


def run_rosdep(folder_path, password) -> bool:
    if password is None:
        print(TerminalFormatter.color_text("Error: No password provided.", color='red'))
        return False
    result = False
    try:
        child = pexpect.spawn(f"bash -c 'source {ros2_sources} && rosdep install --from-paths {folder_path}/src --ignore-src -r -y'", encoding='utf-8', timeout=None)
        # Stream all command output to the terminal in real time
        child.logfile = sys.stdout
        # Wait for password prompt with timeout
        index = child.expect(
            ['password for', pexpect.EOF, pexpect.TIMEOUT], timeout=30)
        if index == 0:
            child.logfile = None  # Disable logging to hide password
            child.sendline(password)
            child.logfile = sys.stdout  # Re-enable logging
            # Wait for completion
            child.expect(pexpect.EOF, timeout=300)
            result = True
        elif index == 1:  # Command finished without password prompt
            print("Command finished without asking for a password.")
            result = True
        elif index == 2:  # Timeout
            print(TerminalFormatter.color_text("Error: Sudo prompt timed out. Please try again.", color='red'))
            result = False
    except pexpect.ExceptionPexpect as e:
        print(TerminalFormatter.color_text(f"Error running rosdep: {str(e)}", color='red'))
        result = False
    finally:
        # Ensure the process is closed
        if child.isalive():
            child.close()
    return result


def run_colcon_build(folder_path) -> bool:

    # Move to the folder_path and run the colcon build command
    try:
        os.chdir(folder_path)
        print(f"Changed directory to: {folder_path}")

        # Run the command and stream the output live
        process = subprocess.Popen(
            f"source {ros2_sources} && colcon build --symlink-install --merge-install",
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Stream output live
        for line in process.stdout:
            print(line.decode('utf-8'), end="")  # Print stdout line-by-line

        # Wait for the process to finish
        process.wait()

        # Stream any errors
        for line in process.stderr:
            print(TerminalFormatter.color_text(line.decode('utf-8'), color='red'), end="")  # Print stderr (errors) in red

        # Check the exit status of the command
        if process.returncode != 0:
            print(TerminalFormatter.color_text(process.returncode, color='red'))
        else:
            print(TerminalFormatter.color_text("Command completed successfully", color='green'))

        return process.returncode == 0

    except Exception as e:
        print(f"An error occurred while running the colcon build command: {e}")
        return False


def deploy_docker_perception(params: Params, perception_ws_path: str) -> bool:
    image_name = "nanosaur-perception"
    nanosaur_perception_path = os.path.join(perception_ws_path, 'src', 'nanosaur_perception')

    src_folders = [
        os.path.join(get_nanosaur_home(), 'shared_src'),
        os.path.join(perception_ws_path, 'src')
    ]

    try:
        os.chdir(nanosaur_perception_path)
        print(f"Changed directory to: {nanosaur_perception_path}")

        ws_dir_list = '--ws-src ' + ' --ws-src '.join(src_folders)
        command = f"scripts/docker_build.sh {ws_dir_list} --image-name {image_name}"

        process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        for line in process.stdout:
            print(line.decode('utf-8', errors='replace'), end="")

        process.wait()

        if process.returncode != 0:
            print(TerminalFormatter.color_text(process.returncode, color='red'))
            return False
        else:
            print(TerminalFormatter.color_text("Command completed successfully", color='green'))
            return True

    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False
# EOF
