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
import subprocess
import pty
import select
import termios
import tty
import signal
from python_on_whales import docker, DockerException
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import get_nanosaur_home
from git import Repo, GitCommandError
import shutil

ros2_distro = 'humble'
ros2_sources = f'/opt/ros/{ros2_distro}/setup.bash'

ISAAC_ROS_COMMON_FOLDER = 'isaac_ros_common'
ISAAC_ROS_COMMON_REPO = 'https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common'


def is_ros2_version_installed(version, folder="/opt/ros"):
    """
    Check if a specific ROS 2 version is installed.

    :param version: The ROS 2 version to check (e.g., 'foxy', 'humble').
    :param folder: Path to the folder where ROS distributions are installed (default: /opt/ros).
    :return: True if the specified version is installed, False otherwise.
    """
    version_path = os.path.join(folder, version)
    return os.path.exists(version_path) and os.path.isdir(version_path)


def run_docker_ros(docker_image):

    docker.run(docker_image, command="bash")


def run_docker_isaac_ros(workspace_path, auto_commands=[]):
    nanosaur_home_path = get_nanosaur_home()
    # Path to the Isaac ROS common package
    isaac_ros_common_path = os.path.join(nanosaur_home_path, ISAAC_ROS_COMMON_FOLDER)

    # Path to the script you want to run
    command = "./scripts/run_dev.sh"
    # Build the command arguments
    args = ["-d", workspace_path, '-a', f"-v {nanosaur_home_path}/shared_src:/workspaces/shared_src"]

    # Get the path to the Isaac ROS common package
    os.chdir(isaac_ros_common_path)
    # print(f"Changed directory to: {isaac_ros_common_path}")

    # Check if .isaac_ros_common-config exists and remove it
    config_path = os.path.join(isaac_ros_common_path, 'scripts', '.isaac_ros_common-config')
    if os.path.exists(config_path):
        os.remove(config_path)
        print(TerminalFormatter.color_text(f"Removed existing config file: {config_path}", color='yellow'))

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
                try:
                    output = os.read(master_fd, 1024)
                    if not output:  # If the subprocess exits, stop the loop
                        break
                    # Filter and render subprocess output to avoid cursor resets
                    filtered_output = output.replace(b"\x1b[2J", b"")  # Remove clear screen sequences
                    sys.stdout.buffer.write(filtered_output)
                    sys.stdout.buffer.flush()
                except OSError as e:
                    if e.errno == 5:  # Input/output error
                        break
                    else:
                        raise
    finally:
        restore_terminal()  # Restore the original terminal settings
        os.close(master_fd)

    print(TerminalFormatter.color_text("Dev script finished", color='green'))


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


def deploy_docker_image(dockerfile_path, tag_image):
    try:
        print(TerminalFormatter.color_text(f"Building Docker image {tag_image}", color='magenta', bold=True))
        docker.build(
            get_nanosaur_home(),
            file=dockerfile_path,
            tags=tag_image
        )
        print(TerminalFormatter.color_text("Docker image built successfully", color='green'))
        return True
    except DockerException as e:
        print(TerminalFormatter.color_text(f"Error building Docker image: {e}", color='red'))
        return False


def deploy_docker_isaac_ros(isaac_ros_ws_path, tags, release_tag_name, debug=False) -> bool:
    """
    Deploys the Isaac ROS Docker image.

    Args:
        isaac_ros_ws_path (str): Path to the Isaac ROS workspace.
        tags (list): List of tags for the Docker image.
        release_tag_name (str): Release tag name for the Docker image.
        debug (bool): Flag to enable debug mode.

    Returns:
        bool: True if the deployment is successful, False otherwise.
    """
    # Define shared source path
    shared_path = os.path.join(get_nanosaur_home(), 'shared_src')
    # List of source folders to include in the workspace
    src_folders = [
        shared_path,
        os.path.join(isaac_ros_ws_path, 'src')
    ]
    ws_dir_list = '--ws-src ' + ' --ws-src '.join(src_folders)
    # Path to the Isaac ROS common package
    isaac_ros_common_path = os.path.join(get_nanosaur_home(), ISAAC_ROS_COMMON_FOLDER)
    # Path to the Nanosaur Docker scripts
    nanosaur_docker_path = os.path.join(shared_path, 'nanosaur', 'nanosaur', 'docker')
    # Build the command to run the Docker build script
    debug_flag = '--debug' if debug else ''
    tags_name = '.'.join(tags)
    command = f"{nanosaur_docker_path}/docker_build_isaac_ros.sh {debug_flag} -d {tags_name} -c {isaac_ros_common_path} -i {release_tag_name} {ws_dir_list}"

    try:
        print(TerminalFormatter.color_text(f"Deploying {release_tag_name}", color='magenta', bold=True))
        # Run the command and stream the output live
        process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        # Stream output live
        for line in process.stdout:
            print(line.decode('utf-8', errors='replace'), end="")
        # Wait for the process to finish
        process.wait()
        # Check the exit status of the command
        if process.returncode != 0:
            print(TerminalFormatter.color_text(f"Command failed with return code: {process.returncode}", color='red'))
            return False
        else:
            print(TerminalFormatter.color_text("Command completed successfully", color='green'))
            return True
    except KeyboardInterrupt:
        print(TerminalFormatter.color_text("Process interrupted by user", color='red'))
        process.terminate()
        process.wait()
        return False
    except Exception as e:
        print(f"An error occurred while running the command: {e}")
        return False


def manage_isaac_ros_common_repo(nanosaur_home_path: str, isaac_ros_branch: str, force) -> bool:
    # Path to the Isaac ROS common package
    isaac_ros_common_path = os.path.join(nanosaur_home_path, ISAAC_ROS_COMMON_FOLDER)

    def update_existing_repo():
        try:
            print(TerminalFormatter.color_text(f"Directory '{isaac_ros_common_path}' already exists. Pulling latest changes from branch '{isaac_ros_branch}'.", color='yellow'))
            repo = Repo(isaac_ros_common_path)
            repo.git.checkout(isaac_ros_branch)
            repo.remotes.origin.pull()
            print(TerminalFormatter.color_text("Repository updated successfully", color='green'))
            return True
        except GitCommandError as e:
            print(TerminalFormatter.color_text(f"Error updating repository: {e}", color='red'))
            return False

    def clone_new_repo():
        try:
            print(TerminalFormatter.color_text(f"Cloning isaac_ros_common into '{isaac_ros_common_path}' from branch '{isaac_ros_branch}'", color='magenta', bold=True))
            Repo.clone_from(ISAAC_ROS_COMMON_REPO, isaac_ros_common_path, branch=isaac_ros_branch)
            print(TerminalFormatter.color_text("Clone completed successfully", color='green'))
            return True
        except GitCommandError as e:
            print(TerminalFormatter.color_text(f"Error cloning repository from branch '{isaac_ros_branch}': {e}", color='red'))
            return False

    # Check if the Isaac ROS common package already exists
    if os.path.exists(isaac_ros_common_path):
        if force:
            print(TerminalFormatter.color_text(f"Deleting existing directory '{isaac_ros_common_path}'.", color='yellow'))
            shutil.rmtree(isaac_ros_common_path)
        return update_existing_repo()
    else:
        return clone_new_repo()
# EOF
