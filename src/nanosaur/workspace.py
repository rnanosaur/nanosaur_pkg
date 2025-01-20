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
import yaml
import subprocess
import pty
import select
import termios
import tty
import signal
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, get_nanosaur_home, create_nanosaur_home, require_sudo_password, conditional_sudo_password

ros2_distro = 'humble'
ros2_sources = f'/opt/ros/{ros2_distro}/setup.bash'

# Default colcon settings
COLCON_DEFAULTS = {
    'build': {
        'base-paths': ['src', '../shared_src'],
        'symlink-install': True,
    }
}


def run_dev_script(platform, params: Params, args):

    perception_path = get_workspace_path(params, params['ws_perception_name'])
    isaac_ros_common_path = os.path.join(perception_path, 'src', 'isaac_ros_common')
    # Get the path to the Isaac ROS common package
    os.chdir(isaac_ros_common_path)
    print(f"Changed directory to: {isaac_ros_common_path}")

    nanosaur_home_path = get_nanosaur_home(params['nanosaur_home'])

    # Path to the script you want to run
    command = "./scripts/run_dev.sh"

    # Build the command arguments
    args = ["-d", nanosaur_home_path]

    # Optional: Commands to run automatically after the script starts
    auto_commands = [f"cd {params['ws_perception_name']}"]

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
    cmd = [command] + (args if args else [])

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


def clean_workspace(nanosaur_ws_name, password):
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    # Get the current user's home directory
    user_home_dir = os.path.expanduser("~")

    # Create the full path for the workspace folder in the user's home
    # directory
    workspace_path = os.path.join(user_home_dir, nanosaur_ws_name)

    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        subfolders = ['build', 'install', 'log']
        subfolders_exist = all(os.path.exists(os.path.join(workspace_path, subfolder)) for subfolder in subfolders)

        if subfolders_exist:
            print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' and subfolders exist. Cleaning build, install and log folders", color='yellow'))
            result = False
            try:
                child = pexpect.spawn(f"sudo rm -rf {workspace_path}/build {workspace_path}/install {workspace_path}/log", encoding='utf-8', timeout=None)
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

                # Stream all command output to the terminal in real time
                child.logfile = sys.stdout

            except pexpect.ExceptionPexpect as e:
                print(TerminalFormatter.color_text(f"Error running rm {str(e)}", color='red'))
                result = False
            finally:
                # Ensure the process is closed
                if child.isalive():
                    child.close()
            print(TerminalFormatter.color_text(f"Workspace '{workspace_path}' cleaned up.", color='green'))
            return result
    else:
        print(TerminalFormatter.color_text(f"Folder '{workspace_path}' does not exist.", color='yellow'))
    return False


def get_workspace_path(params: Params, ws_name):
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home(params['nanosaur_home'])
    # Create the full path for the workspace folder in the user's home directory
    workspace_path = os.path.join(nanosaur_home_path, ws_name)

    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        return workspace_path
    else:
        return None


def create_workspace(nanosaur_home_path, ws_name):
    ws_name_path = os.path.join(nanosaur_home_path, ws_name)
    ws_name_path_src = os.path.join(ws_name_path, "src")
    # Check if folder exists, if not, create it
    if not os.path.exists(ws_name_path_src):
        os.makedirs(ws_name_path_src)
        print(TerminalFormatter.color_text(f"Workspace '{ws_name}' created in {nanosaur_home_path}.", color='green'))
    else:
        print(TerminalFormatter.color_text(f"Workspace '{ws_name}' already exists.", color='yellow'))
    # Save the default colcon settings
    with open(f"{ws_name_path}/colcon_defaults.yaml", 'w') as file:
        yaml.dump(COLCON_DEFAULTS, file)
    return ws_name_path


def build_workspace(nanosaur_raw_github_repo, branch, workspace_path, rosinstall_name, password, skip_rosdep=False, skip_build=False):
    # Download rosinstall for this device
    url = f"{nanosaur_raw_github_repo}/{branch}/nanosaur/rosinstall/{rosinstall_name}.rosinstall"
    rosinstall_path = download_rosinstall(url, workspace_path, f"{rosinstall_name}.rosinstall")
    if rosinstall_path is not None:
        print(TerminalFormatter.color_text(f"- Fill {rosinstall_name} from {rosinstall_name}.rosinstall", bold=True))
    else:
        print(TerminalFormatter.color_text(f"Failed to download {rosinstall_name}.rosinstall Exiting...", color='red'))
        return False
    # Import workspace
    print(TerminalFormatter.color_text(f"- Import workspace from {rosinstall_name}.rosinstall", bold=True))
    # run vcs import to sync the workspace
    vcs_status = run_vcs_import(workspace_path, rosinstall_path)
    if not vcs_status:
        print(TerminalFormatter.color_text("Failed to import workspace", color='red'))
        return False
    # rosdep workspace
    if not skip_rosdep:
        print(TerminalFormatter.color_text(f"- Install all dependencies on workspace {workspace_path}", bold=True))
        if not run_rosdep(workspace_path, password):
            print(TerminalFormatter.color_text("Failed to install dependencies", color='red'))
            return False
    # Build environment
    if not skip_build:
        print(TerminalFormatter.color_text(f"- Build workspace {workspace_path}", bold=True))
        if not run_colcon_build(workspace_path):
            print(TerminalFormatter.color_text("Failed to build workspace", color='red'))
            return False
    # All fine
    return True


@require_sudo_password
def create_developer_workspace(platform, params: Params, args, password=None):
    # determine the device type
    device_type = "robot" if platform['Machine'] == 'jetson' else "desktop"
    # Get the Nanosaur home folder and branch
    nanosaur_home = params['nanosaur_home']
    nanosaur_raw_github_repo = params['nanosaur_raw_github_repo']
    branch = params['nanosaur_branch']
    # Create the Nanosaur home folder
    nanosaur_home_path = create_nanosaur_home(nanosaur_home)

    # Create the shared source folder
    nanosaur_shared_src = os.path.join(nanosaur_home_path, "shared_src")
    # Check if folder exists, if not, create it
    if not os.path.exists(nanosaur_shared_src):
        os.makedirs(nanosaur_shared_src)
        print(TerminalFormatter.color_text(f"Shared src folder created in {nanosaur_home_path}.", color='green'))
    # Download rosinstall for this device
    url = f"{nanosaur_raw_github_repo}/{branch}/nanosaur/rosinstall/shared.rosinstall"
    rosinstall_path = download_rosinstall(url, nanosaur_shared_src, "shared.rosinstall")
    if rosinstall_path is not None:
        print(TerminalFormatter.color_text("- Fill shared src from shared.rosinstall", bold=True))
    else:
        print(TerminalFormatter.color_text("Failed to download rosinstall file. Exiting...", color='red'))
        return False
    # Import workspace
    print(TerminalFormatter.color_text("- Import workspace from shared.rosinstall", bold=True))
    # run vcs import to sync the workspace
    vcs_status = run_vcs_import(nanosaur_home_path, rosinstall_path, src_folder="shared_src")
    if not vcs_status:
        print(TerminalFormatter.color_text("Failed to import workspace", color='red'))
        return False

    # Make the robot workspace
    if device_type == "robot" or args.all_platforms:
        # Make the robot workspace
        ws_name_path = create_workspace(nanosaur_home_path, params['ws_robot_name'])
        if not build_workspace(nanosaur_raw_github_repo, branch, ws_name_path, device_type, password):
            return False
    # Make the simulation workspace
    if device_type == "desktop" or args.all_platforms:
        # Make the simulation workspace
        ws_name_path = create_workspace(nanosaur_home_path, params['ws_simulation_name'])
        if not build_workspace(nanosaur_raw_github_repo, branch, ws_name_path, device_type, password):
            return False

    # Make the perception workspace
    ws_name_path = create_workspace(nanosaur_home_path, params['ws_perception_name'])
    build_workspace(branch, ws_name_path, 'perception', password, skip_rosdep=True, skip_build=True)
    # Set params in developer mode
    params['developer_mode'] = True


def download_rosinstall(url, folder_path, file_name):
    # Create the full file path
    file_path = os.path.join(folder_path, file_name)

    # Check if the file already exists
    if os.path.exists(file_path):
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


def run_vcs_import(workspace_path, rosinstall_path, src_folder="src"):
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


def run_rosdep(folder_path, password):
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


def run_colcon_build(folder_path):

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


@require_sudo_password
def clean(platform, params: Params, args, password=None):
    if args.workspace is not None:
        workspace = args.workspace
    else:
        workspace = "robot" if platform['Machine'] == 'jetson' else "desktop"

    if workspace == 'robot' or args.all_platforms:
        robot_ws_name = params['ws_robot_name']
        robot_ws_path = get_workspace_path(params, robot_ws_name)
        if robot_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Clean robot workspace {robot_ws_name}", bold=True))
            # Clean workspace
            clean_workspace(robot_ws_path, password)

    if workspace == 'desktop' or args.all_platforms:
        simulation_ws_name = params['ws_simulation_name']
        simulation_ws_path = get_workspace_path(params, simulation_ws_name)
        if simulation_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Clean simulation workspace {simulation_ws_name}", bold=True))
            # Clean workspace
            clean_workspace(simulation_ws_path, password)

    if workspace == 'perception' or args.all_platforms:
        perception_ws_name = params['ws_perception_name']
        perception_ws_path = get_workspace_path(params, perception_ws_name)
        if perception_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Clean perception workspace {perception_ws_name}", bold=True))
            # Clean workspace
            clean_workspace(perception_ws_path, password)

    return True


@conditional_sudo_password
def update(platform, params: Params, args, password=None):
    if args.workspace is not None:
        workspace = args.workspace
    else:
        workspace = "robot" if platform['Machine'] == 'jetson' else "desktop"

    if workspace == 'robot' or args.all_platforms:
        robot_ws_name = params['ws_robot_name']
        robot_ws_path = get_workspace_path(params, robot_ws_name)
        if robot_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Update robot workspace {robot_ws_name}", bold=True))
            # Build environment
            if not run_colcon_build(robot_ws_path):
                return False

    if workspace == 'desktop' or args.all_platforms:
        simulation_ws_name = params['ws_simulation_name']
        simulation_ws_path = get_workspace_path(params, simulation_ws_name)
        if simulation_ws_path is not None:
            print(TerminalFormatter.color_text(f"- Update simulation workspace {simulation_ws_name}", bold=True))
            # Build environment
            if not run_colcon_build(simulation_ws_path):
                return False

    return True
# EOF
