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


def start_robot_simulation(params):
    nanosaur_ws_path = workspace.get_workspace_path(params['nanosaur_workspace_name'])
    bash_file = f'{nanosaur_ws_path}/install/setup.bash'
    
    # Check which simulation tool is selected
    if 'simulation_tool' not in params:
        print(TerminalFormatter.color_text("No simulation tool selected. Please run simulation set first.", color='red'))
        return False
    
    command = simulation_tools[params['simulation_tool']]

    try:
        # Combine sourcing the bash file with running the command
        process = subprocess.Popen(
            f"source {bash_file} && {command} namespace:={params['robot_name']}",
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
    robot_name = params['robot_name']
    print(TerminalFormatter.color_text(f"Starting the {robot_name}", color='green'))
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
    if not args.robot_name:
        print(TerminalFormatter.color_text("Please provide a robot name", color='red'))
        return False

    # Update the robot name
    params['robot_name'] = args.robot_name
    print(TerminalFormatter.color_text(f"Robot name set to: {args.robot_name}", color='green'))
    return True


def robot_set_domain_id(platform, params: Params, args):
    """Configure the domain ID."""
    # Check if the domain ID is provided
    if not args.domain_id:
        print(TerminalFormatter.color_text("Please provide a domain ID", color='red'))
        return False

    # Update the domain ID
    params['domain_id'] = args.domain_id
    print(TerminalFormatter.color_text(f"Domain ID set to: {args.domain_id}", color='green'))
    return True


def control_keyboard(platform, params: Params, args):
    """Control the robot using the keyboard."""
    workspace_path = workspace.get_workspace_path(params['nanosaur_workspace_name'])
    bash_file = f'{workspace_path}/install/setup.bash'
    # Read the robot name
    robot_name = params['robot_name']
    print(TerminalFormatter.color_text(f"Control the robot {robot_name} using the keyboard", color='green'))
    subprocess.run(f'source {bash_file} && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/{robot_name}/key_vel',
                   shell=True, executable='/bin/bash')
# EOF
