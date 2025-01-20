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


from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList, Robot


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


def robot_remove(platform, params: Params, args):
    """Remove a robot configuration."""
    if args.robot_name is None:
        robot = RobotList.load(params)._get_robot_by_idx(params.get('robot_idx', 0))
        args.robot_name = robot.name

    confirmation = input(
        f"Are you sure you want to {TerminalFormatter.color_text('**remove**', color='red', bold=True)} the robot configuration for {TerminalFormatter.color_text(args.robot_name, color='green', bold=True)}? (yes/no): ")
    if confirmation.lower() == 'yes':
        RobotList.remove_robot(params, params.get('robot_idx', 0))
        print(TerminalFormatter.color_text("Robot configuration removed", color='green'))
        return True
    else:
        print(TerminalFormatter.color_text("Robot configuration removal canceled", color='yellow'))


def robot_list(platform, params: Params, args):
    """List the robot configurations."""
    RobotList.load(params).print_all_robots(params.get('robot_idx', 0))
    return True
# EOF
