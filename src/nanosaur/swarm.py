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

import inquirer
from inquirer.themes import GreenPassion
import argparse
import logging
from nanosaur.prompt_colors import TerminalFormatter
from nanosaur.utilities import Params, RobotList, Robot


# Set up the logger
logger = logging.getLogger(__name__)


def parser_swarm_menu(subparsers: argparse._SubParsersAction, params: Params) -> argparse.ArgumentParser:
    # Get the robot index from the parameters
    try:
        current_robot_name = RobotList.current_robot(params).name
        # Subcommand: swarm (with a sub-menu for swarm operations)
        parser_swarm = subparsers.add_parser('swarm', help="Manage swarm Nanosaur robots")
        swarm_subparsers = parser_swarm.add_subparsers(dest='swarm_type', help="Robot operations")
        # Add robot status subcommand
        parser_robot_new = swarm_subparsers.add_parser('new', help="Get a new robot to control")
        parser_robot_new.add_argument('name', type=str, nargs='?', help="New robot name")
        parser_robot_new.set_defaults(func=robot_new)
        # Add robot set subcommand
        parser_robot_set = swarm_subparsers.add_parser('set', help=f"Set which robot to control [{current_robot_name}]")
        parser_robot_set.add_argument('robot_name', type=str, nargs='?', help="Name of the robot to control")
        parser_robot_set.set_defaults(func=robot_idx_set)
        # Add robot remove subcommand
        parser_robot_remove = swarm_subparsers.add_parser('remove', help="Remove a robot from the swarm")
        parser_robot_remove.add_argument('robot_name', type=str, nargs='?', help="Name of the robot to remove")
        parser_robot_remove.set_defaults(func=robot_remove)
        # Add robot list subcommand
        parser_robot_list = swarm_subparsers.add_parser('list', help="List all robots in the swarm")
        parser_robot_list.set_defaults(func=robot_list)
        return parser_swarm
    except IndexError:
        return None


def robot_new(platform, params: Params, args):
    """Add a new robot configuration."""

    def validate_name(_, x):
        if not x.isalnum():
            raise inquirer.errors.ValidationError("", reason="Name must contain only letters and numbers")
        return True

    questions = [
        inquirer.Text(
            'name',
            message="Enter the new robot name",
            default=args.name,
            validate=validate_name,
        )
    ]
    answers = inquirer.prompt(questions, theme=GreenPassion())
    robot = Robot(name=answers['name'])
    if RobotList.add_robot(params, robot):
        print(TerminalFormatter.color_text("New robot configuration added", color='green'))
        return True
    print(TerminalFormatter.color_text("Robot configuration already exists", color='red'))


def robot_idx_set(platform, params: Params, args):
    """Set the robot configuration."""
    # Load the robot list
    robots = RobotList.load(params)

    default = robots.get_robot(params.get('robot_idx', 0))
    if args.robot_name is not None:
        default = robots._get_robot_by_name(args.robot_name)

    options = [
        inquirer.List(
            'robot',
            message="Select robot in list",
            choices=robots.to_list(),
            default=default
        )
    ]
    # Get the selected robot
    answers = inquirer.prompt(options, theme=GreenPassion())
    if answers is None:
        return False
    # Get the selected robot and its index
    robot = answers['robot']
    idx = RobotList.get_idx_by_name(params, robot.name)
    print(f"Selected robot: {robot.name} with index {idx}")
    params['robot_idx'] = idx


def robot_remove(platform, params: Params, args):
    """Remove a robot configuration."""
    if args.robot_name is None:
        robot = RobotList.load(params).get_robot(params.get('robot_idx', 0))
        args.robot_name = robot.name

    formatted_robot_name = TerminalFormatter.color_text(args.robot_name, color='green', bold=True)
    questions = [
        inquirer.Confirm(
            'confirm',
            message=f"Confirm {TerminalFormatter.color_text('remove', color='red', bold=True)} config for {formatted_robot_name}?",
            default=False
        )
    ]
    answers = inquirer.prompt(questions, theme=GreenPassion())
    if answers is None:
        return False
    # Remove the robot configuration
    if answers['confirm']:
        RobotList.remove_robot(params, params.get('robot_idx', 0))
        print(TerminalFormatter.color_text("Robot configuration removed", color='green'))
        return True


def robot_list(platform, params: Params, args):
    """List the robot configurations."""
    RobotList.load(params).print_all_robots(params.get('robot_idx', 0))
    return True
# EOF
