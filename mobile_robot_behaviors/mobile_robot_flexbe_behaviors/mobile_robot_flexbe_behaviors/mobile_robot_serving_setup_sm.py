#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2023 ggh-png
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define mobile_robot_serving_setup.

Created on Tue Dec 05 2023
@author: ggh-png
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from mobile_robot_flexbe_states.init_pose_state import InitPoseState
from mobile_robot_flexbe_states.joystick_state import JoystickState
from mobile_robot_flexbe_states.set_waypoint_state import SetWaypointState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class mobile_robot_serving_setupSM(Behavior):
    """
    Define mobile_robot_serving_setup.

    Mobile Robot Serviing Setup

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'mobile_robot_serving_setup'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        InitPoseState.initialize_ros(node)
        JoystickState.initialize_ros(node)
        SetWaypointState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        waypoint = [[0.0, -1.0, 1.0, 1.0],[2.0, -1.0, 1.0, 1.0],[-1.5, 2.5, 1.0, 1.0],[1.9, 3.0, 1.0, 1.0]]
        # x:660 y:315
        _state_machine = OperatableStateMachine(outcomes=['setup_finished'], output_keys=['waypoint'])
        _state_machine.userdata.waypoint = waypoint

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:93 y:80
            OperatableStateMachine.add('init_pose_state',
                                       InitPoseState(init_pose_topic='/initialpose', init_pose_cmd_topic='/init_pose_cmd', waypoint=[0.0, 0.0, 0.0, 1.0]),
                                       transitions={'done': 'joystick_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:283 y:147
            OperatableStateMachine.add('joystick_state',
                                       JoystickState(set_angular_vel=0.5, set_linear_vel=0.5, joystick_topic='/joy_cmd', cmd_vel_topic='/cmd_vel', set_vel_topic='/set_vel'),
                                       transitions={'done': 'set_waypoint_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:442 y:257
            OperatableStateMachine.add('set_waypoint_state',
                                       SetWaypointState(amcl_pose_topic='/amcl_pose', navigator_topic='/navi_cmd'),
                                       transitions={'done': 'setup_finished'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
