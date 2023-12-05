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
Define mobile_robot_serving_fsm.

Created on Tue Dec 05 2023
@author: ggh-png
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from mobile_robot_flexbe_behaviors.mobile_robot_serving_setup_sm import mobile_robot_serving_setupSM
from mobile_robot_flexbe_states.get_order_table_num_state import GetOrderTableNnumState
from mobile_robot_flexbe_states.navigate_to_pose_state import NavigateToPoseState
from mobile_robot_flexbe_states.start_order_wait_state import StartOrder_waitState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class mobile_robot_serving_fsmSM(Behavior):
    """
    Define mobile_robot_serving_fsm.

    Mobile Robot Serving FSM

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'mobile_robot_serving_fsm'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        GetOrderTableNnumState.initialize_ros(node)
        NavigateToPoseState.initialize_ros(node)
        StartOrder_waitState.initialize_ros(node)
        self.add_behavior(mobile_robot_serving_setupSM, 'mobile_robot_serving_setup', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        go_home = 0
        # x:30 y:365, x:853 y:558
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.go_home = go_home

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('mobile_robot_serving_setup',
                                       self.use_behavior(mobile_robot_serving_setupSM, 'mobile_robot_serving_setup'),
                                       transitions={'setup_finished': 'get_order_table_num_state'},
                                       autonomy={'setup_finished': Autonomy.Inherit},
                                       remapping={'waypoint': 'waypoint'})

            # x:657 y:68
            OperatableStateMachine.add('go_to_kitchen',
                                       NavigateToPoseState(timeout=120.0, navi_to_pos_topic='/navigate_to_pose', emergency_topic='/emergency_cmd', amcl_pose_topic='/amcl_pose', navi_progress_topic='/remaining_waypoint', arrived_table_topic='/arrived_table', table_num=0),
                                       transitions={'failed': 'failed', 'done': 'get_order_table_num_state'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint', 'table_num': 'go_home'})

            # x:338 y:436
            OperatableStateMachine.add('go_to_table',
                                       NavigateToPoseState(timeout=120.0, navi_to_pos_topic='/navigate_to_pose', emergency_topic='/emergency_cmd', amcl_pose_topic='/amcl_pose', navi_progress_topic='/remaining_waypoint', arrived_table_topic='/arrived_table', table_num=0),
                                       transitions={'failed': 'failed', 'done': 'start_order_wait_state_2'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint', 'table_num': 'table_num'})

            # x:289 y:180
            OperatableStateMachine.add('start_order_wait_state',
                                       StartOrder_waitState(order_table_topic='/order_table', start_navi_topic='/start_nav', arrived_table_topic='/arrived_table'),
                                       transitions={'done': 'go_to_table'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'table_num': 'table_num'})

            # x:649 y:252
            OperatableStateMachine.add('start_order_wait_state_2',
                                       StartOrder_waitState(order_table_topic='/order_table', start_navi_topic='/start_nav', arrived_table_topic='/arrived_table'),
                                       transitions={'done': 'go_to_kitchen'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'table_num': 'go_home'})

            # x:304 y:32
            OperatableStateMachine.add('get_order_table_num_state',
                                       GetOrderTableNnumState(order_table_topic='/order_table'),
                                       transitions={'serving': 'start_order_wait_state', 'goback': 'start_order_wait_state_2'},
                                       autonomy={'serving': Autonomy.Off, 'goback': Autonomy.Off},
                                       remapping={'table_num': 'table_num'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
