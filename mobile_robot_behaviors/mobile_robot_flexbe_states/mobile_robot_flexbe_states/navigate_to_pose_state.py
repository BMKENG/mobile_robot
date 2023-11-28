#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

# navigate to pose 
from nav2_msgs.action import NavigateToPose
from mobile_robot_interfaces.msg import NaviProgress
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32

class NavigateToPoseState(EventState):

    def __init__(self, timeout=120.0, navi_to_pos_topic="/navigate_to_pose", emergency_topic="/emergency_cmd",
                  amcl_pose_topic="/amcl_pose", navi_progress_topic="/remaining_waypoint", arrived_table_topic="/arrived_table", table_num=0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['failed', 'done'],
                         input_keys=['waypoint', 'table_num']) 

        # timer
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._start_time = None
        
        self.navi_to_pos_topic = navi_to_pos_topic
        self.amcl_pose_topic = amcl_pose_topic
        self.navi_progress_topic = navi_progress_topic
        self.emergency_topic = emergency_topic
        self.arrived_table_topic = arrived_table_topic

        self.table_num = table_num

    
        ProxyActionClient.initialize(NavigateToPoseState._node)

        # navigate to pose action client
        self._client = ProxyActionClient({self.navi_to_pos_topic: NavigateToPose},
                                         wait_duration=0.0)  # pass required clients as dict (topic: type)

        # sub
        self.model_pose_sub = ProxySubscriberCached({self.amcl_pose_topic : PoseWithCovarianceStamped})
        # self.model_pose_sub.set_callback(self.amcl_pose_topic, self.getFeedback)
        self.current_pose = None

        self.sub_navigator = ProxySubscriberCached({self.emergency_topic: String})
        self.sub_navigator.set_callback(self.emergency_topic, self.emergency_callback)

        # pub
        self.remaining_waypoint_pub = ProxyPublisher({self.navi_progress_topic: NaviProgress})
        self.naviprog = NaviProgress()

        self.naviprog.remaining_waypoint = [100, 100, 100, 100]

        self.pub_arrived_table = ProxyPublisher({self.arrived_table_topic: Int32})
        self.arrived_table = Int32()


        # 초기위치 저장변수 
        self.initial_pose = None
        self.feedback_flag = False
        self.remaining_distance = 0
        self.initial_distance = 0
        self.feedback = None

        # 목표 좌표 저장 변수
        self.waypoint = None

        # It may happen that the action client fails to send the action goal.
        self._sucess = False
        self._failed = False

        self.control_dict = {
            "emergency" : lambda : self.emergency()
        }


    def emergency_callback(self, msg):
        # 해당 명령어가 딕셔너리에 있으면 해당 함수 실행
        if msg.data in self.control_dict.keys():
            self.control_dict[msg.data]()
            self.info(f"emergency: {msg.data}")
        else:
            self.info("emergency: wrong command")

    def emergency(self):
        self.navi_cancel()
        self.info("emergency!!!")
        self._failed = True


    def goToPose(self, waypoint_num):
        # self.navi_cancel()
        self.table_num = waypoint_num
        self.info(f"table_num: {self.table_num}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.waypoint[waypoint_num][0]
        goal_msg.pose.pose.position.y = self.waypoint[waypoint_num][1]
        goal_msg.pose.pose.orientation.z = self.waypoint[waypoint_num][2]
        goal_msg.pose.pose.orientation.w = self.waypoint[waypoint_num][3]

        self.goal_handle = self._client.send_goal(self.navi_to_pos_topic, goal_msg)
             
        self.info("Sending goal to navigate to pose")


    def getFeedback(self):
        self.current_pose = self.model_pose_sub.get_last_msg(self.amcl_pose_topic).pose.pose
        if self._client.has_feedback(self.navi_to_pos_topic) and self.current_pose is not None:
            feedback_msg = self._client.get_feedback(self.navi_to_pos_topic)
            if feedback_msg and feedback_msg.feedback and hasattr(feedback_msg.feedback, 'distance_remaining'):
                distance_remaining = feedback_msg.feedback.distance_remaining
                
                if self.feedback_flag == False:
                    self.info(f'현재 좌표: {self.current_pose.position.x}, {self.current_pose.position.y}')
                    # 여기서 self.waypoint[0][0]과 self.waypoint[0][1]은 첫 번째 웨이포인트의 x, y 좌표를 의미합니다.
                    self.initial_distance = ((self.current_pose.position.x - self.waypoint[self.table_num][0]) ** 2 + 
                                            (self.current_pose.position.y - self.waypoint[self.table_num][1]) ** 2) ** 0.5
                    self.feedback_flag = True

                # 여기서는 int로 변환하지 않고, float 값을 유지합니다.
                self.remaining_distance = (distance_remaining / self.initial_distance * 100)
                
                if self.remaining_distance < 0:
                    self.remaining_distance = 0
                elif self.remaining_distance > 100:
                    self.remaining_distance = 100 

                # 100에서 빼서 남은 거리의 백분율을 계산합니다.
                self.remaining_distance = 100 - self.remaining_distance
                self.naviprog.remaining_waypoint[self.table_num] = self.remaining_distance
                # self.info(f'진행률: {self.remaining_distance}')
                self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
            else:
                self.warn("Feedback received but doesn't contain 'distance_remaining'")
        else:
            self.warn("No feedback received from action server")


    def navi_cancel(self):
        if not self._client.has_result(self.navi_to_pos_topic):
            self._client.cancel(self.navi_to_pos_topic)
            self.info("Canceling active goal on exit")
            self.naviprog.remaining_waypoint[self.table_num] = 0
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
        else:
            # self.info("navigate to pose finished")
            self.naviprog.remaining_waypoint[self.table_num] = 100
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
            self.info("navigate complete")


    def execute(self, userdata):
        self.getFeedback()
        if self._client.has_result(self.navi_to_pos_topic):
            self.info("navigate to pose finished")
            self._sucess = True

        # 만약 시간안에 목표를 달성하지 못하면 failed 60초
        # if self._start_time + self._timeout < self._node.get_clock().now():
        #     self._failed = True

        
    
        if self._sucess:
            # self.info("navigate to pose sucess")
            return 'done'
        if self._failed:
            # self.err("navigate to pose failed")
            return 'failed'
        
    def on_enter(self, userdata):
        # goal send navugate to pose
        self.table_num = userdata.table_num
        self.waypoint = userdata.waypoint

        self._start_time = self._node.get_clock().now()
        self._sucess = False
        self._failed = False
        self.feedback_flag = False
        self.remaining_distance = 0
        self.initial_distance = 0

        self.info("navigate to pose enter")
        self.goToPose(self.table_num)

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self.navi_to_pos_topic):
            self._client.cancel(self.navi_to_pos_topic)
            self.info("Canceling active goal on exit")
            self.naviprog.remaining_waypoint[self.table_num] = 0
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
        else:
            # self.info("navigate to pose finished")
            self.naviprog.remaining_waypoint[self.table_num] = 100
            self.remaining_waypoint_pub.publish(self.navi_progress_topic, self.naviprog)
            self.info("navigate complete")

        # if self.table_num == 0:
        #     self.arrived_table.data = 0
        #     self.pub_arrived_table.publish(self.arrived_table_topic, self.arrived_table)



    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)