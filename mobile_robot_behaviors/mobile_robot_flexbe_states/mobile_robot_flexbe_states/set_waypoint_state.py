#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

# msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# 버튼이 눌리면 현재 위치를 저장하는 state
class SetWaypointState(EventState):

    def __init__(self, amcl_pose_topic="/amcl_pose", navigator_topic="/navi_cmd"):
        # See example_state.py for basic explanations.
        super().__init__(output_keys=['waypoint'], outcomes=['done']) 

        
        self.amcl_pose_topic = amcl_pose_topic
        self.navigator_topic = navigator_topic
        
        self.amcl_pose_sub = ProxySubscriberCached({self.amcl_pose_topic : PoseWithCovarianceStamped})
        self.amcl_pose_sub.set_callback(self.amcl_pose_topic, self.amcl_pose_callback)
        self.sub_gui_command = ProxySubscriberCached({self.navigator_topic : String})
        self.sub_gui_command.set_callback(self.navigator_topic, self.gui_callback)
        self.set_waypint_msg = None
        self.current_pose = Pose()
        self.set_waypoint_flag = False
        self.waypoint_num = None    

        # 초기위치 저장변수 
        self.waypoint = [[0.0, 0.0, 0.0, 0.0],
                         [1.0, 1.0, 1.0, 1.0],
                         [2.0, 2.0, 2.0, 2.0],
                         [3.0, 3.0, 3.0, 3.0]] 


        self.control_dict = {
            'set_waypoint_0': lambda : self.set_waypoint(0),
            'set_waypoint_1': lambda : self.set_waypoint(1),
            'set_waypoint_2': lambda : self.set_waypoint(2),
            'set_waypoint_3': lambda : self.set_waypoint(3),
        }

    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def gui_callback(self, msg):
        self.set_waypint_msg = msg.data
        # 올바른 메세지가 들어왔을 때만 실행
        if self.set_waypint_msg in self.control_dict.keys():
            self.control_dict[self.set_waypint_msg]()
            self.info("set_waypoint_msg : {}".format(self.set_waypint_msg))
        else:
            self.info("set_waypoint_msg : {}".format(self.set_waypint_msg))
            self.info("wrong message")
            pass

    def set_waypoint(self, num):
        self.set_waypoint_flag = True
        self.waypoint_num = num
        self.waypoint[num][0] = self.current_pose.position.x
        self.waypoint[num][1] = self.current_pose.position.y
        self.waypoint[num][2] = self.current_pose.orientation.z
        self.waypoint[num][3] = self.current_pose.orientation.w
        self.info("set_waypoint_{} : {}".format(num, self.waypoint[num]))

    def execute(self, userdata):
        if self.set_waypoint_flag:
            self.set_waypoint_flag = False
            userdata.waypoint = self.waypoint
            # self.info("set_waypoint_state execute")
        return 'done'
    
    def on_enter(self, userdata):
        userdata.waypoint = self.waypoint
        self.info("set_waypoint_state enter")
        pass
        
    def on_exit(self, userdata):
        self.info("set_waypoint_state exit")
        pass

    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)