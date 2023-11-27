#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

# navigate to pose 
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

class InitPoseState(EventState):

    def __init__(self, init_pose_topic="/initialpose", init_pose_cmd_topic="/init_pose_cmd"):

        super().__init__(outcomes=['done'])
        self.init_pose_topic = init_pose_topic
        self.init_pose_cmd_topic = init_pose_cmd_topic

        self.init_pose_pub = ProxyPublisher({self.init_pose_topic: PoseWithCovarianceStamped})
        self.init_pose = PoseWithCovarianceStamped()

        self.init_pose_cmd_sub = ProxySubscriberCached({self.init_pose_cmd_topic: String})
        
        self.msg = None

        self._sucess = False

    def start(self):
        self.msg = self.init_pose_cmd_sub.get_last_msg(self.init_pose_cmd_topic)
        if self.msg is None:
            return False
        
        if self.msg.data == "start_nav":
            self.init_pose.header.frame_id = "map"
            self.init_pose.pose.pose.orientation.w = 1.0
            self.init_pose_pub.publish(self.init_pose_topic, self.init_pose)
            self.init_pose_cmd_sub.remove_last_msg(self.init_pose_cmd_topic)
            self.info("init_pose update")
            return True
        else:
            return False


    def execute(self, userdata):
        self._sucess = self.start()
        if self._sucess:
            # self.info("navigate to pose sucess")
            return 'done'

        
    def on_enter(self, userdata):
        self.info("navigation initial pose enter")
        # self.goToPose(self.table_num)

    def on_exit(self, userdata):
        self.info("navigation initial pose exit")

    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)