
import rclpy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32
from mobile_robot_interfaces.msg import NaviProgress


from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread

class ROSNode(QThread):

    def __init__(self):
        super().__init__()
        self._is_running = True
        self.table_num = None
        self.remaining_waypoint = None

    def run(self):
        self.node = rclpy.create_node('mobile_robot_gui_behavior_node')
        
        self.sub_cmd_vel = self.node.create_subscription(Twist, 'set_vel', self.cmd_vel_callback, 10)
        self.set_vel = None

        self.sub_pose = self.node.create_subscription(PoseWithCovarianceStamped, 
        'amcl_pose', self.pose_callback, 10)
        self.current_pose = None

        # 진행률 표시
        self.sub_remaining_waypoint = self.node.create_subscription(NaviProgress,
         'remaining_waypoint', self.remaining_waypoint_callback, 10)



        self.table_num = None
        self.remaining_waypoint = None
        self.pub_joystick = self.node.create_publisher(String, 'joy_cmd', 10)
        self.pub_navigator = self.node.create_publisher(String, 'navi_cmd', 10)
        self.pub_emergency = self.node.create_publisher(String, 'emergency_cmd', 10)
        self.pub_init_pose = self.node.create_publisher(String, 'init_pose_cmd', 10)
        
        # 로봇 출발을 위한 명령
        self.pub_start_nav = self.node.create_publisher(Int32, 'start_nav', 10)
        # 주문 받은 table number
        self.sub_table_num = self.node.create_subscription(Int32, 'order_table',
         self.table_num_callback, 10)


        while rclpy.ok() and self._is_running:
            rclpy.spin_once(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

    def terminate(self):
        self._is_running = False


    def cmd_vel_callback(self, msg):
        self.set_vel = msg

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def remaining_waypoint_callback(self, msg):
        self.remaining_waypoint = msg.remaining_waypoint
        # print(self.remaining_waypoint)
    
    def table_num_callback(self, msg):
        self.table_num = msg.data
