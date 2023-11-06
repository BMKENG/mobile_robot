import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from std_srvs.srv import SetBool
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from basic_navigator import BasicNavigator



# 리눅스에서 DISPLAY 환경 변수 설정 (이미 설정되어 있으면 덮어쓰지 않습니다)
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'

# UI파일 연결
# 단, UI파일은 Python 코드 파일과 같은 디렉토리에 위치해야한다.
# 
form_class = uic.loadUiType("waypoint.ui")[0]

# 화면을 띄우는데 사용되는 Class 선언
class MobileRobotGUI(QMainWindow, form_class, BasicNavigator):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.init_ui()

        # ROS initialization
        rclpy.init()
        self.node = rclpy.create_node('joystick_pyqt_node')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

        # amcl
        self.sub = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.current_pose = None

        # nav2
        self.nav2_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.navigator = BasicNavigator()

        
        self.goal_handle = None
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.node.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose',
                                                       self.amcl_callback,
                                                       amcl_pose_qos)
        self.initial_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)


        # init velocity
        self.set_linear_velocity = 0.5
        self.set_angular_velocity = 0.5
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # init waypoint
        # pose = [x, y, w, z]
        self.waypoint = [
                        [0.0, 0.0, 0.0, 0.0],
                        [1.0, 2.0, 0.0, 0.0],
                        [2.0, 2.0, 0.0, 0.0]
                        ]
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.send_cmd_vel)
        self.timer.timeout.connect(self.update_ros)
        self.timer.start(100)  # 10Hz

    def update_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)


    def amcl_callback(self, msg):
        self.current_pose = msg.pose.pose
        # print("current_pose: ", self.current_pose)


    def init_ui(self):

        # joy button
        self.go.clicked.connect(self.go_button)
        self.back.clicked.connect(self.back_button)
        self.stop.clicked.connect(self.stop_button)
        self.left.clicked.connect(self.left_button)
        self.right.clicked.connect(self.right_button)
        # vel up down button
        self.angular_up.clicked.connect(self.angular_up_button)
        self.angular_down.clicked.connect(self.angular_down_button)
        self.linear_up.clicked.connect(self.linear_up_button)
        self.linear_down.clicked.connect(self.linear_down_button)
        

        # set waypoint button
        self.set_waypoint_1.clicked.connect(self.set_waypoint_button_1)
        self.set_waypoint_2.clicked.connect(self.set_waypoint_button_2)
        self.set_waypoint_3.clicked.connect(self.set_waypoint_button_3)

        # go waypoint button
        self.go_waypoint_1.clicked.connect(self.go_waypoint_button_1)
        self.go_waypoint_2.clicked.connect(self.go_waypoint_button_2)
        self.go_waypoint_3.clicked.connect(self.go_waypoint_button_3)

        # # emergency stop button
        self.emergency.clicked.connect(self.emergency_stop_button)
        



    # joy button
    def go_button(self):
        self.linear_velocity = self.set_linear_velocity
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def back_button(self):
        self.linear_velocity = -self.set_linear_velocity
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def stop_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.send_cmd_vel()
        
    def left_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = self.set_angular_velocity
        self.send_cmd_vel()

    def right_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = -self.set_angular_velocity
        self.send_cmd_vel()

    def send_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.pub.publish(msg)
        self.show()


    # vel up down button
    def angular_up_button(self):
        self.set_angular_velocity += 0.1
        self.stop_button()
        if self.set_angular_velocity >= 1.0:
            self.set_angular_velocity = 1.0

        self.statusBar().showMessage('set angular velocity: {}'.format(self.set_angular_velocity))


    def angular_down_button(self):
        self.set_angular_velocity -= 0.1
        self.stop_button()
        if self.set_angular_velocity <= 0.1:
            self.set_angular_velocity = 0.1
        self.statusBar().showMessage('set angular velocity: {}'.format(self.set_angular_velocity))
    def linear_up_button(self):
        self.set_linear_velocity += 0.1
        self.stop_button()
        if self.set_linear_velocity >= 1.0:
            self.set_linear_velocity = 1.0
        self.statusBar().showMessage('set linear velocity: {}'.format(self.set_linear_velocity))
    def linear_down_button(self):
        self.set_linear_velocity -= 0.1
        self.stop_button()
        if self.set_linear_velocity <= 0.1:
            self.set_linear_velocity = 0.1
        self.statusBar().showMessage('set linear velocity: {}'.format(self.set_linear_velocity))

    # set waypoint button
    def set_waypoint(self, waypoint_num):
        if self.current_pose is not None:
            self.waypoint[waypoint_num][0] = self.current_pose.position.x
            self.waypoint[waypoint_num][1] = self.current_pose.position.y
            self.waypoint[waypoint_num][2] = self.current_pose.orientation.w
            self.waypoint[waypoint_num][3] = self.current_pose.orientation.z
            # print("waypoint_{}: ".format(waypoint_num+1), self.waypoint[waypoint_num])
            self.statusBar().showMessage('waypoint_{}: {}'.format(waypoint_num+1, self.waypoint[waypoint_num]))

        else:
            # print("아직 유효한 waypoint가 저장되지 않았습니다.")
            self.statusBar().showMessage('아직 유효한 waypoint가 저장되지 않았습니다.')
    
    def set_waypoint_button_1(self):
        self.set_waypoint(0)
    
    def set_waypoint_button_2(self):
        self.set_waypoint(1)
    
    def set_waypoint_button_3(self):
        self.set_waypoint(2)
        
    # go waypoint button
    def go_waypoint(self, waypoint_num):
        self.goal_handle = None
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.waypoint[waypoint_num][0]
        goal_msg.pose.pose.position.y = self.waypoint[waypoint_num][1]
        goal_msg.pose.pose.orientation.w = self.waypoint[waypoint_num][2]
        goal_msg.pose.pose.orientation.z = self.waypoint[waypoint_num][3]
        self.nav2_client.wait_for_server()
        # Assuming this is in your go_waypoint function or wherever you're sending the goal
        future_goal_handle = self.nav2_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self.node, future_goal_handle)

        # Now extract the actual GoalHandle
        self.goal_handle = future_goal_handle.result()
        # self.goal_handle = self.nav2_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self.node, self.goal_handle)

        # while self.goal_handle == None:
        #     self.statusBar().showMessage('go waypoint_{} failed!'.format(waypoint_num+1))

        # self.statusBar().showMessage('go waypoint_{} success!'.format(waypoint_num+1))





        
    def emergency_stop_button(self):
        if self.nav2_client and self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.statusBar().showMessage('Emergency stop: Goal cancelled!')
    
        self.stop_button()


    def go_waypoint_button_1(self):
        self.go_waypoint(0)
    
    def go_waypoint_button_2(self):
        self.go_waypoint(1)
    
    def go_waypoint_button_3(self):
        self.go_waypoint(2)



    def closeEvent(self):
        rclpy.shutdown()






    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goThroughPoses(self, poses):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return


if __name__ == "__main__":
    # QApplication : 프로그램을 실행시켜주는 클래스
    app = QApplication(sys.argv)

    # WindowClass의 인스턴스 생성
    myWindow = MobileRobotGUI()

    # 프로그램 화면을 보여주는 코드
    myWindow.show()

    # 프로그램을 이벤트루프로 진입시키는(프로그램을 작동시키는) 코드
    app.exec_()
