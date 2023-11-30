import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer

import rclpy
from std_msgs.msg import String
from std_msgs.msg import Int32
from mobile_robot_gui.qt_to_ros_behavior import ROSNode
from mobile_robot_gui.mobile_robot_sub_gui import SubWindow


# UI 파일 경로 확장
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'
# UI 파일 경로 확장
ui_file_path = os.path.expanduser("~/ros2_ws/src/mobile_robot/mobile_robot_gui/ui/mobile_robot_behavior.ui")

# UI 파일 연결
form_class = uic.loadUiType(ui_file_path)[0]

class MobileRobotGUI(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.init_ui()
        # init ros
        rclpy.init()
        self.ros_node = ROSNode()
        # sub window init
        self.sub_window = SubWindow(self.ros_node)
        self.ros_node.start()


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
        self.set_waypoint_0.clicked.connect(self.set_waypoint_button_0)
        self.set_waypoint_1.clicked.connect(self.set_waypoint_button_1)
        self.set_waypoint_2.clicked.connect(self.set_waypoint_button_2)
        self.set_waypoint_3.clicked.connect(self.set_waypoint_button_3)

        # go waypoint button
        self.go_waypoint_0.clicked.connect(self.go_waypoint_button_0)
        self.go_waypoint_1.clicked.connect(self.go_waypoint_button_1)
        self.go_waypoint_2.clicked.connect(self.go_waypoint_button_2)
        self.go_waypoint_3.clicked.connect(self.go_waypoint_button_3)
        
        # go waypoint progress bar
        self.progressbar_0.setValue(0)
        self.progressbar_1.setValue(0)
        self.progressbar_2.setValue(0)
        self.progressbar_3.setValue(0)


        # emergency start button
        self.start.clicked.connect(self.start_button)
        self.emergency.clicked.connect(self.emergency_button)

        # timer
        self.timer = QTimer(self)
        self.timer.start(100)
        
        self.timer.timeout.connect(self.progressbar_update)
        self.timer.timeout.connect(self.table_num_update)
        


    # joy button
    # ====================================================== #
    def joy_cammnd(self, msg):
        self.msg = String()
        self.msg.data = msg
        self.ros_node.pub_joystick.publish(self.msg)

    def go_button(self):
        self.joy_cammnd('go')
    def back_button(self):
        self.joy_cammnd('back')
    def stop_button(self):
        self.joy_cammnd('stop')
    def left_button(self):
        self.joy_cammnd('left')
    def right_button(self):
        self.joy_cammnd('right')

    # vel up down button
    def angular_up_button(self):
        self.joy_cammnd('angular_up')
        self.statusBar().showMessage('set angular velocity: {}'.format(self.ros_node.set_vel.angular.z))

    def angular_down_button(self):
        self.joy_cammnd('angular_down')
        self.statusBar().showMessage('set angular velocity: {}'.format(self.ros_node.set_vel.angular.z))

    def linear_up_button(self):
        self.joy_cammnd('linear_up')
        self.statusBar().showMessage('set linear velocity: {}'.format(self.ros_node.set_vel.linear.x))

    def linear_down_button(self):
        self.joy_cammnd('linear_down')
        self.statusBar().showMessage('set linear velocity: {}'.format(self.ros_node.set_vel.linear.x))
    # ====================================================== #


    # waypoint button
    # ====================================================== #
    def navi_command(self, msg):
        self.msg = String()
        self.msg.data = msg
        self.ros_node.pub_navigator.publish(self.msg)

    def set_waypoint_button_0(self):
        self.navi_command('set_waypoint_0')
        self.statusBar().showMessage('set waypoint 0 : x: {}, y: {}, z: {}'.format(self.ros_node.current_pose.position.x, self.ros_node.current_pose.position.y, self.ros_node.current_pose.position.z))

    def set_waypoint_button_1(self):
        self.navi_command('set_waypoint_1')
        self.statusBar().showMessage('set waypoint 1 : x: {}, y: {}, z: {}'.format(self.ros_node.current_pose.position.x, self.ros_node.current_pose.position.y, self.ros_node.current_pose.position.z))

    def set_waypoint_button_2(self):
        self.navi_command('set_waypoint_2')
        self.statusBar().showMessage('set waypoint 2 : x: {}, y: {}, z: {}'.format(self.ros_node.current_pose.position.x, self.ros_node.current_pose.position.y, self.ros_node.current_pose.position.z))

    def set_waypoint_button_3(self):
        self.navi_command('set_waypoint_3')
        self.statusBar().showMessage('set waypoint 3 : x: {}, y: {}, z: {}'.format(self.ros_node.current_pose.position.x, self.ros_node.current_pose.position.y, self.ros_node.current_pose.position.z))

    def start_button(self):
        self.msg = String()
        self.msg.data = 'start_nav'
        self.ros_node.pub_init_pose.publish(self.msg)
        self.statusBar().showMessage('start navigation')


    def emergency_button(self):
        self.msg = String()
        self.msg.data = 'emergency'
        self.ros_node.pub_emergency.publish(self.msg)
        self.statusBar().showMessage('emergency!!!')
        




    def progressbar_update(self):
        if self.ros_node.remaining_waypoint is not None:
            self.progressbar_0.setValue(self.ros_node.remaining_waypoint[0])
            self.progressbar_1.setValue(self.ros_node.remaining_waypoint[1])
            self.progressbar_2.setValue(self.ros_node.remaining_waypoint[2])
            self.progressbar_3.setValue(self.ros_node.remaining_waypoint[3])
            # self.ros_node.remaining_waypoint = None
        else:
            # feedback이 들어오지 않거나 종료된 경우 프로그래스 바를 100%로 설정
            self.progressbar_0.setValue(100)
            self.progressbar_1.setValue(100)
            self.progressbar_2.setValue(100)
            self.progressbar_3.setValue(100)


    def go_waypoint_button_0(self):
        self.navi_command('go_to_pose_0') 
        int_msg = Int32()
        int_msg.data = 0
        self.ros_node.pub_order_table.publish(int_msg)
        
        self.statusBar().showMessage('go waypoint 0')

    def go_waypoint_button_1(self):
        self.navi_command('go_to_pose_1')
        int_msg = Int32()
        int_msg.data = 1
        self.ros_node.pub_order_table.publish(int_msg)
        
      
        self.statusBar().showMessage('go waypoint 1')

    def go_waypoint_button_2(self):
        self.navi_command('go_to_pose_2')
        int_msg = Int32()
        int_msg.data = 2
        self.ros_node.pub_order_table.publish(int_msg)
      
        self.statusBar().showMessage('go waypoint 2')

    def go_waypoint_button_3(self):
        self.navi_command('go_to_pose_3')
        # self.ros_node.table_num = 3
        int_msg = Int32()
        int_msg.data = 3
        self.ros_node.pub_order_table.publish(int_msg)
      
        self.statusBar().showMessage('go waypoint 3')
    # ====================================================== #


    # 주문 이벤트가 발생하면 테이블 번호를 받아옴
    def table_num_update(self):
        if self.ros_node.table_num is not None and self.ros_node.table_num != 0:
            self.statusBar().showMessage('table number: {}'.format(self.ros_node.table_num))
            self.table_num_sub_window(self.ros_node.table_num)

            self.ros_node.table_num = None 
    def table_num_sub_window(self, table_num):
            self.sub_window.table_num = table_num
            self.sub_window.show()


    def closeEvent(self, event):
        rclpy.shutdown()



def main(args=None):
    app = QApplication(sys.argv)
    executor = MobileRobotGUI()
    executor.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
