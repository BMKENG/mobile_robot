import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer

import rclpy
from geometry_msgs.msg import Twist

# 리눅스에서 DISPLAY 환경 변수 설정 (이미 설정되어 있으면 덮어쓰지 않습니다)
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'

# UI파일 연결
# 단, UI파일은 Python 코드 파일과 같은 디렉토리에 위치해야한다.
# 
form_class = uic.loadUiType("joy.ui")[0]

# 화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.init_ui()

        # ROS initialization
        rclpy.init()
        self.node = rclpy.create_node('joystick_pyqt_node')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.send_cmd_vel)
        # self.timer.start(100)  # 10Hz


    def init_ui(self):

        self.pushButton_go.clicked.connect(self.go_button)
        self.pushButton_back.clicked.connect(self.back_button)
        self.pushButton_stop.clicked.connect(self.stop_button)
        self.pushButton_left.clicked.connect(self.left_button)
        self.pushButton_right.clicked.connect(self.right_button)


        # layout = QVBoxLayout()
        # central_widget = QWidget(self)
        # central_widget.setLayout(layout)

        # self.setCentralWidget(central_widget)

    def go_button(self):
        self.linear_velocity = 0.5
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def back_button(self):
        self.linear_velocity = -0.5
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def stop_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.send_cmd_vel()
        
    def left_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = 0.5
        self.send_cmd_vel()

    def right_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = -0.5
        self.send_cmd_vel()

    def send_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.pub.publish(msg)

    def closeEvent(self, event):
        rclpy.shutdown()

if __name__ == "__main__":
    # QApplication : 프로그램을 실행시켜주는 클래스
    app = QApplication(sys.argv)

    # WindowClass의 인스턴스 생성
    myWindow = WindowClass()

    # 프로그램 화면을 보여주는 코드
    myWindow.show()

    # 프로그램을 이벤트루프로 진입시키는(프로그램을 작동시키는) 코드
    app.exec_()
