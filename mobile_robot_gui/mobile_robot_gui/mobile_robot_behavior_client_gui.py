import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer

import rclpy
from std_msgs.msg import Int32
from std_msgs.msg import String
from mobile_robot_gui.qt_to_ros_behavior_client import ROSNode
from mobile_robot_gui.mobile_robot_sub_gui import SubWindow


# UI 파일 경로 확장
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'
# UI 파일 경로 확장
ui_file_path = os.path.expanduser("~/ros2_ws/src/mobile_robot/mobile_robot_gui/ui/client.ui")

# UI 파일 연결
form_class = uic.loadUiType(ui_file_path)[0]

class MobileRobotClientGUI(QMainWindow, form_class):
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
        self.table_1.clicked.connect(self.table_1_button)
        self.table_2.clicked.connect(self.table_2_button)
        self.table_3.clicked.connect(self.table_3_button)

        # timer
        self.timer = QTimer(self)
        self.timer.start(100)

            
        self.timer.timeout.connect(self.table_num_update)
        



    # ====================================================== #
    def table_order_command(self, table_number):
        self.statusBar().showMessage('table number: {}'.format(table_number))
        print("table_order_command: ", table_number)
        msg = Int32()
        msg.data = table_number
        self.ros_node.pub_order_table.publish(msg)

    def table_1_button(self):
        # self.statusBar().showMessage('table number: {}'.format(1))
        self.table_order_command(1)
    def table_2_button(self):
        # self.statusBar().showMessage('table number: {}'.format(2))
        self.table_order_command(2)
    def table_3_button(self):
        # self.statusBar().showMessage('table number: {}'.format(3))
        self.table_order_command(3)

    # 주문 이벤트가 발생하면 테이블 번호를 받아옴
    def table_num_update(self):
        if self.ros_node.arrived_table is not None:
            self.statusBar().showMessage('table number: {}'.format(self.ros_node.arrived_table))
            self.table_num_sub_window(self.ros_node.arrived_table)

            self.ros_node.arrived_table = None 
    def table_num_sub_window(self, table_num):
        self.sub_window.table_num = table_num
        # 만약 서브 윈도우가 열려있지 않다면
        if self.sub_window.isVisible() == False:
            self.sub_window.show()
        else:
            pass


    def closeEvent(self, event):
        rclpy.shutdown()



def main(args=None):
    app = QApplication(sys.argv)
    executor = MobileRobotClientGUI()
    executor.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
