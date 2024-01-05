
import math 
import re
import serial
import rclpy as rp
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


from time import sleep
# 멀티 스레드
from rclpy.executors import MultiThreadedExecutor

# 1의 보수를 구하는 함수
def ones_complement(number, bit_length):
    complement = ~number & ((1 << bit_length) - 1)
    return complement


class RPMOdom(Node):
    def __init__(self):
        super().__init__('mobile_robot_serial')
        # 오돔 값을 발행하기 위한 publisher
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.serial_port = '/dev/ttyACM0'  # 시리얼 포트 경로
        self.baud_rate = 115200  # 시리얼 통신 속도

        self.wheel_radius = 0.0379  # 바퀴 반지름 (단위: m/)
        self.wheel_base = 0.15477  # 바퀴 베이스 (단위: m)/

        # 초기 위치와 방향
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0

        # 타이머 설정
        self.timer_period = 0.1  # 주기 (단위: 초)
        self.timer = self.create_timer(self.timer_period, self.read_rpm)
        self.ser = serial.Serial(self.serial_port, self.baud_rate)



            
    def read_rpm(self):
        try:
            # 시리얼 포트로부터 rpm 값을 읽어옴

            received_data = self.ser.readline().decode().strip()

            # 데이터 길이 검증 및 정상 데이터인 경우
            if re.match(r'^-?\d+\.\d+, -?\d+\.\d+$', received_data):

                split_data = received_data.split(',')
                if len(split_data) != 2:
                    raise ValueError("Invalid data format")

                # '-' 기호가 여러 개 있는 경우 처리
                data1 = split_data[0].replace('-', '') if split_data[0].count('-') > 1 else split_data[0]
                data2 = split_data[1].replace('-', '') if split_data[1].count('-') > 1 else split_data[1]
                # print(data1, data2)
                # rp.loginfo("data1: %s, data2: %s", data1, data2)
                # 문자열을 부동 소수점으로 변환
                left_rpm = float(data1) / 600.0
                right_rpm = float(data2) / 600.0
                # print(left_rpm, right_rpm)
                # 좌측, 우측 바퀴의 선속도 계산
                left_wheel_vel = (2 * math.pi * self.wheel_radius * left_rpm) 
                right_wheel_vel = (2 * math.pi * self.wheel_radius * right_rpm)
                # print(left_wheel_vel, right_wheel_vel)
                # 이동 거리 및   방향 갱신
                distance = (left_wheel_vel + right_wheel_vel) / 2.0
                self.orientation_z += (right_wheel_vel - left_wheel_vel) / self.wheel_base
                print(distance, self.orientation_z)
                # 위치 업데이트
                self.position_x += distance * math.cos(self.orientation_z)
                self.position_y += distance * math.sin(self.orientation_z)

                # 오도메트리 메시지 생성 및 발행
                odometry = Odometry()
                odometry.header = Header()
                odometry.header.stamp = self.get_clock().now().to_msg()
                odometry.header.frame_id = 'odom'
                odometry.child_frame_id = 'base_link'
                odometry.pose.pose.position.x = self.position_x
                odometry.pose.pose.position.y = self.position_y
                odometry.pose.pose.orientation.z = math.sin(self.orientation_z / 2.0)
                odometry.pose.pose.orientation.w = math.cos(self.orientation_z / 2.0)
                self.publisher.publish(odometry)
    
        except serial.SerialException as e:
            print(f"SerialException: {e}")
            self.ser.close()
            sleep(0.1)  # 재연결 전에 잠시 대기
            self.ser.open()



class cmd_vel_2_rpm(Node):
    def __init__(self):
        super().__init__('cmd_vel_2_rpm')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.serial_port = '/dev/ttyACM0'  # 시리얼 포트 경로
        self.baud_rate = 115200  # 시리얼 통신 속도
        self.subscription  # prevent unused variable warning
        self.wheel_radius = 0.0379  # 바퀴 반지름 (단위: m/)
        self.wheel_base = 0.15477  # 바퀴 베이스 (단위: m)/


    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x  # 선속도(m/s)
        angular_vel = msg.angular.z  # 각속도(rad/s)

        # 좌측, 우측 바퀴의 선속도 계산
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_base / 2) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_base / 2) / self.wheel_radius

        # 좌측 바퀴 RPM 값을 발행
        left_wheel_rpm = (left_wheel_vel * 60 / (2 * 3.14159))
        # 우측 바퀴 RPM 값을 발행
        right_wheel_rpm = (right_wheel_vel * 60 / (2 * 3.14159))
        ser = serial.Serial(self.serial_port, self.baud_rate)
        # 시리얼 포트로 RPM 값을 전송
        send_data = str(int(left_wheel_rpm)) + ',' + str(int(right_wheel_rpm)) + '\n'
        ser.write(send_data.encode())



def main(args=None):
    rp.init(args=args)
    odom_pub = RPMOdom()
    cmd_vel_sub= cmd_vel_2_rpm()

    executor = MultiThreadedExecutor()
    executor.add_node(odom_pub)
    executor.add_node(cmd_vel_sub)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        odom_pub.destroy_node()
        cmd_vel_sub.destroy_node()
        rp.shutdown()
if __name__ == '__main__':
    main()