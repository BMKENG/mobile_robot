#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

# 메시지
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# 버튼이 눌리면 현재 위치를 저장하는 state
class SetWaypointState(EventState):

    def __init__(self, amcl_pose_topic="/amcl_pose", navigator_topic="/navi_cmd"):
        # 기본 설명 참고
        super().__init__(output_keys=['waypoint'], outcomes=['done']) 

        # 토픽 설정
        self.amcl_pose_topic = amcl_pose_topic
        self.navigator_topic = navigator_topic
        
        # 현재 위치 구독자 및 콜백 함수 설정
        self.amcl_pose_sub = ProxySubscriberCached({self.amcl_pose_topic : PoseWithCovarianceStamped})
        self.amcl_pose_sub.set_callback(self.amcl_pose_topic, self.amcl_pose_callback)
        # GUI 커맨드 구독자 및 콜백 함수 설정
        self.sub_gui_command = ProxySubscriberCached({self.navigator_topic : String})
        self.sub_gui_command.set_callback(self.navigator_topic, self.gui_callback)
        # 변수 초기화
        self.set_waypint_msg = None
        self.current_pose = Pose()
        self.set_waypoint_flag = False
        self.waypoint_num = None    

        # 초기 위치 저장 변수
        self.waypoint = [[0.0, -1.0, 1.0, 1.0],
                         [2.0, -1.0, 1.0, 1.0],
                         [-1.5, 2.5, 1.0, 1.0],
                         [1.9, 3.0, 1.0, 1.0]] 

        # 컨트롤 사전 정의
        self.control_dict = {
            'set_waypoint_0': lambda : self.set_waypoint(0),
            'set_waypoint_1': lambda : self.set_waypoint(1),
            'set_waypoint_2': lambda : self.set_waypoint(2),
            'set_waypoint_3': lambda : self.set_waypoint(3),
        }

    # 현재 위치 콜백 함수
    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    # GUI 커맨드 콜백 함수
    def gui_callback(self, msg):
        self.set_waypint_msg = msg.data
        # 유효한 메시지인 경우 실행
        if self.set_waypint_msg in self.control_dict.keys():
            self.control_dict[self.set_waypint_msg]()
            self.info("set_waypoint_msg : {}".format(self.set_waypint_msg))
        else:
            self.info("set_waypoint_msg : {}".format(self.set_waypint_msg))
            self.info("잘못된 메시지")
            pass

    # 웨이포인트 설정 함수
    def set_waypoint(self, num):
        self.set_waypoint_flag = True
        self.waypoint_num = num
        # 현재 위치 저장
        self.waypoint[num][0] = self.current_pose.position.x
        self.waypoint[num][1] = self.current_pose.position.y
        self.waypoint[num][2] = self.current_pose.orientation.z
        self.waypoint[num][3] = self.current_pose.orientation.w
        self.info("set_waypoint_{} : {}".format(num, self.waypoint[num]))

    def execute(self, userdata):
        if self.set_waypoint_flag:
            self.set_waypoint_flag = False
            userdata.waypoint = self.waypoint
            # 웨이포인트 설정 완료
        return 'done'
    
    def on_enter(self, userdata):
        # 웨이포인트 상태 진입
        userdata.waypoint = self.waypoint
        self.info("set_waypoint_state enter")
        pass
        
    def on_exit(self, userdata):
        # 웨이포인트 상태 종료
        self.info("set_waypoint_state exit")
        pass

    # 로그 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)
