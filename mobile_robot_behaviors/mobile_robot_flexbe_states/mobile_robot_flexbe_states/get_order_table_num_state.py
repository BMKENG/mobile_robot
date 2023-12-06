#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

# 메시지
from std_msgs.msg import Int32

# 주문입력시 테이블 번호를 저장하고 대기하는 state
class GetOrderTableNnumState(EventState):
    def __init__(self, order_table_topic="/order_table"):
        # 기본 설명 참고
        super().__init__(outcomes=['serving', 'goback'], output_keys=['table_num']) 
        self.info("GetOrderTableNnumState 초기화")

        self.order_table_topic = order_table_topic
        # 주문 테이블 구독
        # 주문 입력 받기 - 몇 번 테이블인지
        self.sub_order_table = ProxySubscriberCached({self.order_table_topic : Int32}) 
        self.sub_order_table.set_callback(self.order_table_topic, self.order_table_callback)
        self.order_table_msg = None
        self._sucess = False
        self._serving = False
        self._goback = False

    def order_table_callback(self, msg):
        self.order_table_msg = msg

    def order_table(self):
        if self.order_table_msg is not None:
            if self.order_table_msg.data == 0:
                self._goback = True
                return True
            self.order_table_msg = self.order_table_msg
            self.info("GetOrderTableNnumState 서빙")
            self.sub_order_table.remove_last_msg(self.order_table_topic)
            self._serving = True
            return True
        else:
            return False

    def execute(self, userdata):
        self._sucess = self.order_table()
        if self._sucess == True:
            if self._goback == True:
                return 'goback'
            userdata.table_num = self.order_table_msg.data
            self.info("테이블 번호 : {}".format(userdata.table_num))
            return 'serving'
        else:
            pass
  
    def on_enter(self, userdata):
        self._sucess = False
        self._serving = False
        self._goback = False
        
        self.order_table_msg= None
        self.info("GetOrderTableNnumState 진입")
        self.serving_flag = False
        self.return_flag = False
        pass
        
    def on_exit(self, userdata):
        self.info("GetOrderTableNnumState 종료")
        pass

    # 로그 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)
