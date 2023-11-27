#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

# msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32

# 주문입력시 테이블 번호를 저장하고 대기하는 state
class GetGoBackState(EventState):
    def __init__(self, order_table_topic="/order_table"):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done'], output_keys=['table_num']) 
        self.info("GetOrderTableNnumState init")

        self.order_table_topic = order_table_topic
        # sub order table
        # 주문을 입력 받습니다 - 몇번 테이블인지 
        self.sub_order_table = ProxySubscriberCached({self.order_table_topic : Int32}) 
        self.sub_order_table.set_callback(self.order_table_topic, self.order_table_callback)
        self.order_table_msg = None
        self.order_table_flag = False

    def order_table_callback(self, msg):
        self.order_table_msg = msg.data
        if self.order_table_msg != 0:
            self.info("주문이 들어왔습니다. 테이블 번호 : {}".format(self.order_table_msg))
            self.info("서빙 대기중...")
            self.order_table_flag = True
        else:
            self.info("주방으로 복귀합니다.")
            self.info("복귀 대기중...")
            self.order_table_flag = True



    def execute(self, userdata):
        if self.order_table_flag == True:
            userdata.table_num = self.order_table_msg
            self.order_table_flag = False
            self.start_navi_flag = False
            return 'done'
         
        
    def on_enter(self, userdata):
        self.info("GetOrderTableNnumState enter")
        pass
        
    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.
        self.info("GetOrderTableNnumState exit")
        pass

    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)