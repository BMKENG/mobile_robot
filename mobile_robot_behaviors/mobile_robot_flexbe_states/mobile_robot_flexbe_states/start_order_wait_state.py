#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher
# msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32

# 주문입력시 테이블 번호를 저장하고 대기하는 state
class StartOrder_waitState(EventState):
    def __init__(self, order_table_topic="/order_table", start_navi_topic="/start_nav", arrived_table_topic="/arrived_table"):
        # See example_state.py for basic explanations.
        super().__init__(input_keys=['table_num'], outcomes=['done']) 
        self.info("StartOrder_waitState init")

        self.table_num = None
        self.order_table_topic = order_table_topic
        self.start_navi_topic = start_navi_topic
        self.arrived_table_topic = arrived_table_topic

        # 도착 테이블 확인을 위한 pub 
        self.pub_arrived_table = ProxyPublisher({self.arrived_table_topic: Int32})
        self.arrived_table = Int32()

        # 테이블 주문을 위한 pub 
        self.pub_order_table = ProxyPublisher({self.order_table_topic: Int32})
        self.order_table = Int32()

        # 출발 start navi 확인을 위한 sub
        self.sub_start_navi = ProxySubscriberCached({self.start_navi_topic : Int32})
        self.start_navi_msg = None

        self.start_navi_flag = False

        self._sucess = False

    def start_navi(self):

        self.start_navi_msg = self.sub_start_navi.get_last_msg(self.start_navi_topic)
        if self.start_navi_msg is not None:
            self.start_navi_msg = self.start_navi_msg
            self.sub_start_navi.remove_last_msg(self.start_navi_topic)
            self.start_navi_msg = None
            return True
        else:
            return False


    def execute(self, userdata):
        self._sucess = self.start_navi()
        if self._sucess:
            self.info("StartOrder_waitState sucess")
            return 'done'
        else:
            pass
         
        
    def on_enter(self, userdata):
        self.info("StartOrder_waitState enter")
        self.table_num = userdata.table_num
        self._sucess = False
        if self.table_num == None:
            pass
        if self.table_num == 0:
            self.arrived_table.data = self.table_num
            self.pub_arrived_table.publish(self.arrived_table_topic, self.arrived_table)
        else:
            self.order_table.data = self.table_num
            self.pub_order_table.publish(self.order_table_topic, self.order_table)
        pass
        
    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.
        self.info("set_waypoint_state exit")
        pass

    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)