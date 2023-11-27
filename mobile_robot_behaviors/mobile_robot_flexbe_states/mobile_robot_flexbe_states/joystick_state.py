#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached


# joystick command
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class JoystickState(EventState):

    def __init__(self, set_angular_vel = 0.5, set_linear_vel = 0.5, joystick_topic="/joy_cmd", cmd_vel_topic="/cmd_vel", set_vel_topic="/set_vel"):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done']) 

        self.gui_topic = joystick_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.set_vel_topic = set_vel_topic
        self._table_num = None


        self.joystick = None

        self.set_angular_velocity = set_angular_vel
        self.set_linear_velocity = set_linear_vel

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.sub_gui_command = ProxySubscriberCached({self.gui_topic: String})
        self.sub_gui_command.set_callback(self.gui_topic, self.joystick_callback)
        self.pub_set_vel = ProxyPublisher({self.set_vel_topic: Twist})
        self.pub_cmd_vel = ProxyPublisher({self.cmd_vel_topic: Twist})

        # joy flag
        self.joy_flag = False

        # joystick command dict
        self.control_dict = {
            'go': self.go_button,
            'back': self.back_button,
            'stop': self.stop_button,
            'left': self.left_button,
            'right': self.right_button,
            'angular_up': self.angular_up_button,
            'angular_down': self.angular_down_button,
            'linear_up': self.linear_up_button,
            'linear_down': self.linear_down_button,
            'emergency' : self.emergency_button,
        }

        
        # It may happen that the action client fails to send the action goal.
        self._sucess = False
        self._failed = False

    
    def joystick_callback(self, msg):
        # 올바른 키가 입력되었는지 확인되었으면 해당 키에 맞는 함수를 실행한다.
        if msg is None:
            return
        
        if msg.data in self.control_dict:
            self.control_dict[msg.data]()
            self.info('Joystick command: ' + msg.data)
            # last msg clear 
        else:
            self.warn('Invalid joystick command: ' + msg.data)

    def send_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.pub_cmd_vel.publish(self.cmd_vel_topic, msg)


    def send_set_vel(self):
        msg = Twist()
        msg.linear.x = self.set_linear_velocity
        msg.angular.z = self.set_angular_velocity
        self.pub_set_vel.publish(self.set_vel_topic, msg)  
         
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

    # vel up down button
    def angular_up_button(self):
        self.set_angular_velocity += 0.1
        self.stop_button()
        if self.set_angular_velocity >= 1.0:
            self.set_angular_velocity = 1.0
        self.send_set_vel()


    def angular_down_button(self):
        self.set_angular_velocity -= 0.1
        self.stop_button()
        if self.set_angular_velocity <= 0.1:
            self.set_angular_velocity = 0.1
        self.send_set_vel()

    def linear_up_button(self):
        self.set_linear_velocity += 0.1
        self.stop_button()
        if self.set_linear_velocity >= 1.0:
            self.set_linear_velocity = 1.0
        self.send_set_vel()

    def linear_down_button(self):
        self.set_linear_velocity -= 0.1
        self.stop_button()
        if self.set_linear_velocity <= 0.1:
            self.set_linear_velocity = 0.1
        self.send_set_vel()

    def emergency_button(self):
        self.stop_button()
        self._sucess = True


    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        # joystick command subscriber
        self.info("start joystick")


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.
        self.stop_button()
        self.info("stop joystick")

    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)