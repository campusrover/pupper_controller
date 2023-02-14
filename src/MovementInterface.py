import numpy as np
import time
from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter
import rospy
from geometry_msgs.msg import Twist

MESSAGE_RATE = 20

class MovementInterface:
    def __init__(self, config, udp_port=8830, udp_publisher_port=8840):
        self.config = config

        self.prev_trot_toggle = True
        self.prev_walk_toggle = False
        self.prev_stand_toggle = True
        self.prev_move_toggle = False

        self.lin_x = 0
        self.lin_y = 0
        self.lin_z = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.max_horizontal_speed = np.sqrt(self.config.max_x_velocity**2 + config.max_y_velocity**2)

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, callback=self.twist_cb)


    def twist_cb(self, msg):
        self.lin_x = msg.linear.x 
        self.lin_y = msg.linear.y 
        self.lin_z = msg.linear.z 

        self.roll = msg.angular.x
        self.pitch = msg.angular.y 
        self.yaw = msg.angular.z  
        

    def get_command(self, state, do_print=False):
        

        command = Command(height=self.config.default_z_ref)

        command.activate_event = true
        command.deactivate_event = false

        input_curve = lambda x: np.sign(x) * min(x ** 2, 1)
        x_vel = input_curve(self.lin_y) * self.config.max_x_velocity
        y_vel = input_curve(self.lin_x) * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        horizontal_speed = np.sqrt(x_vel**2 + y_vel**2)
        command.yaw_rate = self.yaw * -self.config.max_yaw_rate
        
        move_event = horizontal_speed > 0 and self.yaw > 0

        command.trot_event = (horizontal_speed < 0.5*self.max_horizontal_speed) and (horizontal_speed > 0)
        command.walk_event = horizontal_speed >= 0.5*max_horizontal_speed

        command.stand_event = horizontal_speed == 0

        self.prev_trot_toggle = command.trot_event
        self.prev_walk_toggle = command.walk_event 
        self.prev_stand_toggle = command.stand_event
        self.prev_move_toggle = move_event

        message_dt = 1.0 / MESSAGE_RATE

        deadbanded_pitch = deadband(self.pitch, self.config.pitch_deadband)
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = state.pitch + message_dt * pitch_rate

        command.height = (
            state.height - message_dt * self.config.z_speed * self.lin_z
        )

        command.roll = (
            state.roll + message_dt * self.config.roll_speed * self.roll
        )

        return command

    def set_color(self, color):
        pass
