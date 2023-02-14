#!/usr/bin/env python3

import numpy as np
import time
from src.Controller import Controller
from src.State import State, BehaviorState
from src.Command import Command
from djipupper.Config import Configuration
from djipupper.Kinematics import four_legs_inverse_kinematics
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pupper_controller.msg import PupperFootPositions
from src.Utilities import deadband, clipped_first_order_filter
import argparse
import rospy
import datetime
import os

DIRECTORY = "logs/"
FILE_DESCRIPTOR = "walking"
MESSAGE_RATE = 20

def numpy_to_foot_positions(arr: np.ndarray):
    fp = PupperFootPositions()
    fp.foot_0.x = arr[0, 0]
    fp.foot_0.y = arr[1, 0]
    fp.foot_0.z = arr[2, 0]
    fp.foot_1.x = arr[0, 1]
    fp.foot_1.y = arr[1, 1]
    fp.foot_1.z = arr[2, 1]
    fp.foot_2.x = arr[0, 2]
    fp.foot_2.y = arr[1, 2]
    fp.foot_2.z = arr[2, 2]
    fp.foot_3.x = arr[0, 3]
    fp.foot_3.y = arr[1, 3]
    fp.foot_3.z = arr[2, 3]
    
    return fp



class DJIPupper:

    def __init__(self):
        self.config = Configuration()
        self.controller = Controller(self.config, four_legs_inverse_kinematics)
        self.state = State(height=self.config.default_z_ref)
        self.state.activation = 1
        self.summarize_config()

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
        self.reset_position = False

        input_curve = lambda x: np.sign(x) * min(x ** 2, 1)
        self.max_x_vel = input_curve(self.config.max_x_velocity*(1/self.config.max_x_velocity)) * self.config.max_x_velocity
        self.max_y_vel = input_curve(self.config.max_y_velocity*(1/self.config.max_y_velocity)) * -self.config.max_y_velocity
        self.max_horizontal_speed = np.sqrt(self.max_x_vel**2 + self.max_y_vel**2)

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, callback=self.twist_cb)
        self.reset_sub = rospy.Subscriber("reset_position", Bool, callback=self.reset_cb)
        self.foot_pos_pub = rospy.Publisher("foot_positions", PupperFootPositions, queue_size=1)

        self.prev_state = BehaviorState.REST


    def twist_cb(self, msg):
        self.lin_x = msg.linear.x
        self.lin_y = msg.linear.y
        self.lin_z = msg.linear.z

        self.roll = msg.angular.x
        self.pitch = msg.angular.y 
        self.yaw = msg.angular.z


    def reset_cb(self, msg):
        self.reset_position = msg.data


    def summarize_config(self):
        print("Summary of gait parameters:")
        print("overlap time: ", self.config.overlap_time)
        print("swing time: ", self.config.swing_time)
        print("z clearance: ", self.config.z_clearance)
        print("default height: ", self.config.default_z_ref)
        print("x shift: ", self.config.x_shift)


    def get_command(self, do_print=False):
        
        
        command = Command(height=self.config.default_z_ref)

        command.activate_event = False
        command.deactivate_event = False

        input_curve = lambda x: np.sign(x) * min(x ** 2, 1)
        x_vel = input_curve(self.lin_x) * self.config.max_x_velocity
        y_vel = input_curve(self.lin_y) * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        horizontal_speed = np.sqrt(x_vel**2 + y_vel**2)
        command.yaw_rate = self.yaw * -self.config.max_yaw_rate
        
        move_event = True

        if horizontal_speed == 0:
            command.stand_event = True
        elif  horizontal_speed >= self.max_horizontal_speed/16:
            command.trot_event = True
        else:
            command.walk_event = True





        self.prev_trot_toggle = command.trot_event
        self.prev_walk_toggle = command.walk_event 
        self.prev_stand_toggle = command.stand_event
        self.prev_move_toggle = move_event

        message_dt = 1.0 / MESSAGE_RATE

        deadbanded_pitch = deadband(self.pitch, self.config.pitch_deadband)
        pitch_rate = clipped_first_order_filter(
            self.state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = self.state.pitch + message_dt * pitch_rate

        command.height = (
            self.state.height - message_dt * self.config.z_speed * self.lin_z
        )

        command.roll = (
            self.state.roll + message_dt * self.config.roll_speed * self.roll
        )

        return command

    def run(self):
        command = self.get_command()
        if self.reset_position:
            self.state = self.controller.set_pose_to_default(self.state)
            self.reset_position = False
            final_foot_locations = numpy_to_foot_positions(self.config.default_stance)
            self.foot_pos_pub.publish(final_foot_locations)
            rospy.sleep(2)
        else:
            self.state = self.controller.run(self.state, command)
            final_foot_locations = numpy_to_foot_positions(self.state.final_foot_locations)
            self.foot_pos_pub.publish(final_foot_locations)








if __name__ == "__main__":
    rospy.init_node("djipupper_controller")
    pupper = DJIPupper()
    while not rospy.is_shutdown():
        rospy.sleep(pupper.config.dt)
        pupper.run()
        print(pupper.state.behavior_state)




