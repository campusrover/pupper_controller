#!/usr/bin/env python3


import rospy
import numpy as np
from pupper_msgs.msg import Box, Boxes
from geometry_msgs.msg import Twist


def sigmoid(x: float, scalar: float = 10):
    '''
    Description:
        A custom sigmoid function used for modulating either linear or angular
        speed based on a distance (x) value

    Params:
        x (float): input of the function
        scalar (float): multiplier for the exponential part of the function

    Returns:
        (float) Output of specialized sigmoid function with input x
    '''
    y = (2/(1+(2.72**(-scalar*x))))-1
    return y


def get_ang_vel(box, width):
    avg_x = (box.xmax - box.xmin)/2
    middle = width/2
    return sigmoid(middle-avg_x, 10) * 0.6


class Follower:

    def __init__(self, object: str):
        self.object = object
        self.boxes_sub = rospy.Subscriber("/boxes", Boxes, callback=self.boxes_cb)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.twist = Twist()
        self.h = None
        self.w = None
        self.target_box = None

    def boxes_cb(self, msg):
        self.h = msg.height
        self.w = msg.width
        potential_targets = []
        for box in msg.boxes:
            if box.label == self.object:
                potential_targets.append(box)
        max_score = 0
        target = None
        for t in potential_targets:
            if t.score > max_score:
                target = t
                max_score = t.score
        self.target_box = target
    
    def run(self):
        if self.target_box is None:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        else:
            self.twist.linear.x = 0.6
            self.twist.angular.z = get_ang_vel(self.target_box, self.width)
        self.twist_pub.publish(self.twist)


if __name__ == "__main__":
    rospy.init_node("follower")
    follower = Follower("remote")
    while not rospy.is_shutdown():
        rospy.sleep(0.08)
        follower.run()
        print((follower.target_box is not None))
