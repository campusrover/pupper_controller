#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

from djipupper.Config import Configuration



config = Configuration()
LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.1
ROLL_STEP_SIZE = 0.2
PITCH_STEP_SIZE = 0.2
msg = """
Control Your Pupper!
---------------------------
Moving around:
   q    w    e    u    i
   a    s    d    j    k
   z         c      

w/s : increase/decrease x velocity
d/a : increase/decrease y velocity
u/j : increase/decrease z velocity
e/q : increase/decrease yaw velocity
z/c : increase/decrease roll velocity
i/k : increase/decrease pitch velocity

space key : force stop
x : reset to default pose

CTRL-C to quit
"""

# e = """
# Communications Failed
# """

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(x, y, z, roll, pitch, yaw):
    r = lambda n: round(n, 2)
    return f"x: {r(x)} m/s, y: {r(y)} m/s, z: {r(z)} m/s, roll: {r(roll)} rad/s, pitch: {r(pitch)} rad/s, yaw: {r(yaw)} rad/s"

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, high):
    if input < -1*high:
      input = -1*high
    elif input > high:
      input = high
    else:
      input = input

    return input



if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('pupper_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    reset_pub = rospy.Publisher('reset_position', Bool, queue_size=10)


    status = 0
    target_x = 0.0
    target_y = 0.0
    target_z = 0.0
    target_roll = 0.0
    target_pitch = 0.0
    target_yaw = 0.0
    reset_position = False


    max_x = config.max_x_velocity 
    max_y = config.max_y_velocity 
    max_z = config.z_speed

    max_roll = config.roll_speed
    max_pitch = config.max_pitch_rate
    max_yaw = config.max_yaw_rate



    print(msg)
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w' :
            target_x = constrain(target_x + LIN_VEL_STEP_SIZE, max_x)
            status = status + 1
            print(vels(target_x,target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 's' :
            target_x = constrain(target_x - LIN_VEL_STEP_SIZE, max_x)
            status = status + 1
            print(vels(target_x,target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'd' :
            target_y = constrain(target_y + LIN_VEL_STEP_SIZE, max_y)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'a' :
            target_y = constrain(target_y - LIN_VEL_STEP_SIZE, max_y)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'u' :
            target_z = constrain(target_z + LIN_VEL_STEP_SIZE, max_z)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'j' :
            target_z = constrain(target_z - LIN_VEL_STEP_SIZE, max_z)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'e' :
            target_yaw = constrain(target_yaw + ANG_VEL_STEP_SIZE, max_yaw)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'q' :
            target_yaw = constrain(target_yaw - ANG_VEL_STEP_SIZE, max_yaw)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'i' :
            target_pitch = constrain(target_pitch + ANG_VEL_STEP_SIZE, max_pitch)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'k' :
            target_pitch = constrain(target_pitch - ANG_VEL_STEP_SIZE, max_pitch)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'z' :
            target_roll = constrain(target_roll + ANG_VEL_STEP_SIZE, max_roll)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == 'c' :
            target_roll = constrain(target_roll - ANG_VEL_STEP_SIZE, max_roll)
            status = status + 1
            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        elif key == ' ':
            status = 0
            target_x = 0.0
            target_y = 0.0
            target_z = 0.0
            target_roll = 0.0
            target_pitch = 0.0
            target_yaw = 0.0
        if key == 'x':
            reset_posiiton = True

            print(vels(target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
        else:
            if (key == '\x03'):
                break

        if status == 20 :
            print(msg)
            status = 0

        twist = Twist()
        reset_msg = Bool()
        twist.linear.x = target_x
        twist.linear.y = target_y
        twist.linear.z = target_z
        twist.angular.x = target_roll 
        twist.angular.y = target_pitch
        twist.angular.z = target_yaw
        reset_msg.data = reset_position

        pub.publish(twist)
        reset_pub.publish(reset_msg)



    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
