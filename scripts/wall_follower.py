#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import String, Header
import sys
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

global prevAdjust
global sumAdjust
global control
global kp
global dt
global ti
global td

class Follower:

        def __init__(self):
                global prevAdjust, sumAdjust, control, kp, dt, ti, td
                prevAdjust = 0
                sumAdjust = 0
                control = 0
                # Values for adjusting for error, determines angular.z calculation
                kp = -1
                dt = .02
                ti = .00001
                td = -1.2

                self.hz = 50;

                self.scan_sub = rospy.Subscriber('scan',
                        LaserScan, self.scan_callback)
                self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                vel_msg = Twist()
                vel_msg.linear.x = .3
                self.velocity_pub.publish(vel_msg)

        # Takes sum and difference of error to adjust angular acceleration
        def calc_control(self, curr):
            global prevAdjust, sumAdjust, control, kp, dt, ti, td
            prev = prevAdjust
            # Integral error
            # Error adjustment based  on total error/diff throughout run
            sumAdjust = sumAdjust + curr * dt

            # Derivative error
            # Uses most recent error and current error diff to provide a fine tune adjustment
            derivAdjust = (curr - prev)/ dt

            # Combines these error calculations along with current offset to find desired direction
            control = kp * curr + ti * sumAdjust + td * derivAdjust

            prevAdjust = curr


        def scan_callback(self, msg):
            global control
            size = len(msg.ranges)
            # Determines the ranges for scan, finds each "side"
            regions = {
                'right':  min(min(msg.ranges[0:(int(size/5)-1)]), 10),
                'fright': min(min(msg.ranges[(int(size/5)):(int(2*size/5)-1)]), 10),
                'front':  min(min(msg.ranges[(int(2*size/5)):(int(3*size/5)-1)]), 10),
                'fleft':  min(min(msg.ranges[(int(3*size/5)):(int(4*size/5)-1)]), 10),
                'left':   min(min(msg.ranges[(int(4*size/5)): (size-1)]), 10),
            }
            print("range ahead: left - %0.1f" %regions['left'], "fleft - %0.1f" %regions['fleft'], " center- %0.1f"%regions['front'], "fright - %0.1f" %regions['fright'], " right - %0.1f" %regions['right'])
            # Uses distance from left to find difference between current and desired
            # Uses this value as an input in calc_control helper to find angular acceleration
            desired_distance = .5
            diff = desired_distance - regions["left"]
            self.calc_control(diff)
            vel_msg = Twist()
            vel_msg.linear.x = .3
            vel_msg.angular.z = control
            self.velocity_pub.publish(vel_msg)

        def run(self):
            rate = rospy.Rate(self.hz)
            while not rospy.is_shutdown():
                rate.sleep()
                
if __name__ == '__main__':
        rospy.init_node('range_ahead')
        follower = Follower()
        follower.run()
