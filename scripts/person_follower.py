#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import String, Header
import sys
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

global directionMod

class PersonFollower:

        def __init__(self):
                global directionMod
                directionMod = 1
                self.hz = 50;
                self.scan_sub = rospy.Subscriber('scan',
                        LaserScan, self.scan_callback)
                self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        def scan_callback(self, msg):
            global directionMod
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
            # Initialize Velocity msg to publish
            vel_msg = Twist()
            # Desired distance to stop from nearest object
            desired_distance = .5
            vel_msg.linear.x = .0
            # Once these values are approximately equal, we know that the robot is pointed towards its goal
            diff = abs(regions["left"] - regions["right"])
            # This statement checks to see if there are any valid nearby objects, or if the scanner is showing its default values
            if regions["left"] < 10 or regions["fleft"] < 10 or regions["front"] < 10 or regions["fright"] < 10 or regions["right"] < 10:
                # If an object is detected in this region, it is more effecient to turn left since it is on the left side
                if regions["fleft"] < 10 : 
                    directionMod = -1
                # Sets angular velocity to start turning, with a mod to influence its turn direction
                vel_msg.angular.z = .4*directionMod
                # This statment checks to see if our robot is pointed in the right direction while ensuring it is reading proper values from scan
                if diff < 0.2 and (regions["left"] != 10 or regions["right"] != 10):
                    # Resets direction modifier and angular velocity as it is no longer turning
                    directionMod = 1
                    vel_msg.angular.z = 0
                    # If it is farther away than desired from the object it will accelerate towards it
                    if regions["left"] > desired_distance and regions["right"] > desired_distance:
                        vel_msg.linear.x = .4
                    else:
                        vel_msg.linear.x = 0
            self.velocity_pub.publish(vel_msg)

        def run(self):
            rate = rospy.Rate(self.hz)
            while not rospy.is_shutdown():
                rate.sleep()
                
if __name__ == '__main__':
        rospy.init_node('range_ahead')
        follower = PersonFollower()
        follower.run()
