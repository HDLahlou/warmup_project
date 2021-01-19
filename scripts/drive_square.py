#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point, Pose, Twist, Vector3
from std_msgs.msg import Header


global setDist
global PI

class goInSquare(object):

    def manageMovement(self,curPos):
        print(curPos.pose.pose)
        rospy.sleep(2)


    def __init__(self):
        global setDist, PI
        setDist = 2
        PI = 3.1415926535897
        rospy.init_node('go_in_square')
        # Attempted to use odom, defaulted to using time instead
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.manageMovement)
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def clearVel(self, vel_msg):
        # Set Velocity Below
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        return vel_msg

    def straight(self):
        global setDist
        # Initialize vel_msg to publish
        vel_msg = Twist()
        #Ensure vel is clear of values
        vel_msg = self.clearVel(vel_msg)
        # Init distance for moving forward
        desired_dist = setDist
        current_dist = 0
        # Speed
        linear_velocity = .25
        # This below sets v to turn the robot
        vel_msg.linear.x = linear_velocity

        # Init time
        t0 = rospy.Time.now().to_sec()
        print(vel_msg)
        # Loop to set vel to change position
        while(current_dist < desired_dist):
            self.velocity_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_dist = linear_velocity*(t1-t0)
        # Ensure robot stops
        vel_msg.linear.x = 0
        self.velocity_pub.publish(vel_msg)
        return 0

    # Turn Function
    def turn(self):
        global PI
        # Initialize vel_msg to publish
        vel_msg = Twist()
        #Ensure vel is clear of values
        vel_msg = self.clearVel(vel_msg)
        # Init angles for 90 degree turn
        desired_angle = PI/2
        current_angle = 0

        # Speed of angular velocity
        angular_velocity = .3
        
        # This below sets v to turn the robot
        vel_msg.angular.z = angular_velocity

        # Init time
        t0 = rospy.Time.now().to_sec()

        # Loop to set vel to change angle
        while(current_angle < desired_angle):
            self.velocity_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_velocity*(t1-t0)
        
        # Ensure robot stops
        vel_msg.angular.z = 0
        self.velocity_pub.publish(vel_msg)
        return 0

    def run(self):
        print("Start")
        for x in range(4):
            self.straight()
            rospy.sleep(1)
            self.turn()
            rospy.sleep(1)

        rospy.spin()

if __name__ == '__main__':
    node = goInSquare()
    node.run()