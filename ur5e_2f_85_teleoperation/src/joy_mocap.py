#!/usr/bin/env python3

#####
#   This class is a simple JoyMocap. 
#   It publishes Twist message for the teleoperation node 
#
#####
import rospy
from geometry_msgs.msg import Twist

class JoyMocap:
    
    def __init__(self):
        rospy.init_node("joy_mocap_node")
        self._twist_pub = rospy.Publisher("joy_twist", Twist, queue_size=1)
    
    def publish_twist(self, linear_vel = [.0,.0,.0], angular_vel = [.0, .0, .0]):
        # create twist message
        twist_msg = Twist()
        # set linear vel
        twist_msg.linear.x = linear_vel[0]
        twist_msg.linear.y = linear_vel[1]
        twist_msg.linear.z = linear_vel[2]
        # set linear angular vel
        twist_msg.angular.x = angular_vel[0]
        twist_msg.angular.y = angular_vel[1]
        twist_msg.angular.z = angular_vel[2]
        self._twist_pub.publish(twist_msg)


if __name__ == '__main__':
    joy_mocap = JoyMocap()

    rate = rospy.Rate(50)

    linear_vel = [.0, .0, .0]
    angular_vel = [.0, .0, .0]
    while not rospy.is_shutdown():
        for j in range(300):
            if j % 100 == 0:
                rospy.loginfo(linear_vel)
            linear_vel = [0.5, .0, .0]
            joy_mocap.publish_twist(linear_vel=linear_vel)
            rate.sleep()
        for j in range(300):
            if j % 100 == 0:
                rospy.loginfo(linear_vel)
            linear_vel = [-0.5, .0, .0]
            joy_mocap.publish_twist(linear_vel=linear_vel)
            rate.sleep()
        for j in range(300):
            if j % 100 == 0:
                rospy.loginfo(angular_vel)
            angular_vel = [0., .0, 1.0]
            joy_mocap.publish_twist(angular_vel=angular_vel)
            rate.sleep()
        for j in range(300):
            if j % 100 == 0:
                rospy.loginfo(angular_vel)
            angular_vel = [0., .0, -1.0]
            joy_mocap.publish_twist(angular_vel=angular_vel)
            rate.sleep()