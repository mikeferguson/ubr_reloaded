#!/usr/bin/env python

"""
This tutorial moves the robot forward and then back again.
"""

import rospy
from geometry_msgs.msg import Twist

if __name__=='__main__':
    rospy.init_node('move_my_base')

    # Create a publisher to command the base    
    pub = rospy.Publisher('base_controller/command', Twist)

    # Move forward for about a second
    #   We have to send the message multiple times because the
    #   base controller will time out if we only send the command once
    rospy.loginfo('Moving forward!')
    for i in range(20):

        # Create a twist message to move forward 
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0

        # Send the command
        pub.publish(msg)

        # Wait before sending again
        rospy.sleep(0.1)

    # Stop and wait a moment for robot to fully stop
    rospy.loginfo('Stopping')
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)
    rospy.sleep(0.5)

    # Now backup
    rospy.loginfo('Moving Backward!')
    for i in range(20):

        # Create a twist message to move forward 
        msg = Twist()
        msg.linear.x = -0.2
        msg.angular.z = 0.0

        # Send the command
        pub.publish(msg)

        # Wait before sending again
        rospy.sleep(0.1)

    # Stop and wait a moment for robot to fully stop
    rospy.loginfo('Stopping')
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)

    rospy.loginfo('Done')

