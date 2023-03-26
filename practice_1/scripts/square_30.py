#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def drive():
    # Drive publisher
    pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd = Twist()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0

    # Parameters
    speed = rospy.get_param("~speed", 0.1)
    max_duration = rospy.get_param("~duration", 100)
    side_length = rospy.get_param("~side_length", 0.3)  # 30cm side length
    angular_speed = rospy.get_param("~angular_speed", 0.5)  # 0.5 rad/s

    # Initialize variables
    t_start = rospy.Time.now()
    stop = False
    current_side_length = 0
    current_angle = 0

    # Periodically publish the drive command to ensure drive continues
    rate = rospy.Rate(10)
    while (not stop) and (not rospy.is_shutdown()):
        # Drive straight
        if current_side_length < side_length:
            cmd.linear.x = speed
            cmd.angular.z = 0
        # Turn right
        elif current_angle < math.pi / 2:
            cmd.linear.x = 0
            cmd.angular.z = angular_speed
        # Drive straight again
        elif current_side_length < 2 * side_length:
            cmd.linear.x = speed
            cmd.angular.z = 0
        # Turn right again
        elif current_angle < math.pi:
            cmd.linear.x = 0
            cmd.angular.z = angular_speed
        # Drive straight one more time
        elif current_side_length < 3 * side_length:
            cmd.linear.x = speed
            cmd.angular.z = 0
        # Turn right one more time
        elif current_angle < 3 * math.pi / 2:
            cmd.linear.x = 0
            cmd.angular.z = angular_speed
        # Stop driving
        else:
            stop = True

        # Publish drive command
        pub_drive.publish(cmd)
        rospy.loginfo(cmd)
        rate.sleep()

        # Update variables
        t_now = rospy.Time.now()
        duration = t_now - t_start
        current_side_length = speed * duration.to_sec()
        current_angle = angular_speed * duration.to_sec()

        # Check if max duration exceeded
        if duration > rospy.Duration(max_duration):
            stop = True

    # Send stop command
    cmd.linear.x = 0
    cmd.angular.z = 0
    pub_drive.publish(cmd)
