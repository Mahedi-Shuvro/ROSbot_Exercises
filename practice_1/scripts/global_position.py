
"""this code calculates the global position of the robot as it moves and logs it at a rate of 10 Hz.
This can be useful for tracking the robot's movement and navigating it to specific locations."""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def callback(data):
    global x, y, theta, v_x, v_theta
    v_x = data.twist.twist.linear.x
    v_theta = data.twist.twist.angular.z
    dt = (data.header.stamp - last_time).to_sec()
    last_time = data.header.stamp
    delta_x = (v_x * math.cos(theta) * dt)
    delta_y = (v_x * math.sin(theta) * dt)
    delta_theta = (v_theta * dt)
    x += delta_x
    y += delta_y
    theta += delta_theta

def listener():
    global x, y, theta, last_time
    x = 0
    y = 0
    theta = 0
    v_x = 0
    v_theta = 0
    last_time = rospy.Time.now()

    rospy.init_node('odometry')
    rospy.Subscriber('/odom', Odometry, callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Global Position: x=%f, y=%f, theta=%f", x, y, theta)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
