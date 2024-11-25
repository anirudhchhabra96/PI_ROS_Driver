#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import math
rospy.init_node("hex_commands")

pub = rospy.Publisher('/target_vel_hex', Float64MultiArray, queue_size=10)
msg = Float64MultiArray()
msg.data = [0, 0, 0, 0, 0, 0, 1]  # Example list of integers

k = 0

while not rospy.is_shutdown():
    msg.data = [0, 0, 0, 0, 0, 10*math.sin(k), 1]
    # msg.data = [10*math.sin(k), 0, 0, 0, 0, 0, 1]
    k = k + 1
    pub.publish(msg)
    rospy.sleep(1)
