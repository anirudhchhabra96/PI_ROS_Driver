#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import math
rospy.init_node("hex_commands")

pub = rospy.Publisher('/target_vel_hex', Float64MultiArray, queue_size=1)
msg = Float64MultiArray()
msg.data = [0, 0, 0, 0, 0, 0, 1]  # Example list of integers

k = 1

def compute_velocity(d, angular_velocity):
    """
    Compute the 6-DOF velocity command to rotate the cube about its center.

    ARGS:
        d (float) - Offset along z-axis between joint origin and cube center.
        angular_velocity (tuple): Desired angular velocities (omega_x, omega_y, omega_z).
    
    Returns 6-DOF velocity command [v_x, v_y, v_z, omega_x, omega_y, omega_z].
    """
    omega_x, omega_y, omega_z = angular_velocity

    v_x = omega_y * d
    v_y = -omega_x * d
    v_z = 0

    return [v_x, v_y, v_z, omega_x, omega_y, omega_z]

i = 0
while not rospy.is_shutdown():
    msg.data = [0, 0, 0, 0, 0, 1*math.sin(k/10), 100]
    # msg.data = [10*math.sin(k), 0, 0, 0, 0, 0, 1]
    k = k+0.1
    # if i > 20:
    #     k = -k
    #     i = 0

    # i += 1
    pub.publish(msg)
    rospy.sleep(0.001)





