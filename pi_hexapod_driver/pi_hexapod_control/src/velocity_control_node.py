#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
import time
import warnings
from std_msgs.msg import Int32MultiArray

class HexTrajectoryControl(object):
    def __init__(self):
        rospy.init_node("hex_trajectory_control")
        self.trajectory_list = [
           [0.0, 0.0, 0, 0, 0.0, 0.0] for i in range(1)
        ]
        self.how_long = 0.0

        self.get_vel_cmd = rospy.Subscriber('/target_vel_hex', Float64MultiArray ,self.get_vel_cmd_callback)

        self.qdot_pub = rospy.Publisher(
                        '/pi_hardware_interface/qdot_cmd',
                        JointTrajectory,
                        queue_size=10
        )

        # Each qdot trajectory point will execute for this time (in ms).

    def get_vel_cmd_callback(self, msg):
        self.trajectory_list = [
           msg.data[:5] for i in range(1)
           
        ]
        # print(msg.data)
        self.how_long = msg.data[-1]*1000

    def publish(self, trajectory: list, qdot_period:float=50):
        """
        Commands hexapod to execute trajectory.

        PARAMS:
          trajectory - A list of list where the inner list defines the 
                       velocity for each axis (X, Y, Z, U, V, W).
                       Velocity is in mm/s for translation and mrad/s for rotation.

                       The outer list defines a point on the trajectory.
                       Each point is executed after the qdot_period.

          qdot_period - The amount of time (in ms) to execute each point.
                        The hexapod will move at the commanded rate for this long.
        """
        trajectory_msg = JointTrajectory()

        for point in trajectory:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.velocities = point
            print(point)
            for element in point:
                if abs(element) > 20:
                    print(f"WARNING: Velocity command ({element}) is > max (20 mm/s).")
            trajectory_msg.points.append(trajectory_point)

        trajectory_msg.points[0].effort.append(qdot_period)

        self.qdot_pub.publish(trajectory_msg)


    def run(self):
        # trajectory = [
        #    [-20.0, 0.0, 0, 0, 0.0, 0.0] for i in range(1)
        # ]

        #while not rospy.is_shutdown():
        for i in range(2):
            # howlong = 4*1000
            self.publish(self.trajectory_list, self.how_long)
            rospy.sleep(1)

if __name__ == "__main__":
    node = HexTrajectoryControl()
    # while not rospy.is_shutdown():
    node.run()