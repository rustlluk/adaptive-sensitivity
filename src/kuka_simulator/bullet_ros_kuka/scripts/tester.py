#!/usr/bin/env python3

import rospy
from bullet_ros_kuka.motion_interface import MoveGroupPythonInterface
from bullet_ros_kuka.robot_kinematics_interface import ForwardKinematics, InverseKinematics
import numpy as np
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("examples_node")

    """
    Connect to Motion Interface with group 'iiwa_arm'
        - group 'iiwa_arm' is from base_link to hand_base_link frame
        - other groups can be defined in iiwa_bhand_moveit_config/config/iiwa7.srdf
    """
    MoveGroupArm = MoveGroupPythonInterface("iiwa_arm")
    """
    Just info message
    """
    rospy.loginfo("Starting ")

    """VELOCITY CONTROL"""
    MoveGroupArm.move_velocity([-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 0.1 rad/s in joint_0

    rospy.sleep(5)  # do the movement for some time
    # Stop the movement
    MoveGroupArm.move_velocity([0, 0, 0, 0, 0, 0, 0])
