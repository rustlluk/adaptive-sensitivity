#!/usr/bin/env python3

import rospy
from bullet_ros_kuka.motion_interface import MoveGroupPythonInterface
from bullet_ros_kuka.robot_kinematics_interface import ForwardKinematics, InverseKinematics
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
# from bhand_controller.srv import Actions, ActionsRequest
# from bhand_controller.msg import State

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

    """
    Get current end-effector and get its pose:
    """
    rospy.loginfo(f"Current end-effector is {MoveGroupArm.eef_link}")
    rospy.loginfo(f"Pose of the end-effector is: {MoveGroupArm.get_ee_pose()}")

    """
    Move the robot 5cm up from its current position
    """
    p = MoveGroupArm.get_ee_pose()
    p.position.z += 0.05
    MoveGroupArm.go_to_pose(p, wait=True)  # True here is to block the program until completion of the movement.

    """
    Get current joint state of the robot:
    """
    rospy.loginfo(f"Joint state is: {MoveGroupArm.get_current_state()}")

    """
    Move joint_0 25 degrees from current position
    """
    p = MoveGroupArm.get_current_state()
    p[0] += np.deg2rad(25)
    MoveGroupArm.go_to_joint_position(p)


    # """
    # Open the gripper
    # """
    # rospy.loginfo("Opening gripper")
    # MoveGroupArm.open_gripper()
    #
    # """
    # Close the gripper
    # """
    # rospy.loginfo("Closing gripper")
    # MoveGroupArm.close_gripper()
    #
    # """
    # The previous command corresponds to calling /bhand_node/actions with action 2 for closing and action 3 for opening
    # To do it, you need to active the hand (done only once)
    # However, it can be done EXACTLY ONE, else it goes into error state
    # """
    #
    # grip_service = rospy.ServiceProxy("/bhand_node/actions", Actions)
    # gripper_state = rospy.wait_for_message("/bhand_node/state", State)
    # if not gripper_state.hand_initialized:
    #     request = ActionsRequest()
    #     request.action = 1
    #
    #     rospy.loginfo(f"Gripper initialized successfully: {grip_service.call(request)}")
    #
    # """
    # And then call specific action. Now to open
    # """
    # request = ActionsRequest()
    # request.action = 3

    # # wait for completion
    # rospy.sleep(1)
    # rospy.loginfo(f"Gripper opened successfully: {grip_service.call(request)}")

    """
    Get inverse kinematics without movement
    """
    # Create pose request
    pose_current = MoveGroupArm.get_ee_pose()

    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "iiwa_link_0"
    ps.pose = pose_current

    ik = InverseKinematics()

    result = ik.getIK(group_name="iiwa_arm", ik_link_name="hand_base_link", pose=ps, avoid_collisions=True, timeout=0.1)
    rospy.loginfo(f"The IK solution is: {result}")

    """VELOCITY CONTROL"""
    MoveGroupArm.move_velocity([-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 0.1 rad/s in joint_0

    rospy.sleep(5)  # do the movement for some time
    # Stop the movement
    MoveGroupArm.move_velocity([0, 0, 0, 0, 0, 0, 0])
