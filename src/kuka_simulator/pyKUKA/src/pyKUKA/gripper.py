import os
import numpy as np


class Barrett:
    JOINTS = ["bh_j32_joint", "bh_j12_joint", "bh_j22_joint", "bh_j11_joint"] # moveable joints
    MIMICS = {"bh_j32_joint": [0.3442622950819672, "bh_j33_joint"], "bh_j12_joint": [0.3442622950819672, "bh_j13_joint"],
              "bh_j22_joint": [0.3442622950819672, "bh_j23_joint"], "bh_j11_joint": [1, "bh_j21_joint"]}

    def __init__(self, client):
        """
        Class for the RG6

        :param client: instance of Client, holding info about the scene
        :type client: Client
        """
        self.client = client
        self.id = client.robot
        self.constraints, self.finger_joint_ids, self.fingers_info = self.prepare_gripper()
        self.name = "gripper"
        self.finger_links_ids = [_.robot_link_id for _ in self.client.links if "finger" in _.name]
        self.max_force = 120

    def prepare_gripper(self):
        """
        Sets constraints for joints to works as mimic joint in URDF

        :return:
        :rtype:
        """

        # find ids of the 6 articulated joints
        joints_ids = []
        moveable_joint_ids = []
        fingers_info = []
        for idx in range(self.client.getNumJoints(self.id)):
            j_name = self.client.getJointInfo(self.id, idx)[1].decode("UTF-8")
            if j_name in self.JOINTS:
                moveable_joint_ids.append(idx)
                fingers_info.append(self.client.getJointInfo(self.id, idx))
                for idxx in range(self.client.getNumJoints(self.id)):
                    j_namee = self.client.getJointInfo(self.id, idxx)[1].decode("UTF-8")
                    if j_namee == self.MIMICS[j_name][1]:
                        break
                joints_ids.append((idx, idxx, self.MIMICS[j_name][0]))

        # Set constraints -> only one joint (finger_joint) is articulated in reality and 5 others should follow it
        # Set a gear constraint
        constraints = []
        for joint1, joint2, s in joints_ids:
            constraints.append(self.client.createConstraint(self.id, joint1,
                                                            self.id, joint2,
                                                            jointType=self.client.JOINT_GEAR,
                                                            jointAxis=[1, 0, 0],
                                                            parentFramePosition=[0, 0, 0],
                                                            childFramePosition=[0, 0, 0]))

            # Some have different direction, because of axes settings etc.
            self.client.changeConstraint(constraints[-1], gearRatio=-1/s,  # +1 in gear ratio means reverse direction
                                         maxForce=10, erp=1)
            self.client.setJointMotorControl2(self.id, joint2, self.client.POSITION_CONTROL, targetVelocity=0, force=0)

        return constraints, moveable_joint_ids, fingers_info

    def reset_constraints(self):
        """
        Remove all constraints

        :return:
        :rtype:
        """
        for c in self.constraints:
            self.client.removeConstraint(c)

    def stop(self):
        for idx, finger_joint_id in enumerate(self.finger_joint_ids):
            position = self.client.getJointState(self.id, finger_joint_id)[0]
            self.client.setJointMotorControl2(self.id, finger_joint_id,
                                              controlMode=self.client.POSITION_CONTROL, targetPosition=position,
                                              force=self.fingers_info[idx][self.client.jointInfo["MAXFORCE"]],
                                              maxVelocity=self.fingers_info[idx][self.client.jointInfo["MAXVELOCITY"]])

    def set_gripper_pose(self, position, pose=0, velocity=None, wait=False):
        """
        Set goal position of gripper

        :param position: position of the gripper
        :type position: float
        :param wait: whether to wait for the motion to finish
        :type wait: bool
        :return:
        :rtype:
        """
        assert 0 <= position <= 1, "Gripper width must be between 0 and 1"
        joints_min = 2.44
        joint_max = 0

        # pose 0 = two finger
        # pose 1 = 'one' finger

        if pose == 0:
            self.client.move_position("bh_j11_joint", 0.01, velocity=velocity, wait=True)
        elif pose == 1:
            self.client.move_position("bh_j11_joint", 3.14, velocity=velocity, wait=True)

        position = ((position - 0) / (1 - 0)) * (joint_max - joints_min) + joints_min
        self.client.move_position(["bh_j12_joint", "bh_j22_joint", "bh_j32_joint"], [position]*3, velocity=velocity, wait=wait)
