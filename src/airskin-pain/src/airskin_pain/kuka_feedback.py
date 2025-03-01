#!/usr/bin/env python3

import rospy
import numpy as np
import argparse


def prepare_parser():
    arg_parser = argparse.ArgumentParser(
        description="Main script for shape completion experiments"
    )

    arg_parser.add_argument(
        "--setup",
        "-s",
        dest="setup",
        required=False,
        default="sim",
        help="Setup"
    )


    args = arg_parser.parse_args()
    return args.setup


class CollisionDetection:
    def __init__(self, thresholds, setup="sim", buffer_size = 20):

        self.setup = setup
        if setup == "sim":
            from bullet_ros_kuka.msg import JointTorque, KukaTouch
        else:
            from iiwa_msgs.msg import JointTorque
            from kuka_humanoids.msg import KukaTouch

        # Force threshold for each one of the joints
        self.torque_thresholds = thresholds

        # Initialize external forces (torques)
        self.current_external_torques = [0.0] * 7

        # Initialize torque buffer
        self.torque_buffer = []
        # Initialize size of torque buffer
        self.buffer_size = buffer_size

        self.msg = KukaTouch()

        # Subscribe to the external joint torque topic
        rospy.Subscriber("/iiwa/state/ExternalJointTorque", JointTorque, callback=self.torque_callback)
        self.pub = rospy.Publisher("/airskin_pain/touch", KukaTouch, queue_size=1)
        self.run()

    def torque_callback(self, msg):
        """
        Callback to update forces (torques) for each joint from the JointTorque message.
        """

        new_torques = [
            msg.torque.a1, msg.torque.a2, msg.torque.a3,
            msg.torque.a4, msg.torque.a5, msg.torque.a6, msg.torque.a7
        ]

        # Add new torques to buffer
        self.torque_buffer.append(new_torques)

        # Limit buffer to buffer size
        if len(self.torque_buffer) > self.buffer_size:
            self.torque_buffer.pop(0)

        # Update current torques
        self.current_external_torques = new_torques

    def get_n_valid_torques(self,n):
        """
        Retrieves the n-th valid set of torques before the threshold was triggered.
        This function returns torque data from the buffer.
        """
        if self.torque_buffer and len(self.torque_buffer) >= n:
            return self.torque_buffer[n]

        # Default value
        return [0.0] * 7


    def isolate_collision_link(self, collision_torques):
        """
        Collision isolation algorithm to identify which link is in collision.
        The link that is in collision should be the one that has last non-zero value in collision_torques.

        collision_offset -> values of external torques in time =  buffer_size*rospy_rate(Hz) minus param of get_n_valid_torques.
        """
        collided_link = None

        # Lower index is older value
        """
        This will be probably highly dependent on moving speed
        """
        collision_offset = self.get_n_valid_torques(0)

        # Calculate the difference between the collision torques and the collision offset
        torque_difference = np.subtract(collision_torques, collision_offset)

        # Threshold to zero residuals
        torque_threshold = 0.01

        # Apply the threshold to make small residuals zero
        #torque_difference = np.where(np.abs(torque_difference) < torque_threshold, 0.0, torque_difference)
        # Mask the collision torques
        collision_torques = np.where(np.abs(torque_difference) < torque_threshold, 0.0, collision_torques)

        # find the link in collision
        for i, torque in enumerate(collision_torques):
            if abs(torque) > 0.0:
                collided_link = i + 1
            else:
                break
        rospy.logwarn(f"Collided link: {collided_link}")

        return collision_torques, collided_link

    def run(self):

        # Polling rate
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.msg.touch = []

            if np.any(np.abs(self.current_external_torques) > self.torque_thresholds):

                if self.setup == "real":
                    collision_torques = self.current_external_torques.copy()

                    collision_torques, collision_link = self.isolate_collision_link(collision_torques)
                    if collision_link is not None:
                        self.msg.touch = [collision_link]
                else:
                    self.msg.touch = np.where(np.abs(self.current_external_torques) > self.torque_thresholds)[0]+1
            self.pub.publish(self.msg)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("collision_detection")
    setup = prepare_parser()
    c = CollisionDetection([1.0] * 7, setup)

