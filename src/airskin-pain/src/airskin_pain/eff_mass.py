#!/usr/bin/env python3

from airskin_pain.utils import KDLHandle
import rospy
from kdl_parser_py import urdf as kdl_parser
from sensor_msgs.msg import JointState
import numpy as np
import PyKDL as kdl
from std_msgs.msg import Int32MultiArray
import argparse
import sys


def prepare_parser():
    arg_parser = argparse.ArgumentParser(
        description="Main script for shape completion experiments"
    )

    arg_parser.add_argument(
        "--mode",
        "-m",
        dest="mode",
        required=False,
        default="mass",
        help="Mode of computation: mass or norm"
    )

    arg_parser.add_argument(
        "--robot",
        "-r",
        dest="robot",
        required=False,
        default="ur",
        help="Which robot to use: ur or kuka"
    )

    args_to_parse = []
    for arg in sys.argv[1:]:
        if ("__" and ":=" and "_") not in arg:
            args_to_parse.append(arg)

    args = arg_parser.parse_args(args_to_parse)
    return args.mode, args.robot


class Mass:
    def __init__(self, mode, robot):
        self.mode = mode
        self.robot = robot
        # kdl init
        _, self.kdl_tree = kdl_parser.treeFromParam("robot_description")
        self.kdl_handles = {}
        self.base_link = "base_link" if robot == "ur" else "iiwa_link_0"
        self.num_pads = 11 if robot == "ur" else 8
        if robot == "ur":
            # Add airskin links
            for pad in range(self.num_pads):
                self.kdl_handles[f"airskin_{pad}"] = KDLHandle(f"airskin_{pad}", self.kdl_tree, self.base_link)
        elif robot == "kuka":
            for link in range(self.num_pads):
                self.kdl_handles[f"iiwa_link_{link}"] = KDLHandle(f"iiwa_link_{link}", self.kdl_tree, self.base_link)

        self.force_levels = np.array([0, 280])
        self.force_levels_quasi = np.array([0, 140])

        self.hid_to_check = np.arange(self.num_pads)
        self.thresholds = Int32MultiArray(data=[0]*self.num_pads)

        self.pub = rospy.Publisher("/airskin_pain/thresholds", Int32MultiArray, queue_size=1)

        self.js = None
        self.js_vel = None

        self.quasi_pad = 9 if robot == "ur" else 6


    def __call__(self, msg):
        self.js = np.array(msg.position)
        self.js_vel = np.array(msg.velocity)

    def run(self):
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/joint_states", JointState)
            self.js = np.array(msg.position)
            self.js_vel = np.array(msg.velocity)
            if self.js is not None:
                for h_id, h in enumerate(self.kdl_handles.values()):
                    if h_id not in self.hid_to_check:
                        self.thresholds.data[h_id] = 9999
                    else:
                        js = self.js
                        js_vel = self.js_vel
                        pos_arr = h.np_to_kdl(js[:h.num_joints])
                        vel_arr = h.np_to_kdl(js_vel[:h.num_joints])
                        h.fk_vel_solver.JntToCart(kdl.JntArrayVel(pos_arr, vel_arr), h.fk_vel_frame)

                        u = h.kdl_to_np(h.fk_vel_frame.p.v, 3)
                        u_norm = np.linalg.norm(u)
                        if u_norm > 1e-2:
                            u /= u_norm

                            if self.mode == "mass":
                                h.jac_solver.JntToJac(h.np_to_kdl(js[:h.num_joints]), h.jac)
                                h.dyn_solver.JntToMass(h.np_to_kdl(js[:h.num_joints]), h.dyn)
                                M = h.kdl_to_np_mat(h.dyn)
                                J = h.kdl_to_np_mat(h.jac)

                                Ai = J @ (np.linalg.inv(M) @ J.T)
                                Avi = Ai[0:3, 0:3]  # == J[:3, :] @ (np.linalg.inv(M) @ J[:3, :].T)

                                m = 1 / (u.T @ Avi @ u)
                                if m > h.mass:
                                    m = 0
                            else:
                                m = h.mass/2
                            if m == 0:
                                F = 300
                            else:
                                F = (u_norm * np.sqrt(75000))/np.sqrt(1/m + 1/5.6)
                            if F < 0:
                                F = 0
                        else:
                            F = 300
                        if self.robot == "ur":
                            force_levels = self.force_levels if h_id != self.quasi_pad else self.force_levels_quasi
                        else:
                            force_levels = self.force_levels if np.linalg.norm(u - np.array([0, 0, -1]))>1e-1 else self.force_levels_quasi
                        self.thresholds.data[h_id] = np.nonzero(F >= force_levels)[0][-1]
                self.pub.publish(self.thresholds)


if __name__ == "__main__":
    rospy.init_node("eff_mass_calculator")
    mode, robot = prepare_parser()
    m = Mass(mode, robot)
    m.run()
