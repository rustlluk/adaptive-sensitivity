#!/usr/bin/env python3
import rospy
import numpy as np
from bullet_ros_kuka.msg import JointTorque, JointQuantity
import PyKDL as kdl
from kdl_parser_py import urdf as kdl_parser
from sensor_msgs.msg import JointState


class KDL_PUBLISHER:
    """
    Takes values from robot joint torques and from kdl and publish directly the external torques
    """

    def __init__(self):
        self.sub = None
        self.vel_last = None
        self.pos_last = None
        self.time_last = None
        self.pub = None
        self.torques = kdl.JntArray(7)
        _, self.kdl_tree = kdl_parser.treeFromParam("robot_description")
        self.chain = self.kdl_tree.getChain("iiwa_link_0", "hand_base_link")
        self.id_solver = kdl.ChainIdSolver_RNE(self.chain, kdl.Vector(0, 0, -9.81))

    def kdl_to_np(self, ar):
        python_ar = np.zeros(ar.rows())
        for idx, _ in enumerate(ar):
            python_ar[idx] = _
        return python_ar

    def np_to_kdl(self, ar):
        kdl_ar = kdl.JntArray(len(ar))
        for idx, _ in enumerate(ar):
            kdl_ar[idx] = _
        return kdl_ar

    def __call__(self, js):
        """
        Check for torques threshold from external torques computed with openrave
        @return:
        @rtype:
        """
        if len(js.effort) != 7:
            return 0

        if self.vel_last is None:
            self.pos_last = np.array(js.position)
            self.vel_last = np.array(js.velocity)
            self.time_last = js.header.stamp.to_sec()
            return 0

        pos = np.array(js.position)

        time_curr = js.header.stamp.to_sec()
        vel = np.array(js.velocity)
        acc = (vel - self.vel_last) / float((time_curr - self.time_last) * 1e3)

        self.id_solver.CartToJnt(self.np_to_kdl(pos), self.np_to_kdl(vel), self.np_to_kdl(acc),
                                 [kdl.Wrench() for _ in range(self.chain.getNrOfSegments())], self.torques)
        effort = self.kdl_to_np(self.torques) - np.array(js.effort)
        # effort[np.abs(effort) > 10] = 0

        msg = JointTorque()
        msg.header.stamp = rospy.Time.now()
        msg.torque = JointQuantity(*effort)
        self.pub.publish(msg)

        self.pos_last = pos
        self.vel_last = vel
        self.time_last = time_curr
        return 0


if __name__ == "__main__":
    rospy.init_node("ext_torque_node")
    ext_torque_publisher = KDL_PUBLISHER()
    ext_torque_publisher.pub = rospy.Publisher("/iiwa/state/ExternalJointTorque", JointTorque, queue_size=1)
    ext_torque_publisher.sub = rospy.Subscriber("/joint_states", JointState, ext_torque_publisher)

    rospy.spin()