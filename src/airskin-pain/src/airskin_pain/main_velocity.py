#!/usr/bin/env python3
import copy
import os.path
from subprocess import Popen, PIPE
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_msgs.msg import Float64MultiArray, Int32, Int32MultiArray
import tf
import PyKDL as kdl
from kdl_parser_py import urdf as kdl_parser
import numpy as np
from sensor_msgs.msg import JointState
import tf.transformations as ts
import datetime
import argparse
from airskin_pain.utils import KDLHandle

class Experiment:
    INIT_POSE_UR = [0.085, -1.360, 1.724, -0.313, 0.084+np.pi/2, -0.057]
    INIT_POSE_KUKA = [-0.5752605214573311, 0.3913028182971287, -0.4846779332788253, -1.9710003242771965, 0.4363, -2.04, -0.3361504139341079]

    def __init__(self, robot="ur", setup="sim", save_bag=True, bag_name=None, velocity=0.6):
        rospy.init_node("experiment_node")
        if setup == "sim":
            if robot == "ur":
                from bullet_ros_ur.msg import AirskinTouch
                from bullet_ros_ur.robot_kinematics_interface import ForwardKinematics
                from bullet_ros_ur.motion_interface import MoveGroupPythonInterface
            elif robot == "kuka":
                from bullet_ros_kuka.msg import KukaTouch
                from bullet_ros_kuka.robot_kinematics_interface import ForwardKinematics
                from bullet_ros_kuka.motion_interface import MoveGroupPythonInterface
        elif setup == "real":
            if robot == "ur":
                from airskin.msg import AirskinTouch
                from ur10e_humanoids.robot_kinematics_interface import ForwardKinematics
                from ur10e_humanoids.motion_interface import MoveGroupPythonInterface
            elif robot == "kuka":
                from kuka_humanoids.msg import KukaTouch
                from kuka_humanoids.robot_kinematics_interface import ForwardKinematics
                from kuka_humanoids.motion_interface import MoveGroupPythonInterface
        else:
            raise ValueError("setup must be 'sim' or 'real")
        self.setup = setup
        self.robot = robot
        self.num_link = 6 if robot == "ur" else 7
        self.INIT_POSE = self.INIT_POSE_UR if robot == "ur" else self.INIT_POSE_KUKA
        if save_bag:
            if bag_name is None:
                start_time_ = datetime.datetime.now()
                bag_name = str(start_time_).replace(".", "-").replace(" ", "-").replace(":", "-")
            bag_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "bags", bag_name + ".bag")
            if not os.path.exists(os.path.dirname(bag_path)):
                os.makedirs(os.path.dirname(bag_path))
            command = f"rosbag record -O {bag_path} -a __name:=my_bag"
            Popen(command, stdout=PIPE, shell=True)

        # FK service
        self.fk = ForwardKinematics()

        # controller mode switch service
        if robot == "ur":
            self.switch_srv = rospy.ServiceProxy("/robot/controller_manager/switch_controller", SwitchController)
        elif robot == "kuka":
            self.switch_srv = rospy.ServiceProxy("/iiwa/controller_manager/switch_controller", SwitchController)
        self.switch_srv.wait_for_service()

        # tf listener -> it is better to have one during the whole time
        self.listener = tf.TransformListener()

        # publisher for velocity control
        if robot == "ur":
            self.velocity_publisher = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=1, latch=True)
        elif robot == "kuka":
            self.velocity_publisher = rospy.Publisher("/iiwa/command/JointVelocity", Float64MultiArray, queue_size=1, latch=True)
        self.velocity_msg = Float64MultiArray()

        # publisher for velocity control
        self.cart_vel_pub = rospy.Publisher("/airskin_pain/cart_vel", Float64MultiArray, queue_size=1, latch=True)
        self.cart_vel_msg = Float64MultiArray()

        try:
            # python move_group commander interface
            if robot == "ur":
                self.mg = MoveGroupPythonInterface("manipulator")
            elif robot == "kuka":
                self.mg = MoveGroupPythonInterface("iiwa_arm")
        except:
            if robot == "ur":
                self.mg = MoveGroupPythonInterface("manipulator")
            elif robot == "kuka":
                self.mg = MoveGroupPythonInterface("iiwa_arm")

        # joint states subscriber
        self.js = np.zeros((7,))
        self.js_vel = np.zeros((7,))
        self.last_js_msg = None
        self.js_sub = rospy.Subscriber("/joint_states", JointState, self.get_js, queue_size=1)
        self.joint_names = rospy.wait_for_message("/joint_states", JointState).name[:self.num_link]

        # kdl init
        _, self.kdl_tree = kdl_parser.treeFromParam("robot_description")
        self.kdl_handles = {}

        self.base_link = "base_link" if robot == "ur" else "iiwa_link_0"

        if robot == "ur":
            # Add airskin links
            for pad in range(11):
                self.kdl_handles[f"airskin_{pad}"] = KDLHandle(f"airskin_{pad}", self.kdl_tree, self.base_link)
        elif robot == "kuka":
            for link in range(8):
                self.kdl_handles[f"iiwa_link_{link}"] = KDLHandle(f"iiwa_link_{link}", self.kdl_tree, self.base_link)

        if robot == "ur":
            # add the end-effector
            self.kdl_handles["gripper"] = KDLHandle("gripper_link", self.kdl_tree, self.base_link) #gripper_link
        else:
            self.kdl_handles["gripper"] = KDLHandle("iiwa_link_7", self.kdl_tree, self.base_link)

        # Generic properties
        self.phase = 0
        self.js_when_impact = None
        self.js_vel_when_impact = None
        self.phase_when_impact = 0
        self.max_velocity = velocity
        self.min_velocity = 0.05
        self.link_velocity = self.max_velocity
        self.end_points = []
        self.impact_time = None
        self.cooldown = None
        self.static_duration = 0.5
        self.cooldown_duration = 1
        self.current_velocity = None
        self.stop_duration = 1
        self.last_pain = None

        self.behaviour = "stop"

        # airskin processing
        self.pain = None
        if robot == "ur":
            self.airskin_sub = rospy.Subscriber("/airskin_pain/touch", AirskinTouch, self.airskin_cb, queue_size=1)
        elif robot == "kuka":
            self.airskin_sub = rospy.Subscriber("/airskin_pain/touch", KukaTouch, self.airskin_cb, queue_size=1)
        self.airskin_act_time = None

        # thresholds
        self.thresholds = np.ones((11,))
        self.ths_sub = rospy.Subscriber("/airskin_pain/thresholds", Int32MultiArray, self.ths_cb, queue_size=1)

        # status publisher
        self.status_pub = rospy.Publisher("/airskin_pain/phase", Int32, queue_size=1, latch=True)

        if robot == "ur":
            self.axes = [-1, 1, 2, 0, 2, 2, 1]
        elif robot == "kuka":
            self.axes = [-1, 0, 2, 1, 2, 2, 0]
        self.generate_end_points()
        self.avoid_direction = None
        self.rate = rospy.Rate(100)
        self.forward = np.array([0, 1, 0]) if robot == "ur" else np.array([1, 0, 0])
        self.left = np.array([-1, 0, 0]) if robot == "ur" else np.array([0, 1, 0])

        if self.robot == "kuka" and self.setup == "sim":
            self.collision_objects = []
            from bullet_ros_kuka.msg import Collisions
            self.collision_objects_sub = rospy.Subscriber("/bullet_ros/collisions", Collisions, self.collision_objects_cb, queue_size=1)

    def collision_objects_cb(self, msg):
        self.collision_objects = np.array(msg.obj_names)

    def __del__(self):
        proc = Popen("rosnode kill /my_bag", shell=True, stdout=PIPE, stderr=PIPE)
        proc.wait()

    def ths_cb(self, msg):
        self.thresholds = np.array(msg.data)


    def airskin_cb(self, msg):
        if self.phase == 5: # KEEP HERE!!! Useful when robot going up from qs collision
            return 0
        touch = np.array(msg.touch)
        if len(touch) > 0 and np.any(self.thresholds[touch] == 1):
            if self.pain is not None:
                t = rospy.Time.now()
                if t - self.impact_time > rospy.Duration(self.static_duration):
                    self.pain = None
                    self.phase = -1
                    self.impact_time = None
                    self.cooldown = t
            else:
                if self.cooldown is None or not np.any(np.isin(self.last_pain, touch)) or (rospy.Time.now() - self.cooldown > rospy.Duration(self.cooldown_duration)):
                    self.pain = touch
                    self.last_pain = self.pain
                    if self.phase != -1 and self.phase != -2:
                        self.phase_when_impact = self.phase
                    self.phase = -2
                    self.js_when_impact = np.array(self.js)
                    self.js_vel_when_impact = np.array(self.js_vel)
                    self.impact_time = rospy.Time.now()
                    self.cooldown = None
        else:
            if self.pain is not None:
                self.pain = None
                self.impact_time = None
                self.phase = -1

    def generate_end_points(self):
        self.end_points.append(None)
        if self.robot == "kuka":
            start_pose = self.fk.getFK(self.kdl_handles["gripper"].end_effector, self.joint_names, self.INIT_POSE)
        else:
            start_pose = self.fk.getFK("wrist_1_link", self.joint_names, self.INIT_POSE)
        start_point = np.array([getattr(start_pose.pose_stamped[0].pose.position, _) for _ in ["x", "y", "z"]])

        if self.robot == "ur":
            offsets = [[0, 0.55, 0], [0, 0, -0.15], [-0.85, 0, 0], [0, 0, -0.15], [0, 0, 0.11], [0, -0.3, 0]]
        elif self.robot == "kuka":
            offsets = [[0.425, 0, 0], [0, 0, -0.075], [0, 0.625, 0], [0, 0, -0.20], [0, 0, 0.05], [-0.25, 0, 0]]

        end_point = start_point
        for offset in offsets:
            end_point = end_point + offset
            self.end_points.append(end_point)

    def get_js(self, msg):
        """
        Callback for joint state msg
        :param msg: message with joint states
        :type msg: sensor_msgs/JointState
        :return:
        :rtype:
        """
        self.js = msg.position[:self.num_link]
        self.js_vel = msg.velocity[:self.num_link]

    def compute_velocity(self, kdl_handle, axis, local=True):
        """

        :param kdl_handle: pointer to instance of kdl handle class that encapsulates kdl function for given chain
        :type kdl_handle: KDLHandle or None
        :param axis: if local = True: axis in the local frame of the end-effector which we should follow;
                                      its will be base for the translation velocity
                     if local = False: the direction of movement in base_link frame
        :type axis: list 1x3
        :param local: whether the axis is expressed on local (end-effector) or base frame
        :type local: bool
        :return:
        :rtype:
        """

        if local:
            # Get current rotation of the pad and compute its negative normal
            _, rot = self.get_transformation(kdl_handle.end_effector, self.base_link)

            direction = np.matmul(ts.quaternion_matrix(rot), np.hstack((axis, 1)))[
                        :3]  # go against normal of the given link
            direction /= np.linalg.norm(direction)  # normalize
            # Set goal speed to 10 cm/s against the normal
        else:
            direction = np.array(axis)
        if self.phase == 4:
            self.link_velocity = np.min([0.4, self.max_velocity])
        return self.link_velocity*direction

    def move_velocity(self, kdl_handle, velocity):
        """
        Function to compute velocity of joints using Jacobian matrix for given end-effector with given axis of motion
        :param kdl_handle: pointer to instance of kdl handle class that encapsulates kdl function for given chain
        :type kdl_handle: KDLHandle
        :param velocity: translational velocity of the given end-effector
        :return: 1x3 list
        :rtype:
        """
        # create goal for KDL with zero angular velocity
        goal = kdl.Twist(kdl.Vector(*velocity), kdl.Vector(*[0, 0, 0]))

        joints = kdl_handle.np_to_kdl(self.js[:kdl_handle.num_joints])

        # Solve -> the output will be saved directly to velocities
        kdl_handle.vel_solver.CartToJnt(joints, goal, kdl_handle.velocities)
        # Publish velocity commands
        angles = kdl_handle.kdl_to_np(kdl_handle.velocities).tolist()
        if self.robot == "ur":  # Just because our real UR is broken and wrist 2 cannot move otherwise the robot dies
            angles[-2] = 0
        self.velocity_msg.data = angles + [0] * (self.num_link - kdl_handle.num_joints)  # 0.1 rad/s in joint_0
        self.velocity_publisher.publish(self.velocity_msg)

    def is_close(self):
        """
        Checks whether the end-effector is close to the desired position
        :return:
        :rtype:
        """
        if self.robot == "kuka":
            position, _ = self.get_transformation("iiwa_link_7", "iiwa_link_0")
        else:
            position, _ = self.get_transformation("wrist_1_link", "base_link")

        if self.phase == -1:
            return False
        dist = np.abs(position[self.axes[self.phase]] - self.end_points[self.phase][self.axes[self.phase]])
        self.link_velocity = np.max([self.max_velocity * np.min([dist*25, 1]), self.min_velocity])
        return dist < 0.0075

    def stop(self, slow=False):
        self.move_velocity(self.kdl_handles["gripper"], [0, 0, 0])

    def run(self):
        """
        Main function of the class
        :return:
        :rtype:
        """
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.status_pub.publish(Int32(data=self.phase))

            # airskin signal
            if self.phase == -2:
                self.stop()
                start_time = rospy.Time.now()
                while rospy.Time.now() - start_time < rospy.Duration(self.stop_duration):
                    self.rate.sleep()

                qs_pad = 9 if self.robot == "ur" else 5
                if self.phase_when_impact == 4 and qs_pad in self.last_pain:
                    if self.robot == "kuka" and self.setup == "sim":
                        if "fmd" not in self.collision_objects:
                            continue
                    self.phase = 5
                    self.phase_when_impact = 5

            # phase to return back to path
            elif self.phase == -1:
                self.phase = self.phase_when_impact
            # go home
            elif self.phase == 0 or self.phase == 7:
                if self.setup == "sim" and self.robot == "ur":
                    self.mg.close_gripper()

                self.switch_controllers(velocity=False)

                for i in range(5): # stupid fix to assure that the robot is in the initial position
                    rospy.sleep(0.1)
                    self.mg.go_to_joint_position(self.INIT_POSE)
                self.switch_controllers(velocity=True)
                if self.phase == 0:
                    self.phase = 1
                else:
                    self.stop()
                    break
            # forward movement
            elif self.phase == 1:
                self.current_velocity = self.compute_velocity(None, self.forward, False)
                self.move_velocity(self.kdl_handles["gripper"], self.current_velocity)
                if self.is_close():
                    self.phase = 2
            # down
            elif self.phase == 4:
                self.current_velocity = self.compute_velocity(None, [0, 0, -1], False)
                self.move_velocity(self.kdl_handles["gripper"], self.current_velocity)
                if self.is_close():
                    self.phase = 5
            # down a little bit
            elif self.phase == 2:
                self.current_velocity = self.compute_velocity(None, [0, 0, -1], False)
                self.move_velocity(self.kdl_handles["gripper"], self.current_velocity)
                if self.is_close():
                    self.phase = 3
            elif self.phase == 6:
                self.current_velocity = self.compute_velocity(None, -self.forward, False)
                self.move_velocity(self.kdl_handles["gripper"], self.current_velocity)
                if self.is_close():
                    self.status_pub.publish(Int32(data=99))
                    self.phase = 7
            # up
            elif self.phase == 5:
                self.current_velocity = self.compute_velocity(None, [0, 0, 1], False)
                self.current_velocity *= 0.5
                self.move_velocity(self.kdl_handles["gripper"], self.current_velocity)
                if self.is_close():
                    self.phase = 6

            # left
            elif self.phase == 3:

                self.current_velocity = self.compute_velocity(None, self.left, False)
                self.move_velocity(self.kdl_handles["gripper"], self.current_velocity)
                if self.is_close():

                    self.phase = 4

            if rospy.Time.now() - start_time > rospy.Duration(300):
                break
            self.rate.sleep()

    def get_transformation(self, what, where):
        """
        Help util to get transformation
        @param what: For what frame to obtain the transformation
        @type what: string
        @param where: In which frame to express
        @type where: string
        @return:
        @rtype:
        """
        translation, rotation = self.listener.lookupTransform(where, what, rospy.Time(0))
        return np.array(translation), np.array(rotation)

    def switch_controllers(self, velocity=True):
        """
        Function to call service to switch robot's controllers

        Right now, the function does not check whether the desired controller is already running -> it can produce
        error message in ROS output and return False
        :param velocity: whether to switch to velocity control
        :type velocity: bool
        :return: success status
        :rtype: bool
        """

        req = SwitchControllerRequest()
        if velocity:
            req.start_controllers = ["joint_group_vel_controller"]
            if self.robot == "ur":
                req.stop_controllers = ["scaled_pos_joint_traj_controller"]
            else:
                req.stop_controllers = ["PositionJointInterface_trajectory_controller"]
        else:
            if self.robot == "ur":
                req.start_controllers = ["scaled_pos_joint_traj_controller"]
            else:
                req.start_controllers = ["PositionJointInterface_trajectory_controller"]
            req.stop_controllers = ["joint_group_vel_controller"]
        req.strictness = 2
        req.start_asap = False
        return self.switch_srv.call(req)


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
        help="Which setup to use: sim or real"
    )

    arg_parser.add_argument(
        "--robot",
        "-r",
        dest="robot",
        required=False,
        default="ur",
        help="Which robot to use: ur or kuka"
    )

    arg_parser.add_argument(
        "--save_bag",
        "-b",
        dest="save_bag",
        action="store_true",
        required=False,
        default=False,
        help="Whether to save the bag file"
    )

    arg_parser.add_argument(
        "--bag_name",
        "-bn",
        dest="bag_name",
        required=False,
        default=None,
        help="Name of the bag to save"
    )

    arg_parser.add_argument(
        "--velocity",
        "-v",
        dest="velocity",
        required=False,
        default=0.6,
        help="Velocities to test"
    )

    args = arg_parser.parse_args()
    return args.setup, args.save_bag, args.bag_name, float(args.velocity), args.robot


if __name__ == "__main__":
    setup, save_bag, bag_name, velocity, robot = prepare_parser()
    e = Experiment(robot, setup, save_bag, bag_name, velocity)
    e.run()
