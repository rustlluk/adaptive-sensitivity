#!/usr/bin/env python3
import sys
import rospy
from pyKUKA.pykuka import pyKUKA
from bullet_ros_kuka.msg import bullet_to_ros, ros_to_bullet, Collisions, OtherObjects, JointTorque, JointQuantity
import numpy as np
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from bullet_ros_kuka.srv import grip, gripResponse, changeObjPose, changeObjPoseResponse
import tf.transformations as ts
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import os


class BulletROS:
    def __init__(self, bhand):
        rospy.init_node("pyKUKA")
        if bhand:
            self.client = pyKUKA(config="ros.yaml")
        else:
            self.client = pyKUKA(config="ros_buckets.yaml")
        self.R_URDFs = {}
        # UR10e joint names in the correct order
        joints = "iiwa_joint_1;iiwa_joint_2;iiwa_joint_3;iiwa_joint_4;iiwa_joint_5;iiwa_joint_6;iiwa_joint_7"

        # Get the joint ids of the UR10e joints
        self.joint_names = joints.split(";")
        self.joint_ids = []
        for joint in self.joint_names:
            self.joint_ids.append(self.client.find_joint_id(joint)[0])
        self.joint_names = np.array(self.joint_names)

        self.bhand = rospy.get_param("gripper", False)

        if self.bhand:
            self.barret_joint_ids = []
            barret_joints  = ["bh_j11_joint", "bh_j12_joint", "bh_j13_joint", "bh_j21_joint", "bh_j22_joint", "bh_j23_joint", "bh_j32_joint", "bh_j33_joint"]
            for joint in barret_joints:
                self.barret_joint_ids.append(self.client.find_joint_id(joint)[0])
            self.bhand_state = JointState()
            self.bhand_state.name = barret_joints
            self.bhand_state.position = [0] * len(self.barret_joint_ids)
            self.bhand_state.velocity = [0] * len(self.barret_joint_ids)
            self.bhand_state.effort = [0] * len(self.barret_joint_ids)
            self.bhand_state_publisher = rospy.Publisher("/bhand_node/joint_states", JointState, queue_size=1, latch=True)

        # Initialize the message to be published
        self.msg = bullet_to_ros()
        self.msg.positions = [0] * len(self.joint_ids)
        self.msg.velocities = [0] * len(self.joint_ids)
        self.msg.efforts = [0] * len(self.joint_ids)

        # Initialize the grip service
        self.grip_service = rospy.Service("/iiwa/grip", grip, self.grip)

        # Initialize the initial joint angles
        init_pos = []
        for j_name in self.joint_names:
            for joint in self.client.joints:
                if j_name == joint.name:
                    if hasattr(self.client.config.initial_joint_angles, joint.name):
                        init_pos.append(np.deg2rad(getattr(self.client.config.initial_joint_angles, joint.name)))
                    else:
                        init_pos.append(0)
                    break

        # Set the joint names and initial joint angles as parameters
        rospy.set_param("/bullet_ros/joint_names", joints)
        rospy.set_param("/bullet_ros/init_pos", ";".join(map(str, init_pos)))
        self.cmd = None
        self.move_type = 0  # 0 - position, 1 - velocity

        # Initialize the marker publisher for balls in RVIZ
        self.marker_array = MarkerArray()
        self.marker_pub = rospy.Publisher("/other_objects", MarkerArray, queue_size=1)

        # Initialize the publisher for the bullet to ros message
        self.pub = rospy.Publisher("/bullet_ros/read", bullet_to_ros, queue_size=1)

        self.collisions_pub = rospy.Publisher("/bullet_ros/collisions", Collisions, queue_size=1)
        self.other_objects_joints = rospy.Publisher("/bullet_ros/other_objects", OtherObjects, queue_size=1)

        # Initialize the controller service
        self.controller_srv = rospy.ServiceProxy("/iiwa/controller_manager/list_controllers", ListControllers)

        # Initialize the service to change the object pose
        self.free_obj_service = rospy.Service("/bullet_ros/change_obj_pose", changeObjPose, self.change_obj_pose)

        self.ext_tor_pub = rospy.Publisher("/iiwa/state/ExternalJointTorque", JointTorque, queue_size=1)


        # Initialize other variables
        self.fixed_states = None
        self.last_pos_cmd = None

        # Lister to ROS RobotHW interface
        self.sub = rospy.Subscriber("/bullet_ros/write", ros_to_bullet, self.read, queue_size=1)

    def change_obj_pose(self, request):
        """
        Change pose of other than robot object

        :param request: request fot the pose change
        :type request: bullet_ros_kuka.srv.changeObjPoseRequest
        :return:
        :rtype:
        """
        # Check if the object exists
        found = False
        for obj_id, obj_name, _, _, _ in self.client.other_objects:
            if obj_name == request.name.data:
                found = True
                break
        if not found:
            rospy.logerr(f"Object {request.name.data} not found")
            return changeObjPoseResponse(0)

        # Prepare pose based on request
        if len(request.position) != 3:
            pos = [0, 0, 0]
        else:
            pos = request.position
        if len(request.orientation) != 4:
            ori = [0, 0, 0, 1]
        else:
            ori = request.orientation

        # Call bullet to change the pose
        self.client.resetBasePositionAndOrientation(obj_id, pos, ori)
        return changeObjPoseResponse(1)

    def read(self, msg):
        """
        Function to read the message from the RobotHW interface and send it to the bullet

        :param msg: msg with velocity and joint angles
        :type msg: bullet_ros_kuka.msg.ros_to_bullet
        :return:
        :rtype:
        """
        self.cmd = msg

        # Check which controller is running and based on that select how to move the robot (velocity or position)
        controller_info = self.controller_srv.call()
        for c in controller_info.controller:
            if c.name == "joint_group_vel_controller":
                if c.state == "running":
                    if self.move_type == 0:
                        self.last_pos_cmd = self.cmd.cmd
                    self.move_type = 1
                else:
                    if self.move_type == 1:
                        if self.last_pos_cmd is not None and np.allclose(self.last_pos_cmd, self.cmd.cmd):
                            self.move_type = 1
                        else:
                            self.last_pos_cmd = None
                            self.move_type = 0
                    else:
                        self.move_type = 0
                break

        # Main loop - > call bullet movement function, update simulation, read new state, activate airskin and send all
        self.move()
        self.client.update_simulation(None)
        self.read_state()
        self.publish_other_objects()
        self.publish_collision_forces()
        self.pub.publish(self.msg)

    def publish_collision_forces(self):
        msg = Collisions()
        msg_ext = JointTorque()
        msg_ext.header.stamp = rospy.Time.now()
        ext_t = np.zeros((7,))
        links = ["iiwa_link_1", "iiwa_link_2", "iiwa_link_3", "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"]
        for obj_id, obj_name, _, _, _ in self.client.other_objects:
            collisions = self.client.getContactPoints(self.client.robot, obj_id)
            for contact in collisions:
                force = contact[self.client.contactPoints["FORCE"]]
                if force != 0:
                    msg.forces.append(force)
                    msg.obj_names.append(obj_name)
                    for link in self.client.links:
                        if link.robot_link_id == contact[self.client.contactPoints["INDEXA"]]:
                            msg.robot_links.append(link.name)
                            break
                    if link.name in links and np.abs(force) > np.abs(ext_t[links.index(link.name)]):
                        ext_t[links.index(link.name)] = force
                    msg.distances.append(contact[self.client.contactPoints["DISTANCE"]])
        if len(msg.forces) > 0:
            self.collisions_pub.publish(msg)
        msg_ext.torque = JointQuantity(*ext_t)
        self.ext_tor_pub.publish(msg_ext)

    def publish_other_objects(self):
        """
        Function to publish other than robot objects to RVIZ

        :return:
        :rtype:
        """
        marker_id = 0
        other_objs_msg = OtherObjects()
        for obj_id, obj_name, is_fixed, color, mesh in self.client.other_objects:
            # pos, ori = self.client.getBasePositionAndOrientation(obj_id)

            for j in range(self.client.getNumJoints(obj_id)):
                other_objs_msg.object.append(obj_name)
                other_objs_msg.joint.append(self.client.getJointInfo(obj_id, j)[1].decode("utf-8"))
                other_objs_msg.joint_position.append(self.client.getJointState(obj_id, j)[0])


            visualData = self.client.getVisualShapeData(obj_id)

            for m in visualData:
                # Get information about individual parts of the object
                f_path = m[self.client.visualShapeData["FILE"]].decode("utf-8")

                if f_path not in self.R_URDFs:
                    # URDF rotations and translations
                    init_xyz, init_rpy, scale, link_name = self.client.find_xyz_rpy(os.path.basename(f_path),
                                                                                    urdf_name=obj_name)

                    R_urdf = np.eye(4)
                    R_urdf[:3, :3] = np.reshape(
                        self.client.getMatrixFromQuaternion(self.client.getQuaternionFromEuler(init_rpy)), (3, 3))
                    R_urdf[:3, 3] = init_xyz
                    self.R_URDFs[f_path] = R_urdf
                else:
                    R_urdf = self.R_URDFs[f_path]

                if self.client.config.show_collision:
                    f_path = f_path.replace("visual", "collision").replace(".obj", "_vhacd.obj")

                col = m[self.client.visualShapeData["COLOR"]]
                link = m[self.client.visualShapeData["LINK"]]
                link_name = os.path.basename(m[self.client.visualShapeData["FILE"]].decode("utf-8")).split(".")[0]
                # non-base links
                if link != -1:
                    # get link info
                    linkState = self.client.getLinkState(obj_id, link, computeLinkVelocity=0,
                                                         computeForwardKinematics=0)
                    # get orientation and position wrt URDF - better than in world
                    ori = linkState[self.client.linkInfo["URDFORI"]]
                    pos = linkState[self.client.linkInfo["URDFPOS"]]
                # link == -1 is base. For that, getBasePosition... needs to be used. This joint must not contain URDF visual xyz and rpy
                else:
                    pos, ori = self.client.getBasePositionAndOrientation(obj_id)
                # get ori and position as 4x4 transformation matrix
                R = np.eye(4)
                R[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(ori), (3, 3))
                R[:3, 3] = pos

                R = R @ R_urdf
                if not self.client.config.show_collision:
                    f_path = f_path.replace("collision", "visual").replace("_vhacd.obj", ".obj")

                if marker_id >= len(self.marker_array.markers):
                    marker = Marker()
                    marker.id = marker_id
                    marker_id += 1
                    marker.ns = 'mesh_marker'
                    marker.header.frame_id = "iiwa_link_0"
                    marker.action = marker.ADD
                    marker.scale.x = scale
                    marker.scale.y = scale
                    marker.scale.z = scale
                    marker.color.a = 1.0
                    marker.color.r = col[0]
                    marker.color.g = col[1]
                    marker.color.b = col[2]
                    marker.type = marker.MESH_RESOURCE
                    marker.mesh_resource = "package://bullet_ros_kuka/../other_meshes/" + f_path.split("/other_meshes/")[-1]
                    marker.pose.position = Point(*R[:3, 3])
                    marker.pose.orientation = Quaternion(*ts.quaternion_from_matrix(R))
                    self.marker_array.markers.append(marker)
                else:
                    self.marker_array.markers[marker_id].action = Marker.MODIFY
                    self.marker_array.markers[marker_id].pose.position = Point(*R[:3, 3])
                    self.marker_array.markers[marker_id].pose.orientation = Quaternion(*ts.quaternion_from_matrix(R))
                    marker_id += 1
            other_objs_msg.position += pos
            other_objs_msg.orientation += ori
            other_objs_msg.link.append(link_name)
        self.marker_pub.publish(self.marker_array)
        self.other_objects_joints.publish(other_objs_msg)

    def read_state(self):
        """
        Read current robot state to be sent back to RobotGH interface

        :return:
        :rtype:
        """
        state = self.client.getJointStates(self.client.robot, self.joint_ids)
        for idx, j_state in enumerate(state):
            self.msg.positions[idx] = j_state[0]
            self.msg.velocities[idx] = j_state[1]
            self.msg.efforts[idx] = j_state[3]

        if self.bhand:
            state = self.client.getJointStates(self.client.robot, self.barret_joint_ids)
            for idx, j_state in enumerate(state):
                self.bhand_state.position[idx] = j_state[0]
                self.bhand_state.velocity[idx] = j_state[1]
                self.bhand_state.effort[idx] = j_state[3]
            self.bhand_state_publisher.publish(self.bhand_state)

    def move(self):
        """
        Based on move_type (velocity or position) move the robot in the given pose or fix it in place

        :return:
        :rtype:
        """
        if self.cmd is None:
            return 0
        if self.move_type == 0:  # Position = just send directly
            self.client.move_position(self.joint_names, self.cmd.cmd, wait=False)

        elif self.move_type == 1:
            # if all velocity command are not 0 send them to the robot
            if not np.allclose(self.cmd.vel_cmd, np.zeros(len(self.cmd.vel_cmd))):
                self.fixed_states = None
                self.client.move_velocity(self.joint_names, np.array(self.cmd.vel_cmd))
            else: # all velocities 0
                if self.fixed_states is None:  # we do not know where to keep the robot
                    state = self.client.getJointStates(self.client.robot, self.joint_ids)
                    pos = np.zeros(len(self.joint_ids))
                    for idx, j_state in enumerate(state):
                        pos[idx] = j_state[0]
                    self.fixed_states = pos
                # Keep the robot on place
                self.client.move_position(self.joint_names, self.fixed_states, wait=False)

    def grip(self, request):
        """
        Grip request callback

        :param request: request to close/open the gripper
        :type request: bullet_ros_kuka/srv/gripRequest
        :return:
        :rtype:
        """
        if not (0 <= request.width <= 1):
            rospy.logerr("Width must be between 0-1")
            out = gripResponse()
            out.success = False
            return out

        if not (0 <= request.velocity <= 1):
            rospy.logerr("Velocity must be between 0-1")
            out = gripResponse()
            out.success = False
            return out
        if request.velocity == 0:
            request.velocity = 1

        vel_min = 0
        vel_max = 2
        velocity = ((request.velocity - 0) / (1 - 0)) * (vel_max -vel_min) + vel_min
        self.client.gripper.set_gripper_pose(request.width, wait=True, pose=request.pose, velocity=velocity)  # This needs to be fixed

        while not rospy.is_shutdown() and not self.client.motion_done():
            rospy.sleep(1/250)

        out = gripResponse()
        out.success = True
        return out


if __name__ == "__main__":
    print(sys.argv[1])
    bhand = True if sys.argv[1]=="true" else False
    c = BulletROS(bhand)
    rospy.spin()
