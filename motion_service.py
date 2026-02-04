#!/usr/bin/env python3

# ROS Service for motion planning
# Exposes a simple API for motion templates such as "pick", "place", "move", etc.
import argparse
import subprocess

import numpy as np

import moveit_commander
import moveit_msgs
import rosnode
import rospy
import trajectory_msgs
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3Stamped
from motion_msgs.srv import (Pick, PickRequest, PickResponse, Prepare,
                             PrepareRequest, PrepareResponse, DetectWorkspace, DetectWorkspaceRequest, DetectWorkspaceResponse)

from motion_msgs.srv import Pick as Place  # added for code understandability, so Place requests are not misunderstood as Pick
from motion_msgs.srv import PickRequest as PlaceRequest # added for code understandability
from motion_msgs.srv import PickResponse as PlaceResponse # added for code understandability

from motion_msgs.srv import Pick as ToPose  # added for code understandability, so ToPose requests are not misunderstood as Pick
from motion_msgs.srv import PickRequest as ToPoseRequest # added for code understandability
from motion_msgs.srv import PickResponse as ToPoseResponse # added for code understandability

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, EmptyRequest, EmptyResponse, Empty
from sensor_msgs.msg import PointCloud2, JointState
from table_plane_extractor_msgs.srv import TablePlaneExtractor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
import threading
from tf.transformations import quaternion_from_euler
import math
import cv2
from teleop_tools_msgs.msg import IncrementActionGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import copy

DEFAULT_POSE_JOINT_POSITIONS = {
    "arm_torso": [
        0.34, # torso_lift_joint
        0.10, # arm_1_joint
        0.47, # arm_2_joint
        -0.20, # arm_3_joint
        1.56, # arm_4_joint
        1.60, # arm_5_joint
        0.25, # arm_6_joint
        1.19 # arm_7_joint
    ],
"arm_right_torso": [
        0.34, # torso_lift_joint
        -0.48, # arm_1_joint
        -0.25, # arm_2_joint
        1.84, # arm_3_joint
        1.76, # arm_4_joint
        1.60, # arm_5_joint
        -0.50, # arm_6_joint
        1.19 # arm_7_joint
    ]
}

DEFAULT_HEAD_JOINT_POSITIONS = [
    0.0, # head_1_joint
    -0.75 # head_2_joint
]

# Whether to enlarge the table bounding box to the floor level
ENLARGE_TABLE_BB_TO_FLOOR = True  


class MotionService:
    def __init__(self, group_name="arm_torso"):
        self.group_name = group_name
        rospy.loginfo("Initializing Motion Service...")
        rospy.init_node("motion_service_node")
        rospy.on_shutdown(self.on_ros_shutdown)
        self.move_group_lock = threading.Lock()
        self.gripper_image_sub = rospy.Subscriber("/gripper_camera/rgb/image_raw", Image, self.gripper_image_cb)
        self.cv2ros_converter = CvBridge()
        # Setup head controller
        self.head_cmd = rospy.Publisher(
            "/head_controller/command", JointTrajectory, queue_size=1
        )
        self.gripper_mover = rospy.Publisher("/gripper_controller/increment/goal", IncrementActionGoal, queue_size=10)
        self.torso_mover = rospy.Publisher("/torso_controller/increment/goal", IncrementActionGoal, queue_size=10)

        # Setup MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.finger_group = moveit_commander.MoveGroupCommander("gripper")

        # Change end effector link
        self.move_group.set_end_effector_link("gripper_link")
        #self.move_group.set_end_effector_link("gripper_fingertips_frame")
        #self.move_group.set_goal_tolerance(0.01)
        #self.move_group.set_planner_id("RRTConnectkConfigDefault")
        # self.move_group.allow_replanning(True) # TODO: Testing whether allowing to replan makes a difference
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        # self.move_group.set_start_state_to_current_state()

        #IF OCTOMAP IS ENABLED

        # try:
        #     # Actual update rate control
        #     rospy.set_param("/move_group/sensors", ["sensor_plugin/point_cloud"])
        #     rospy.set_param("/move_group/sensor_plugin/point_cloud/sensor_plugin", "occupancy_map_monitor/PointCloudOctomapUpdater")
            
        #     # THIS is update frequency - how often octomap processes new data (seconds between updates)
        #     rospy.set_param("/move_group/sensor_plugin/point_cloud/point_subsample", 1)
        #     rospy.set_param("/move_group/max_range", 1.0)
        #     rospy.set_param("/move_group/sensor_plugin/point_cloud/padding_offset", 0.1)
        #     rospy.set_param("/move_group/sensor_plugin/point_cloud/padding_scale", 1.0)
            
        #     # Most important - point cloud decimation/subsampling
        #     rospy.set_param("/move_group/sensor_plugin/point_cloud/point_subsample", 3)  # Process every 3rd point
        #     rospy.set_param("/move_group/sensor_plugin/point_cloud/max_update_rate", 1.0)  # Update max 1Hz instead of continuous
            
        #     rospy.loginfo("Throttled octomap update rate")
        # except Exception as e:
        #     rospy.logwarn(f"Could not throttle octomap: {e}")


        # TODO: Try checking whether the allowed tolerance makes a difference
        try:
            rospy.set_param("/move_group/trajectory_execution/allowed_start_tolerance", 0.05)
            rospy.loginfo("Set the allowed start tolerance to 0.05")
        except Exception as e:
            rospy.logwarn(f"Could not set allowed allowed start tolerance: {e}")

        # Set up table detection
        self.depth_topic = "xtion/depth_registered/points"
        table_extractor_service = "/table_plane_extractor/get_planes"
        self.table_extractor = rospy.ServiceProxy(table_extractor_service, TablePlaneExtractor)
        rospy.loginfo('Waiting for table plane extractor service.')
        rospy.wait_for_service(table_extractor_service)
        rospy.loginfo('Service available.')

        # Setup exposed service
        self.prepare_service = rospy.Service(
            "/motion/prepare", Prepare, self.prepare_robot
        )

        self.pick_service = rospy.Service(
            "/motion/pick", Pick, self.pick_with_vision_closure #self.pick
        )

        self.place_service = rospy.Service(
            "/motion/place", Place, self.place
        )

        self.to_pose_service = rospy.Service(
            "/motion/to_pose", ToPose, self.to_pose_callback
        )

        self.detect_workspace_service = rospy.Service(
            "/motion/detect_workspace", DetectWorkspace, self.detect_workspace
        )

        self.reset_planning_scene_service = rospy.Service("/motion/reset_planning_scene", SetBool, self.planning_scene_reset)

        self.marker_publisher = rospy.Publisher(
            "/motion/grasp_markers",
            MarkerArray,
            latch=True
        )

        rospy.loginfo("Done initializing Motion Service.")

    def _move_to_joint_positions(self, joint_positions):
        """
        Moves the robot to the specified joint positions.

        :param joint_names: List of joint names.
        :param joint_positions: List of joint position values.
        """
        # with self.move_group_lock:        
        if 1: 
            rospy.loginfo("Moving to joint positions...")
            self.move_group.set_joint_value_target(joint_positions)
            self.move_group.go(wait=True)
            self.move_group.stop()

    def lower_head(self):
        """
        Lowers the head of the robot to a predefined position.
        """
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = [
            "head_1_joint",
            "head_2_joint",
        ]
        jtp = JointTrajectoryPoint()
        jtp.positions = DEFAULT_HEAD_JOINT_POSITIONS
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def move_to_pose (self, pose: Pose):
        try: 
            self.move_group.set_pose_target(pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return success, f"Success! Arm moved to pose: {pose}" if success else "Failed to reach pose!"
        except Exception as e:
            rospy.logerr(f"{e}")
            return False, f"{e}"
                
    def to_pose_callback(self,req):
        res, msg = self.move_to_pose(req.object_pose)
        return ToPoseResponse(success=res, message=msg)

    def prepare_robot(self, req):
        """
        Prepares the robot for operation by moving the torso and arm to a safe position.
        """
        rospy.loginfo("Detecting workspace...")
        self._detect_workspace()  # Ensure workspace is detected before picking
        rospy.loginfo("Unfolding arm safely...")
        # Move the torso to a safe position
        joint_positions = DEFAULT_POSE_JOINT_POSITIONS[self.group_name]
        self._move_to_joint_positions(joint_positions)
        self.move_gripper(0.04)
        
        # Lower the head
        self.lower_head()
        rospy.loginfo("Robot prepared.")

        return PrepareResponse()

    def _cleanup_target_object(self):
        try:
            attached_objects = self.scene.get_attached_objects(["target_object"])

            if "target_object" in attached_objects:
                rospy.loginfo("Detaching target_object from gripper...")
                try:
                    self.move_group.detach_object("target_object")
                except Exception as e:
                    rospy.logerr(f"{e}") 
            
            if "target_object" in self.scene.get_known_object_names():
                rospy.loginfo("Removing target object from world...")
                self.scene.remove_world_object("target_object")

            rospy.loginfo("Target object cleanup complete.")

        except Exception as e:
            rospy.logwarn(f"Error during target object cleanup: {e}")

    def planning_scene_reset(self, req):
        # with self.move_group_lock:
        try:
            try:                
                self.move_group.detach_object("target_object")
            except Exception as e:
                rospy.logerr(f"{e}") 
                pass
            try:
                self.scene.remove_world_object("target_object")
            except Exception as e:
                rospy.logerr(f"{e}") 
                pass
            rospy.loginfo("Removed world object...")

            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.move_group.clear_path_constraints()
            # self.move_group.set_start_state_to_current_state()
            if rospy.get_param("/move_group/trajectory_execution/allowed_start_tolerance")<0.05:
                rospy.set_param("/move_group/trajectory_execution/allowed_start_tolerance", 0.05)

            return SetBoolResponse(success=True, message="Finished cleaning the planning scene") 
        
        except Exception as e:
            return SetBoolResponse(success=False, message=f"Error{e}")

    def pick_with_vision_closure(self, req):
        with self.move_group_lock:
            mesh = req.object_mesh
            pose = req.object_pose
            grasps = req.grasps
            rospy.loginfo("Processing pick request...")

            try:
                moveit_grasps = []
                for grasp in grasps.poses:
                    moveit_grasp = moveit_msgs.msg.Grasp()
                    moveit_grasp.grasp_pose = grasp

                    # Convert Pose to PoseStamped
                    grasp_pose_stamped = PoseStamped()
                    grasp_pose_stamped.header.frame_id = "base_footprint"
                    grasp_pose_stamped.header.stamp = rospy.Time.now()
                    grasp_pose_stamped.pose = grasp
                    print(f"\n\n\n\n\ngrasp pose translation: {grasp.position.x}, {grasp.position.y}, {grasp.position.z} \n\n grasp_pose quaternion {grasp.orientation.x}, {grasp.orientation.y}, {grasp.orientation.z}, {grasp.orientation.w}, \n\n\n\n\n")
                    moveit_grasp.grasp_pose = grasp_pose_stamped

                    # Set pre-grasp approach
                    moveit_grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint"
                    moveit_grasp.pre_grasp_approach.direction.vector.z = -1.0  # Approach from above
                    moveit_grasp.pre_grasp_approach.min_distance = 0.095
                    moveit_grasp.pre_grasp_approach.desired_distance = 0.115

                    # Set post-grasp retreat
                    moveit_grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint"
                    moveit_grasp.post_grasp_retreat.direction.vector.z = 1.0  # Retreat upwards
                    moveit_grasp.post_grasp_retreat.min_distance = 0.0
                    moveit_grasp.post_grasp_retreat.desired_distance = 0.0

                    # Set pre-grasp posture (open gripper)
                    moveit_grasp.pre_grasp_posture = self._get_gripper_posture(0.04)  # Open position
                    # Set grasp posture (closed gripper)
                    moveit_grasp.grasp_posture = self._get_gripper_posture(0.039)  # Closed position

                    # Set maximum contact force
                    # moveit_grasp.max_contact_force = 0.1
                    
                    moveit_grasps.append(moveit_grasp)

                self._visualize_grasps(moveit_grasps)
                rospy.loginfo("Publishing {} grasp markers".format(len(moveit_grasps)))

                # Add object to planning scene
                self._add_object_to_scene(mesh, pose)

                # Set support surface if needed
                self.move_group.set_support_surface_name("table")

                # Attempt the pick operation
                success = self.move_group.pick("target_object", moveit_grasps)
                rospy.sleep(0.2)

                if not success:
                    try:                
                        self.move_group.detach_object("target_object")
                    except Exception as e:
                        rospy.logerr(f"{e}") 
                        
                    try:
                        self.scene.remove_world_object("target_object")
                    except Exception as e:
                        rospy.logerr(f"{e}") 
                        
                    rospy.loginfo("Removed world object...")
                
                self.vision_close_gripper()
                joint_positions = None
                try:
                    msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
                    joint_positions = list(msg.position)
                    joint_positions = [joint_positions[11]+0.2]+joint_positions[:7]

                    rospy.loginfo(f"Got message: {msg}")
                except rospy.ROSException:
                    rospy.logerr("Fetching joint state timeout!")
                if joint_positions: 
                    if joint_positions[0]<=0.34:
                        self._move_to_joint_positions(joint_positions)
                    else:
                        current_pose = pose
                        current_pose.position.z += 0.2
                        self.move_group.set_pose_target(current_pose)
                        self.move_group.go(wait=True)
                        self.move_group.stop()
                        self.move_group.clear_pose_targets()

                return PickResponse(success=success == 1, message="Pick operation completed." if success == 1 else "Pick operation failed.")

            except Exception as e: 
                rospy.logerr(f"Pick operation exception: {e}")
                # Critical: clean up on failure
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                try:
                    self.scene.remove_world_object("target_object")
                except Exception as e:
                    rospy.logerr(f"{e}")
                
                return PickResponse(success=False, message=str(e))

    def pick(self, req):
        with self.move_group_lock:
            mesh = req.object_mesh
            pose = req.object_pose
            grasps = req.grasps
            rospy.loginfo("Processing pick request...")

            try:
                moveit_grasps = []
                for grasp in grasps.poses:
                    moveit_grasp = moveit_msgs.msg.Grasp()
                    moveit_grasp.grasp_pose = grasp

                    # Convert Pose to PoseStamped
                    grasp_pose_stamped = PoseStamped()
                    grasp_pose_stamped.header.frame_id = "base_footprint"
                    grasp_pose_stamped.header.stamp = rospy.Time.now()
                    grasp_pose_stamped.pose = grasp
                    print(f"\n\n\n\n\ngrasp pose translation: {grasp.position.x}, {grasp.position.y}, {grasp.position.z} \n\n grasp_pose quaternion {grasp.orientation.x}, {grasp.orientation.y}, {grasp.orientation.z}, {grasp.orientation.w}, \n\n\n\n\n")
                    moveit_grasp.grasp_pose = grasp_pose_stamped

                    # Set pre-grasp approach
                    moveit_grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint"
                    moveit_grasp.pre_grasp_approach.direction.vector.z = -1.0  # Approach from above
                    moveit_grasp.pre_grasp_approach.min_distance = 0.095
                    moveit_grasp.pre_grasp_approach.desired_distance = 0.115

                    # Set post-grasp retreat
                    moveit_grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint"
                    moveit_grasp.post_grasp_retreat.direction.vector.z = 1.0  # Retreat upwards
                    moveit_grasp.post_grasp_retreat.min_distance = 0.1
                    moveit_grasp.post_grasp_retreat.desired_distance = 0.25

                    # Set pre-grasp posture (open gripper)
                    moveit_grasp.pre_grasp_posture = self._get_gripper_posture(0.04)  # Open position
                    
                    closed_rgipper_joint = rospy.get_param("/motion/closed_gripper_joint")
                    # Set grasp posture (closed gripper)
                    moveit_grasp.grasp_posture = self._get_gripper_posture(closed_rgipper_joint)  # Closed position

                    # Set maximum contact force
                    # moveit_grasp.max_contact_force = 0.1
                    
                    moveit_grasps.append(moveit_grasp)

                self._visualize_grasps(moveit_grasps)
                rospy.loginfo("Publishing {} grasp markers".format(len(moveit_grasps)))

                # Add object to planning scene
                self._add_object_to_scene(mesh, pose)

                # Set support surface if needed
                self.move_group.set_support_surface_name("table")

                # Attempt the pick operation
                success = self.move_group.pick("target_object", moveit_grasps)
                rospy.sleep(0.2)

                if not success:
                    try:                
                        self.move_group.detach_object("target_object")
                    except Exception as e:
                        rospy.logerr(f"{e}") 
                        
                    try:
                        self.scene.remove_world_object("target_object")
                    except Exception as e:
                        rospy.logerr(f"{e}") 
                        
                    rospy.loginfo("Removed world object...")

                return PickResponse(success=success == 1, message="Pick operation completed." if success == 1 else "Pick operation failed.")

            except Exception as e: 
                rospy.logerr(f"Pick operation exception: {e}")
                # Critical: clean up on failure
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                try:
                    self.scene.remove_world_object("target_object")
                except Exception as e:
                    rospy.logerr(f"{e}")
                
                return PickResponse(success=False, message=str(e))                

    def place(self, req):
        place_success = False
        place_message = ""
        try: 
            # Create a list with one PlaceLocation
            placement_pose = req.object_pose
            self.move_group.set_support_surface_name("table")
            place_location = moveit_msgs.msg.PlaceLocation()
            
            # Set the frame and pose for place location
            place_location.place_pose.header.frame_id = "base_footprint"
            
            # Set orientation using RPY (0, 0, pi/2)

            current_pose = self.move_group.get_current_pose().pose
            place_location.place_pose.pose.orientation = current_pose.orientation

            place_location.place_pose.pose.position = placement_pose.position
            place_location.place_pose.pose.position.z += 0.02

            # Pre-place approach - defined with respect to frame_id
            place_location.pre_place_approach.direction.header.frame_id = "base_footprint"
            # Direction is set as negative z axis
            place_location.pre_place_approach.direction.vector.z = -1.0
            place_location.pre_place_approach.min_distance = 0.095
            place_location.pre_place_approach.desired_distance = 0.115
            
            # Post-place retreat - defined with respect to frame_id
            place_location.post_place_retreat.direction.header.frame_id = "base_footprint"
            # Direction is set as negative y axis
            place_location.post_place_retreat.direction.vector.z = 1.0
            place_location.post_place_retreat.min_distance = 0.1
            place_location.post_place_retreat.desired_distance = 0.25
            

            # Set support surface and execute place
            place_success = self.move_group.place("target_object", [place_location])
            # Similar to the pick case - open gripper after placing
            self.move_gripper(0.043)

        except Exception as e:
            place_message = f"{e}"
        
        return PlaceResponse(success=place_success, message=place_message)                

    def move_gripper(self, gripper_joint_position):
        """
        Moves the robot gripper fingers the specified joint positions.

        :param joint_positions: joint position value for gripper links, assumes symetric motion.
        """
        # with self.move_group_lock:        
        if 1: 
            rospy.loginfo("Moving to joint positions...")
            self.finger_group.set_joint_value_target([gripper_joint_position]*2)
            self.finger_group.go(wait=True)
            self.finger_group.stop()

    def gripper_image_cb(self,msg):
        self.gripper_image = msg

    def get_mask(self, image, lower, upper):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, lower, upper, cv2.THRESH_BINARY)
        return binary_image

    def is_grabbed(self):
        img = self.cv2ros_converter.imgmsg_to_cv2(self.gripper_image, desired_encoding='passthrough') 
        binary_mask = self.get_mask(img,190,255)
        kernel = np.ones((10,10), np.uint8)
        img_erosion = cv2.dilate(binary_mask, kernel, iterations=1)
        cv2.imwrite('mask_eroded.jpg', img_erosion)
        mask = cv2.morphologyEx(img_erosion, cv2.MORPH_CLOSE, kernel)
        mask = cv2.bitwise_not(mask)
        num_labels, labels = cv2.connectedComponents(mask)
        blob_count = num_labels - 1
        return blob_count == 1

    def vision_close_gripper(self, gripper_joint_position=-0.01):
        """
        """
        success = False
        r = rospy.Rate(10)
        t0 = rospy.get_time()
        while not success and 2.0>rospy.get_time()-t0:
            new_command = IncrementActionGoal()
            new_command.goal.increment_by = [gripper_joint_position, gripper_joint_position]
            self.gripper_mover.publish(new_command)
            rospy.loginfo(success)
            success = self.is_grabbed()
            rospy.loginfo(success)
            r.sleep()        
        
    def detect_workspace(self, req):
        """
        Detects the workspace by identifying the table and floor planes.
        """
        rospy.loginfo("Detecting workspace...")
        self._detect_workspace()
        rospy.loginfo("Workspace detection completed.")

        return DetectWorkspaceResponse()

    def _get_gripper_posture(self, position):
        """Helper method to generate gripper posture"""
        posture = trajectory_msgs.msg.JointTrajectory()
        posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = rospy.Duration(0.5)
        posture.points.append(point)
        return posture

    def _add_object_to_scene(self, mesh, pose):
        """Helper method to add object to planning scene"""
        collision_object = moveit_msgs.msg.CollisionObject()
        collision_object.header.frame_id = "base_footprint"
        collision_object.id = "target_object"
        
        # Add mesh to collision object
        collision_object.meshes.append(mesh)
        collision_object.mesh_poses.append(pose)
        collision_object.operation = collision_object.ADD

        # Add object to planning scene
        self.scene.add_object(collision_object)

    def _visualize_grasps(self, grasps):
        """Visualize grasp poses as coordinate frames in RViz"""
        marker_array = MarkerArray()
        
        for idx, grasp in enumerate(grasps):
            # Arrow for X axis
            marker_x = Marker()
            marker_x.header = grasp.grasp_pose.header
            marker_x.ns = "grasp_{}".format(idx)
            marker_x.id = idx * 3
            marker_x.type = Marker.ARROW
            marker_x.action = Marker.ADD
            marker_x.pose = grasp.grasp_pose.pose
            marker_x.scale.x = 0.1  # Arrow length
            marker_x.scale.y = 0.01  # Arrow width
            marker_x.scale.z = 0.01  # Arrow height
            marker_x.color.r = 1.0
            marker_x.color.a = 1.0
            marker_array.markers.append(marker_x)
        
        self.marker_publisher.publish(marker_array)

    def _detect_workspace(self):
        """
        Detects the table using the table plane extractor service.
        Returns a list of detected planes.
        """

        try:
            cloud = rospy.wait_for_message(self.depth_topic, PointCloud2, timeout=5)
        except rospy.ROSException as e:
            rospy.logerr("Timeout while waiting for point cloud message: {}".format(e))
            return

        try:
            response = self.table_extractor(cloud)
            boxes = response.plane_bounding_boxes
            rospy.loginfo("Found {} tables".format(len(boxes)))
        except rospy.ServiceException as e:
            rospy.logerr("Table plane extractor service call failed: {}".format(e))
            return
        except Exception as e:
            rospy.logerr("Unexpected error during table detection: {}".format(e))
            return

        largest_box = None

        for box in boxes:
            if largest_box is None or box.size.x * box.size.y > largest_box.size.x * largest_box.size.y:
                largest_box = box

        if largest_box is None:
            rospy.logwarn("No table detected.")
        else:
            table_pose = PoseStamped()
            table_pose.header.frame_id = 'base_footprint'
            table_pose.pose.position.x = largest_box.center.position.x
            table_pose.pose.position.y = largest_box.center.position.y
            table_pose.pose.position.z = largest_box.center.position.z

            self.scene.add_box("table", table_pose, (largest_box.size.y, largest_box.size.x, largest_box.size.z))

        # Add floor plane to the scene
        floor_pose = PoseStamped()
        floor_pose.header.frame_id = 'base_footprint'
        floor_pose.pose.position.x = 0.0
        floor_pose.pose.position.y = 0.0
        floor_pose.pose.position.z = 0.0
        self.scene.add_box("floor", floor_pose, (10, 10, 0.01))

        return

    def _reset_move_group(self):
        """Reset move_group to clean state"""
        # with self.move_group_lock:
        if 1:
            try:
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                # Clear all objects from scene
                for name in self.scene.get_known_object_names():
                    if name not in ["table", "floor"]:  # Keep workspace
                        self.scene.remove_world_object(name)
            except Exception as e:
                rospy.logerr(f"Error resetting move_group: {e}")

    def on_ros_shutdown(self):
        rospy.loginfo("ROS shutdown called, cleaning up...")
        try:
            self.move_group.stop()
            self.move_group.clear_pose_targets()
        except Exception:
            pass


def kill_head_manager():
    """
    Kills the head_manager node if it is running.
    This is useful to ensure that the head manager does not interfere with the motion service.
    """
    rospy.loginfo("Killing head_manager node if running...")
    try:
        if "/pal_head_manager" in rosnode.get_node_names():
            subprocess.call(["rosnode", "kill", "/pal_head_manager"])
            rospy.loginfo("pal_head_manager node killed.")
    except Exception as e:
        rospy.logwarn("Could not kill pal_head_manager node: {}".format(e))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Motion Service")
    parser.add_argument(
        "--group-name",
        type=str,
        default="arm_torso",
        help="MoveIt group name to use for motion planning.",
    )
    args = parser.parse_args()
    
    kill_head_manager()
    motion_service = MotionService(group_name=args.group_name)
    rospy.spin()



