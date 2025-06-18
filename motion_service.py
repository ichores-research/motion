# ROS Service for motion planning
# Exposes a simple API for motion templates such as "pick", "place", "move", etc.
import argparse
import rospy
import moveit_commander
from motion_msgs.srv import Prepare, PrepareRequest, PrepareResponse, Pick, PickRequest, PickResponse

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_msgs
import trajectory_msgs
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs

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


class MotionService:
    def __init__(self, group_name="arm_torso"):
        self.group_name = group_name
        rospy.loginfo("Initializing Motion Service...")
        rospy.init_node("motion_service_node")

        # Setup head controller
        self.head_cmd = rospy.Publisher(
            "/head_controller/command", JointTrajectory, queue_size=1
        )

        # Setup MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Change end effector link
        self.move_group.set_end_effector_link("gripper_link")
        # self.move_group.allow_replanning(True)
        # self.move_group.set_planning_time(30)
        # self.move_group.set_num_planning_attempts(3)

        # Setup service
        self.prepare_service = rospy.Service(
            "/motion/prepare", Prepare, self.prepare_robot
        )

        self.pick_service = rospy.Service(
            "/motion/pick", Pick, self.pick
        )

        self.marker_publisher = rospy.Publisher(
            "/motion/grasp_markers",
            MarkerArray,
            queue_size=1
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # Dummy table
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose = Pose(
            position=Point(1.2, 0.0, 0.32),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        self.scene.add_box(
            "table",
            table_pose,
            size=(1.5, 1.5, 0.75),
        )

        rospy.loginfo("Done initializing Motion Service.")


    def _move_to_joint_positions(self, joint_positions):
        """
        Moves the robot to the specified joint positions.

        :param joint_names: List of joint names.
        :param joint_positions: List of joint position values.
        """
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


    def prepare_robot(self, req: PrepareRequest):
        """
        Prepares the robot for operation by moving the torso and arm to a safe position.
        """
        rospy.loginfo("Unfolding arm safely")
        # Move the torso to a safe position
        joint_positions = DEFAULT_POSE_JOINT_POSITIONS[self.group_name]
        self._move_to_joint_positions(joint_positions)
        
        # Lower the head
        self.lower_head()
        rospy.loginfo("Robot prepared.")

        return PrepareResponse()

    def pick(self, req: PickRequest):
        mesh = req.object_mesh
        pose = req.object_pose
        grasps = req.grasps
        rospy.loginfo("Received pick request.")

        # Convert grasps from request into MoveIt format
        moveit_grasps = []
        for grasp in grasps.poses:
            moveit_grasp = moveit_msgs.msg.Grasp()
            moveit_grasp.grasp_pose = grasp

            # Convert Pose to PoseStamped
            grasp_pose_stamped = PoseStamped()
            grasp_pose_stamped.header.frame_id = "base_footprint"
            grasp_pose_stamped.header.stamp = rospy.Time.now()
            grasp_pose_stamped.pose = grasp
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
            
            # Set grasp posture (closed gripper)
            moveit_grasp.grasp_posture = self._get_gripper_posture(0.0)  # Closed position
            
            moveit_grasps.append(moveit_grasp)

        self._visualize_grasps(moveit_grasps)
        rospy.loginfo(f"Publishing {len(moveit_grasps)} grasp markers")

        # Add object to planning scene
        self._add_object_to_scene(mesh, pose)

        # Set support surface if needed
        self.move_group.set_support_surface_name("table")

        # Attempt the pick operation
        success = self.move_group.pick("target_object", moveit_grasps)

        return PickResponse(success=success == 1, message="Pick operation completed." if success == 1 else "Pick operation failed.")

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
            marker_x.ns = f"grasp_{idx}"
            marker_x.id = idx * 3
            marker_x.type = Marker.ARROW
            marker_x.action = Marker.ADD
            marker_x.pose = grasp.grasp_pose.pose
            marker_x.scale.x = 0.1  # Arrow length
            marker_x.scale.y = 0.01  # Arrow width
            marker_x.scale.z = 0.01  # Arrow height
            marker_x.color.r = 1.0
            marker_x.color.a = 1.0
            
            # Arrow for Y axis (rotated 90 degrees around Z)
            marker_y = Marker()
            marker_y.header = grasp.grasp_pose.header
            marker_y.ns = f"grasp_{idx}"
            marker_y.id = idx * 3 + 1
            marker_y.type = Marker.ARROW
            marker_y.action = Marker.ADD
            marker_y.pose = grasp.grasp_pose.pose
            # Rotate the Y arrow 90 degrees around Z
            marker_y.pose.orientation = Quaternion(0, 0, 0.707, 0.707)
            marker_y.scale = marker_x.scale
            marker_y.color.g = 1.0
            marker_y.color.a = 1.0
            
            # Arrow for Z axis (rotated -90 degrees around Y)
            marker_z = Marker()
            marker_z.header = grasp.grasp_pose.header
            marker_z.ns = f"grasp_{idx}"
            marker_z.id = idx * 3 + 2
            marker_z.type = Marker.ARROW
            marker_z.action = Marker.ADD
            marker_z.pose = grasp.grasp_pose.pose
            # Rotate the Z arrow -90 degrees around Y
            marker_z.pose.orientation = Quaternion(0.707, 0, 0, 0.707)
            marker_z.scale = marker_x.scale
            marker_z.color.b = 1.0
            marker_z.color.a = 1.0
            
            marker_array.markers.extend([marker_x, marker_y, marker_z])
        
        self.marker_publisher.publish(marker_array)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Motion Service")
    parser.add_argument(
        "--group-name",
        type=str,
        default="arm_torso",
        help="MoveIt group name to use for motion planning.",
    )
    args = parser.parse_args()

    motion_service = MotionService(group_name=args.group_name)
    rospy.spin()
