import geometry_msgs
import moveit_commander
import rospy
from sensor_msgs.msg import Image as ROSImage
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PIL import Image
import numpy as np
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
import moveit_msgs
# from  v4r_util.tf2 import TF2Wrapper
GROUP_NAME = "arm_torso"

# necessary packages: robokudo_msgs, v4r_util, table_plane_extractor

class PickObject:
    def __init__(self):
        rospy.init_node("pick_object_with_collision_avoidance", anonymous=True)
        rospy.loginfo("pick object with collision avoidance")

        # Setup MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(GROUP_NAME, wait_for_servers=55.0)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # Change end effector link
        end_effector_link = "gripper_link"
        self.move_group.set_end_effector_link(end_effector_link)
        self.move_group.set_planner_id("ESTkConfigDefault")
        self.move_group.allow_replanning(True)
        self.move_group.set_planning_time(30)
        self.move_group.set_num_planning_attempts(100) # TODO: Test whether number of attempts makes a difference here

        # TODO: Testing whether clearing the planning scene makes a difference
        self.scene.clear()

        self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()

        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        # self.tf_wrapper = TF2Wrapper()

    def act(self):
        # Get current gripper pose
        pose = self.move_group.get_current_pose().pose

        # pose.position.x += 0.1

        # matrix = np.array([[-0.58331135, -0.77824814, -0.23254609, -0.18678527],
        #  [-0.04613663, 0.31758274, -0.9471075 , -0.20293074],
        # [0.81093727, -0.54172966, -0.22115539,  0.38104918],  
        # [0.        ,  0.        ,  0.        , 1.        ]])

        # translation = matrix[:3, 3]
        # rotation = np.eye(4)
        # rotation[:3, :3] = matrix[:3, :3]

        # quaternion = tf.transformations.quaternion_from_matrix(rotation)

        # pose = geometry_msgs.msg.PoseStamped()
        # pose.header.frame_id = "xtion_rgb_optical_frame"
        # pose.pose.position.x = translation[0]
        # pose.pose.position.y = translation[1]
        # pose.pose.position.z = translation[2]
        # pose.pose.orientation.x = quaternion[0]
        # pose.pose.orientation.y = quaternion[1]
        # pose.pose.orientation.z = quaternion[2]
        # pose.pose.orientation.w = quaternion[3]


        # pose = self.tf_wrapper.transform_pose("base_link", pose)
        # pose.pose.position.z = 0.7

        # rospy.loginfo(str(pose))

        pose.position.x = 0.8
        pose.position.y = 0
        pose.position.z = 0.8

        # Quat to rpy
        # rot = pose.orientation
        # rpy = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        # rpy = (rpy[0] + adt[3], rpy[1] + adt[4], rpy[2] + adt[5])

        # quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        # pose.orientation = geometry_msgs.msg.Quaternion(*quat)


        self.move_group.set_pose_target(pose)

        # Publish the plan to visualize it in RViz
        # self.wait_for_planning_scene_object()
        #self.wait_for_planning_scene_object("table")

        rospy.loginfo("Planning...")
        (success, trajectory, time, error) = self.move_group.plan()
        rospy.logwarn(str(trajectory))
        rospy.loginfo("Planned.")

        # Vizualize the trajectory

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(trajectory)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
        
        # wait for input
        # input("Press Enter to continue...")

        rospy.loginfo("Executing...")
        self.move_group.execute(trajectory)

        # # `go()` returns a boolean indicating whether the planning and execution was successful.
        # success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        self.grasp(open=False)

        rospy.loginfo(f"Published action success: {success}")

    def grasp(self, open=True):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0] if not open else [0.04, 0.04]
        point.time_from_start = rospy.Duration(1.0)
        joint_trajectory.points.append(point)

        self.gripper_pub.publish(joint_trajectory)

    def spin(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Tiago Controller ready.")
            self.act()
            break


    def wait_for_planning_scene_object(self, object_name='part'):
        rospy.loginfo("Waiting for object '" + object_name + "'' to appear in planning scene...")
        gps_req = GetPlanningSceneRequest()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES

        part_in_scene = False
        while not rospy.is_shutdown() and not part_in_scene:
            # This call takes a while when rgbd sensor is set
            gps_resp = self.scene_srv.call(gps_req)
            # check if 'part' is in the answertable
            for collision_obj in gps_resp.scene.world.collision_objects:
                if collision_obj.id == object_name:
                    part_in_scene = True
                    break
            else:
                rospy.sleep(1.0)

        rospy.loginfo("'" + object_name + "'' is in scene!")

        
        

if __name__ == "__main__":
    try:
        node = PickObject()
        node.spin()
    except rospy.ROSInternalException:
        rospy.loginfo("Tiago Motion Controller shutting down.")
