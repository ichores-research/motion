# ROS Service for motion planning
# Exposes a simple API for motion templates such as "pick", "place", "move", etc.
import rospy
import moveit_commander
from motion_msgs.srv import Prepare, PrepareResponse

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_POSE_JOINT_POSITIONS = [
    0.34, # torso_lift_joint
    0.10, # arm_1_joint
    0.47, # arm_2_joint
    -0.20, # arm_3_joint
    1.56, # arm_4_joint
    1.60, # arm_5_joint
    0.25, # arm_6_joint
    1.19 # arm_7_joint
]

DEFAULT_HEAD_JOINT_POSITIONS = [
    0.0, # head_1_joint
    -0.75 # head_2_joint
]


class MotionService:
    def __init__(self, group_name="arm_torso"):
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


    def prepare_robot(self, req):
        """
        Prepares the robot for operation by moving the torso and arm to a safe position.
        """
        rospy.loginfo("Unfolding arm safely")
        # Move the torso to a safe position
        self._move_to_joint_positions(DEFAULT_POSE_JOINT_POSITIONS)
        
        # Lower the head
        self.lower_head()
        rospy.loginfo("Robot prepared.")

        return PrepareResponse()

    # def pick(self, item_id, target_pose):
    #     pass



if __name__ == "__main__":
    motion_service = MotionService()
    rospy.spin()
