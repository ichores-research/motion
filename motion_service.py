# ROS Service for motion planning
# Exposes a simple API for motion templates such as "pick", "place", "move", etc.
import time
import rospy
import moveit_commander

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MotionService:
    def __init__(self):
        rospy.loginfo("Initializing Motion Service...")

        rospy.loginfo("Setting publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher(
            "/torso_controller/command", JointTrajectory, queue_size=1
        )
        self.head_cmd = rospy.Publisher(
            "/head_controller/command", JointTrajectory, queue_size=1
        )
        self.arm_cmd = rospy.Publisher(
            "/arm_controller/command", JointTrajectory, queue_size=1
        )
        self.gripper_cmd = rospy.Publisher(
            "/gripper_controller/command", JointTrajectory, queue_size=1
        )

        rospy.loginfo("Done initializing Motion Service.")

    def lower_head(self):
        """
        Lowers the head of the robot to a predefined position.
        """
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ["head_1_joint", "head_2_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def move_to_positions(self, joint_names, positions_list, time_durations, publisher):
        """
        Moves the robot to the specified positions.

        :param joint_names: List of joint names.
        :param positions_list: List of joint position configurations.
        :param time_durations: List of time durations for each configuration.
        :param publisher: The ROS publisher to send the trajectory to.
        """
        rospy.loginfo("Moving to specified positions...")
        if len(positions_list) != len(time_durations):
            rospy.logerr("Mismatch between positions and time durations.")
            return

        jt = JointTrajectory()
        jt.joint_names = joint_names

        for positions, duration in zip(positions_list, time_durations):
            jtp = JointTrajectoryPoint()
            jtp.positions = positions
            jtp.time_from_start = rospy.Duration(duration)
            jt.points.append(jtp)

        publisher.publish(jt)
        rospy.loginfo("Trajectory published to %s", publisher.name)

    def prepare_robot(self):
        """
        Prepares the robot for operation by moving the torso and arm to a safe position.
        """
        rospy.loginfo("Unfolding arm safely")

        # Move torso first
        torso_joint_names = ["torso_lift_joint"]
        torso_positions_list = [[0.34]]  # Only one position for the torso
        torso_time_durations = [3.0]
        self.move_to_positions(
            torso_joint_names,
            torso_positions_list,
            torso_time_durations,
            self.torso_cmd,
        )

        # Move arm joints
        arm_joint_names = [
            "arm_1_joint",
            "arm_2_joint",
            "arm_3_joint",
            "arm_4_joint",
            "arm_5_joint",
            "arm_6_joint",
            "arm_7_joint",
        ]
        arm_positions_list = [
            [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0],
            [0.10, 0.47, -0.20, 1.56, -1.58, 0.25, 0.0],
            [0.10, 0.47, -0.20, 1.56, 1.60, 0.25, 1.19],
        ]
        arm_time_durations = [3.0, 8.5, 10.5]
        self.move_to_positions(
            arm_joint_names, arm_positions_list, arm_time_durations, self.arm_cmd
        )

        # Lower the head
        self.lower_head()
        rospy.loginfo("Robot prepared.")

    # def pick(self, item_id, target_pose):
    #     pass


if __name__ == "__main__":
    rospy.init_node("motion_service_node")
    motion_service = MotionService()
    rospy.spin()
