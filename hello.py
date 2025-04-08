import moveit_commander
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GROUP_NAME = "arm_torso"

class MotionNode:
    def __init__(self):
        rospy.init_node("tiago_openvla_controller", anonymous=True)
        rospy.loginfo("Initializing Tiago OpenVLA Controller")

        # Setup MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)
        self.move_group.set_end_effector_link("gripper_link")
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)

    def spin(self):
        rospy.loginfo("Tiago Motion Controller is spinning...")
        rospy.spin()

    def grasp(self, open=True):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0] if not open else [0.04, 0.04]
        point.time_from_start = rospy.Duration(1.0)
        joint_trajectory.points.append(point)

        self.gripper_pub.publish(joint_trajectory)



if __name__ == "__main__":
    try:
        node = MotionNode()
        node.grasp(False)
        node.grasp(True)
        # node.spin()
    except rospy.ROSInternalException:
        rospy.loginfo("Tiago Motion Controller shutting down.")