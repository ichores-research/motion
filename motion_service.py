# ROS Service for motion planning
# Exposes a simple API for motion templates such as "pick", "place", "move", etc.
import rospy

class MotionService:
    def __init__(self):
        # Initialize ROS motion service
        self.node = rospy.init_node('motion_service_node')
        # Initialize motion service
        # self.service = rospy.Service('motion_service', MotionService.srv.MotionService, self.handle_motion_request)

    # def pick(self, item_id, target_pose):
    #     pass


if __name__ == '__main__':
    motion_service = MotionService()
    rospy.spin()



