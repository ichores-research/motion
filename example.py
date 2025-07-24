import numpy as np
from geometry_msgs.msg import Pose
import rospy

import open3d as o3d
from utils import pick_object


def test_pick():
    rospy.init_node('test_pick_node', anonymous=True)

    # Invent some Pose
    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0.0
    pose.position.z = 0.7
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    
    # Prepare geometry_msgs.Mesh with object mesh from  STL file
    # and geometry_msgs.PoseArray with grasps from numpy file
    model_path = "data/obj_000010.ply"
    model = o3d.io.read_triangle_mesh(model_path)

    npy_path = "data/obj_000010.npy"
    grasps = np.load(npy_path, allow_pickle=True)
    
    # pick the object
    pick_object(model, grasps, pose)
    print("Pick operation completed successfully.")

    

if __name__ == "__main__":
    test_pick()
    print("Test completed successfully.")