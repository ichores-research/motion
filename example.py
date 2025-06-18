import numpy as np
from motion_msgs.srv import Prepare, Pick, PickRequest, PrepareRequest
from geometry_msgs.msg import Pose, PoseArray, Point32
import rospy
from shape_msgs.msg import Mesh

import open3d as o3d
from shape_msgs.msg import Mesh, MeshTriangle
import tf.transformations as tft

def ply_to_shape_mesh(ply_path):
    # Read PLY file using Open3D
    mesh = o3d.io.read_triangle_mesh(ply_path)
    mesh_msg = Mesh()

    # Convert vertices
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    # Add vertices
    for vertex in vertices:
        point = Point32()
        point.x = float(vertex[0]) / 1000.0
        point.y = float(vertex[1]) / 1000.0
        point.z = float(vertex[2]) / 1000.0
        mesh_msg.vertices.append(point)

    # Add triangles
    for triangle in triangles:
        mesh_triangle = MeshTriangle()
        mesh_triangle.vertex_indices = [int(triangle[0]), 
                                      int(triangle[1]), 
                                      int(triangle[2])]
        mesh_msg.triangles.append(mesh_triangle)

    return mesh_msg

def npy_to_pose_array(npy_path):
    # Load numpy array from file
    poses = np.load(npy_path, allow_pickle=True)

    pose_array = PoseArray()
    for pose in poses:
        matrix = pose.reshape(4,4)
        translation = matrix[:3, 3]
        orientation = tft.quaternion_from_matrix(matrix)

        p = Pose()
        p.position.x = float(translation[0])
        p.position.y = float(translation[1])
        p.position.z = float(translation[2])
        p.orientation.x = float(orientation[0])
        p.orientation.y = float(orientation[1])
        p.orientation.z = float(orientation[2])
        p.orientation.w = float(orientation[3])
        pose_array.poses.append(p)
    return pose_array


def test_pick():
    rospy.init_node('test_pick_node', anonymous=True)
    prepare_service = rospy.ServiceProxy('/motion/prepare', Prepare)
    pick_service = rospy.ServiceProxy('/motion/pick', Pick)
    rospy.wait_for_service('/motion/prepare')
    rospy.wait_for_service('/motion/pick')
    print("Services are ready.")

    # Prepare the robot for picking
    try:
        prepare_service(PrepareRequest())
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return
    
    try:
        # Prepare geometry_msgs.Mesh with object mesh from  STL file
        # and geometry_msgs.PoseArray with grasps from numpy file
        mesh_path = "data/obj_000010.ply"
        mesh_msg = ply_to_shape_mesh(mesh_path)
        print(f"Mesh vertices count: {len(mesh_msg.vertices)}")

        npy_path = "data/obj_000010.npy"
        pose_array = npy_to_pose_array(npy_path)
        print(f"PoseArray poses count: {len(pose_array.poses)}")

        # Invent some Pose
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.7
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        # Create Pick message
        pick_req = PickRequest()
        pick_req.object_mesh = mesh_msg
        pick_req.object_pose = pose
        pick_req.grasps = pose_array
        
        print(pick_req)

        # Call the pick service
        response = pick_service(pick_req)
        print(f"Pick service response: {response.success}, {response.message}")
    except Exception as e:
        print(f"An error occurred: {e}")

    

if __name__ == "__main__":
    test_pick()
    print("Test completed successfully.")