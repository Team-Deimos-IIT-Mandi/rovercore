#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger # Or a custom service
import joblib
import numpy as np
import os
from armmoveit.srv import GetIKSeed  # <-- Replace with your actual service definition

# Assuming you create a custom Service definition in your package
# from my_robot_msgs.srv import GetIKSeed 

class IKSeederNode(Node):
    def __init__(self):
        super().__init__('seeder')
        
        # Paths
        # Change these lines in seeder.py
        tree_path = '/workspace/workspace_tree.joblib'
        npz_path = '/workspace/workspace_lookup_sampled.npz' # Use the sampled one!
        
        # Load KDTree once into RAM
        self.get_logger().info("Loading KDTree and angles into RAM...")
        self.tree = joblib.load(tree_path)
        data = np.load(npz_path, mmap_mode='r')
        self.angles = data['angles']
        self.get_logger().info("IK Seeder Node: Tree Resident in Memory.")

        # Create Service (Replace 'GetIKSeed' with your custom Service type)
        self.srv = self.create_service(GetIKSeed, 'get_ik_seed', self.handle_seed)

    def handle_seed(self, request, response):
        # target_xyz = [request.x, request.y, request.z]
        # _, index = self.tree.query(target_xyz)
        # response.angles = self.angles[index].tolist()
        # Check what the KDTree actually finds
        dist, index = self.tree.query([request.x, request.y, request.z], k=1)
        self.get_logger().info(f"Tree query result - Dist: {dist}, Index: {index}")

        if index is not None:
            # Use the index to pull from your pre-loaded angles array
            response.joints = self.angles[index].tolist() 
        else:
            self.get_logger().warn("KDTree found NO neighbors!")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = IKSeederNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()