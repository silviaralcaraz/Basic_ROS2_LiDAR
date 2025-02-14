"""
@file       main.py

@author     Silvia R. Alcaraz
@version    1.0

@copyright  Copyright (C) 2025 
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from lidar_processing.lidar_node import LidarProcessor

def main(args=None):
    """
    Main function to initialize the ROS2 node and handle its lifecycle.

    This function sets up the ROS2 environment, initializes a LidarProcessor node,
    and uses a MultiThreadedExecutor to manage the node's callbacks.
    """
    # Initialize ROS2 and create an instance of LidarProcessor node
    rclpy.init(args=args) 
    node = LidarProcessor() 
    
    # Use MultiThreadedExecutor to handle the callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run the executor to handle node operations
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass    
    
    # Cleanup the node and shut down ROS2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()