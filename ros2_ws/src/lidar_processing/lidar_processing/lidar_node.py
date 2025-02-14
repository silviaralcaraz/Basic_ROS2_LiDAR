"""
@file       lidar_node.py

@author     Silvia R. Alcaraz
@version    1.0

@copyright  Copyright (C) 2025 
"""

from lidar_processing.point_cloud_utils import *
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
import time

# ANSI codes for printing
YELLOW = '\033[93m'
RESET = '\033[0m'

class LidarProcessor(Node):
    """
    This node subscribes to a topic receiving PointCloud2 messages, 
    converts them to Open3D format, applyies a filter based on a distance
    threshold, and publishes the filtered point cloud converted back to 
    PointCloud2 format. Also, logs processing times if debug mode is enabled.
    """
    def __init__(self):
        """
        Initialize the LidarProcessor node.
        """
        super().__init__('lidar_node')
        self.setup_parameters()
        self.setup_qos_and_topics()
        
    def setup_parameters(self):
        """
        Set up parameters for the node.
        """
        self.real_time_constraint = self.declare_parameter('real_time_constraint', 0.1).value
        self.debug_mode = self.declare_parameter('debug_mode', False).value
        self.input_topic = self.declare_parameter('input_topic', '/velodyne_points').value
        self.frame_id = self.declare_parameter('frame_id', 'velodyne').value
        self.distance_threshold = self.declare_parameter('distance_threshold', 10.0).value

        params = {
            "Real-time constraint": f"{self.real_time_constraint} seconds",
            "Debug mode": self.debug_mode,
            "Input topic": self.input_topic,
            "Frame id": self.frame_id,
            "Distance threshold": f"{self.distance_threshold} m"
        }
        self.get_logger().info(f"Node parameters: {params}")


    def setup_qos_and_topics(self):
        """
        Configure the Quality of Service (QoS) profile and set up the ROS 2 topics.
        """
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reduce communication delay
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Queue size
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic, 
            self.listener_callback,
            qos_profile)
        
        # Create a publisher to send out the modified point cloud
        self.publisher = self.create_publisher(PointCloud2, '/filtered_pointcloud', 10)

    def listener_callback(self, msg):
        """
        Callback function triggered when a new PointCloud2 message is received.

        Args:
            msg (sensor_msgs.msg.PointCloud2): The received point cloud message.

        Returns:
            None
        """
        start_processing = time.time()
        
        # Convert ROS 2 message to Open3D point cloud
        start_time = time.time()
        cloud = ros2_msg_to_o3d_xyz(msg)
        ros_to_o3d_time = time.time() - start_time
        
        # Filter points based on the distance threshold
        start_time = time.time()
        filtered_cloud = filter_points_by_distance(cloud, self.distance_threshold)
        filter_time = time.time() - start_time
        
        # Convert back to ROS2 message
        start_time = time.time()
        filtered_msg = o3d_to_ros_msg_xyz(filtered_cloud, self.frame_id)
        o3d_to_ros_time = time.time() - start_time
        
        # Publish the filtered point cloud
        self.publisher.publish(filtered_msg)
        
        processing_time = time.time() - start_processing
        
        # Log the times if debug mode is enabled
        if self.debug_mode:
            self.get_logger().info(f"Processing time: {processing_time:.4f} seconds")
            if processing_time > self.real_time_constraint:
                self.get_logger().warn(f'{YELLOW}Processing time {processing_time:.6f} seconds exceeded the constraint by {processing_time - self.real_time_constraint:.6f} seconds{RESET}')
            self.get_logger().info(f"Time for ROS to Open3D conversion: {ros_to_o3d_time:.4f} seconds")
            self.get_logger().info(f"Time for filtering by distance: {filter_time:.4f} seconds")
            self.get_logger().info(f"Time for Open3D to ROS conversion: {o3d_to_ros_time:.4f} seconds")
            self.get_logger().info(f"-------------------------------")