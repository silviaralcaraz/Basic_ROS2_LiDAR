"""
@file       point_cloud_utils.py

@author     Silvia R. Alcaraz
@version    1.0

@copyright  Copyright (C) 2025 
"""

import rclpy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

def ros2_msg_to_o3d(ros_cloud):
    """
    Convert a ROS2 PointCloud2 message to an Open3D Tensor-based PointCloud 
    containing all the message fields.
    
    Args:
        ros_cloud (sensor_msgs.msg.PointCloud2): ROS2 point cloud message.
        ---------------------------------------------
        Available fields in the PointCloud2 message:
        - x (datatype: 7, offset: 0)
        - y (datatype: 7, offset: 4)
        - z (datatype: 7, offset: 8)
        - intensity (datatype: 7, offset: 12)
        - rgb (datatype: 6, offset: 16)
        - label (datatype: 4, offset: 20)
        ---------------------------------------------
    Returns:
        o3d.t.geometry.PointCloud: Open3D tensor-based point cloud.
    """
    # Define device and dtype for tensor creation
    device = o3d.core.Device("CPU:0")
    dtype = o3d.core.float32  

    # Point cloud data -> structured numpy array
    dtype_np = [('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('intensity', np.float32), ('rgb', np.uint32), ('label', np.int32)]
    cloud_array = np.array(list(pc2.read_points(ros_cloud, field_names=("x", "y", "z", "intensity", "rgb", "label"), skip_nans=True)), dtype=dtype_np)

    # Split fields into separate arrays using numpy
    points = np.stack((cloud_array['x'], cloud_array['y'], cloud_array['z']), axis=-1)
    intensities = cloud_array['intensity']
    rgb_values = cloud_array['rgb']
    labels = cloud_array['label']

    # Process RGB values to extract individual channels
    r = ((rgb_values >> 16) & 0xFF) / 255.0
    g = ((rgb_values >> 8) & 0xFF) / 255.0
    b = (rgb_values & 0xFF) / 255.0
    colors = np.stack((r, g, b), axis=-1)

    # Create Open3D Tensor-based PointCloud
    o3d_cloud = o3d.t.geometry.PointCloud(device)
    o3d_cloud.point.positions = o3d.core.Tensor(points, dtype, device)
    o3d_cloud.point.colors = o3d.core.Tensor(colors, dtype, device)
    o3d_cloud.point.intensities = o3d.core.Tensor(intensities, dtype, device)
    o3d_cloud.point.labels = o3d.core.Tensor(labels, o3d.core.int32, device)

    return o3d_cloud

def ros2_msg_to_o3d_xyz(ros_cloud):
    """
    Convert the (x, y, z) ROS2 PointCloud2 message to an Open3D Tensor-based (x,y,z) PointCloud.
    
    Args:
        ros_cloud (sensor_msgs.msg.PointCloud2): ROS2 point cloud message.

    Returns:
        o3d.t.geometry.PointCloud: Open3D tensor-based point cloud.
    """
    device = o3d.core.Device("CPU:0")
    dtype = o3d.core.float32  

    dtype_np = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
    cloud_array = np.array(list(pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True)), dtype=dtype_np)

    points = np.stack((cloud_array['x'], cloud_array['y'], cloud_array['z']), axis=-1)

    o3d_cloud = o3d.t.geometry.PointCloud(device)
    o3d_cloud.point.positions = o3d.core.Tensor(points, dtype, device)

    return o3d_cloud

def o3d_to_ros_msg(o3d_cloud, frame_id="velodyne"):
    """
    Convert an Open3D Tensor-based PointCloud (x, y, z, intensity, rgb, labels) to 
    a ROS2 PointCloud2 message.

    Args:
        o3d_cloud (o3d.t.geometry.PointCloud): Open3D point cloud.
        frame_id (str): Frame of reference for the output ROS2 PointCloud2.

    Returns:
        PointCloud2: Efficiently created ROS2 PointCloud2 message.
    """
    # Extract point attributes and prepare for ROS message conversion
    points = o3d_cloud.point.positions.numpy().astype(np.float32)
    colors = (o3d_cloud.point.colors.numpy() * 255).astype(np.uint8)
    intensities = o3d_cloud.point.intensities.numpy().astype(np.float32)
    labels = o3d_cloud.point.labels.numpy().astype(np.uint32)

    # Pack RGB as a single float using vectorized operations
    rgb_floats = (colors[:, 0].astype(np.uint32) << 16 |
                  colors[:, 1].astype(np.uint32) << 8 |
                  colors[:, 2].astype(np.uint32)).astype(np.float32)

    # Prepare a structured array for the PointCloud2 message
    structured_array = np.zeros(len(points), dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('intensity', np.float32), ('rgb', np.float32), ('label', np.uint32)
    ])
    structured_array['x'], structured_array['y'], structured_array['z'] = points.T
    structured_array['intensity'] = intensities
    structured_array['rgb'] = rgb_floats
    structured_array['label'] = labels

    # Create ROS2 message header
    header = Header()
    header.stamp = rclpy.time.Time().to_msg()
    header.frame_id = frame_id

    # Define PointCloud2 fields and create the message
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="label", offset=20, datatype=PointField.UINT32, count=1),
    ]
    pc2_msg = pc2.create_cloud(header, fields, structured_array)

    return pc2_msg

def o3d_to_ros_msg_xyz(o3d_cloud, frame_id):
    """
    Convert an Open3D Tensor-based PointCloud (x, y, z) to a ROS2 PointCloud2 message efficiently.

    Args:
        o3d_cloud (o3d.t.geometry.PointCloud): Open3D point cloud.
        frame_id (str): Frame of reference for the output ROS2 PointCloud2.

    Returns:
        PointCloud2: Efficiently created ROS2 PointCloud2 message.
    """
    points = o3d_cloud.point.positions.numpy().astype(np.float32)
    
    structured_array = np.zeros(len(points), dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('intensity', np.float32), ('rgb', np.float32), ('label', np.uint32)
    ])
    structured_array['x'], structured_array['y'], structured_array['z'] = points.T

    header = Header()
    header.stamp = rclpy.time.Time().to_msg()
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="label", offset=20, datatype=PointField.UINT32, count=1),
    ]
    pc2_msg = pc2.create_cloud(header, fields, structured_array)

    return pc2_msg
        
def filter_points_by_distance(cloud, distance_threshold):
    """
    Filter points in a point cloud based on a distance threshold from the origin.
    
    Args:
        cloud (o3d.t.geometry.PointCloud): Open3D point cloud.
        distance_threshold (float): Distance threshold in meters.
        
    Returns:
        o3d.t.geometry.PointCloud: Filtered Open3D point cloud.
    """
    # Convert Open3D tensor to NumPy
    positions_np = cloud.point.positions.numpy()
    
    # Compute distances from the origin
    distances = np.linalg.norm(positions_np, axis=1)

    # Apply filtering
    mask = distances < distance_threshold
    cloud = cloud.select_by_index(np.where(mask)[0])

    return cloud

def print_available_fields(pointcloud_msg):
    """
    Print the available fields in a ROS2 PointCloud2 message.
    
    Args:
        pointcloud_msg (sensor_msgs.msg.PointCloud2): ROS2 point cloud message.
        
    Returns:
        None
    """
    print("Available fields in the PointCloud2 message:")
    for field in pointcloud_msg.fields:
        print(f"- {field.name} (datatype: {field.datatype}, offset: {field.offset})")

def print_point_cloud(logger, cloud):
    """
    Log information about a point cloud using a ROS 2 logger.

    Args:
        logger (rclpy.impl.rcutils_logger.RcutilsLogger): ROS 2 logger.
        cloud (o3d.geometry.PointCloud): Open3D point cloud object.

    Returns:
        None
    """
    points = np.asarray(cloud.points)
    logger.info(f'Point cloud has {len(points)} points')
    for point in points:
        logger.info(f'Point: {point}')