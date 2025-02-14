from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_processing',  # Package name
            executable='lidar_node',  # Name of the main entrypoint (from setup.py)
            name='lidar_processor',  # ROS 2 node name
            output='screen',
            parameters=[ 
                {'real_time_constraint': 0.1},  
                {'debug_mode': False},  
                {'input_topic': '/velodyne_points'},
                {'frame_id': 'velodyne'},    
                {'distance_threshold': 10.0}  
            ]
        )
    ])
