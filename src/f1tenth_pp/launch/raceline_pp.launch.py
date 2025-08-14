from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Path + Pure Pursuit
        Node(
            package='f1tenth_pp',
            executable='path_follow_pp',
            name='path_follow_pp',
            output='screen',
            parameters=[{
                'csv_path': '/home/shchon11/sim_ws/maps/raceline_raw.csv',
                'frame_map': 'map',
                'frame_base': 'base_link',
                'path_topic': '/raceline',
                'drive_topic': '/drive',
                'wheelbase': 0.33,
                'lookahead': 1.5,
                'v_max': 5.0,
                'v_min': 0.6,
                'a_lat_max': 3.0,
                'smooth_window': 9,
                'publish_path': True
            }]
        )
    ])

