from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_share = get_package_share_directory('arm_description')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_share, 'launch', 'display.launch.py')
            )
        ),
        Node(package='arm_sim', executable='sim_node', output='screen'),
        Node(
            package='arm_control', executable='controller_node', output='screen',
            parameters=[os.path.join(
                get_package_share_directory('arm_control'), 'config', 'gains.yaml'
            )]
        ),
        Node(package='arm_traj_gen', executable='traj_gen_node', output='screen'),
    ])
