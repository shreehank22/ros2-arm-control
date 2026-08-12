from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro, os

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('ur_description'), 'urdf', 'ur.urdf.xacro'
    )
    robot_description = {
        'robot_description': xacro.process_file(
            xacro_file, mappings={'name': 'ur5e', 'ur_type': 'ur5e'}
        ).toxml()
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])