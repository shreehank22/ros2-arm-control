from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro, os

def generate_launch_description():
    # same FR3 URDF the simulation uses; xacro resolves its $(find arm_control) path
    xacro_file = os.path.join(
        get_package_share_directory('arm_sim'), 'urdf', 'fr3.urdf'
    )
    robot_description = {
        'robot_description': xacro.process_file(xacro_file).toxml()
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
