from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    arm_sim_share = FindPackageShare("arm_sim")

    world_file = PathJoinSubstitution([
        arm_sim_share,
        "worlds",
        "ur5e_world.sdf"
    ])

    robot_description_file = PathJoinSubstitution([
        arm_sim_share,
        "urdf",
        "ur5e.urdf"
    ])

    robot_description = Command([
        "cat ",
        robot_description_file
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ["-r ", world_file]
        }.items()
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur5e",
            "-file", robot_description_file,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.4"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
    ])