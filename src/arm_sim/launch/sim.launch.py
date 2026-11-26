import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    arm_sim_share = FindPackageShare("arm_sim")

    headless = LaunchConfiguration("headless")
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without the GUI (server only)",
    )

    world_file = PathJoinSubstitution([
        arm_sim_share,
        "worlds",
        "fr3_world.sdf"
    ])

    robot_description_file = PathJoinSubstitution([
        arm_sim_share,
        "urdf",
        "fr3.urdf"
    ])

    # xacro resolves $(find arm_control) in the gz_ros2_control <parameters> path
    robot_description = ParameterValue(
        Command([
            "xacro ",
            robot_description_file
        ]),
        value_type=str
    )

    # lets Gazebo resolve the package://arm_sim/meshes/... paths in the URDF
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.dirname(get_package_share_directory("arm_sim"))
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PythonExpression(["'-s ' if '", headless, "' == 'true' else ''"]),
                world_file
            ]
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

    # spawned from /robot_description so Gazebo gets the xacro-processed URDF
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "fr3",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.4",

            # FR3 home configuration
            "-J", "fr3_joint1", "0.0",
            "-J", "fr3_joint2", "-0.7854",
            "-J", "fr3_joint3", "0.0",
            "-J", "fr3_joint4", "-2.3562",
            "-J", "fr3_joint5", "0.0",
            "-J", "fr3_joint6", "1.5708",
            "-J", "fr3_joint7", "0.7854",
        ],
        output="screen"
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # effort controller that receives controller_node's /effort_controller/commands
    effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # gripper action controller for the Franka Hand (/hand_controller/gripper_cmd)
    hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # start the effort and hand controllers once the joint state broadcaster is up
    effort_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[effort_controller, hand_controller],
        )
    )

    return LaunchDescription([
        declare_headless,
        gz_resource_path,
        gazebo,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        effort_controller_after_broadcaster,
    ])
