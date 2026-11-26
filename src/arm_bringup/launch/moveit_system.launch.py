"""FR3 with MoveIt: Gazebo simulation, controller_node, trajectory_action_server, move_group and RViz.

controller_node starts with the simulation, so its gravity-compensation torques are already
being published when effort_controller activates and the arm never hangs unpowered.
trajectory_action_server starts only after the effort_controller spawner from sim.launch.py has
finished. It holds the arm at its first measured pose and executes MoveIt trajectories on
/fr3_arm_controller/follow_joint_trajectory by streaming them to /desired_joint_states.
planning_scene_setup adds the platform and the ground to MoveIt's planning scene, so plans avoid them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without the GUI (server only)",
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arm_sim"),
                "launch",
                "sim.launch.py"
            ])
        ),
        launch_arguments={"headless": LaunchConfiguration("headless")}.items()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fr3_moveit_config"),
                "launch",
                "moveit_sim.launch.py"
            ])
        )
    )

    controller_node = Node(
        package="arm_control",
        executable="controller_node",
        output="screen",
    )

    trajectory_action_server = Node(
        package="arm_traj_gen",
        executable="trajectory_action_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Adds the platform and the ground to MoveIt's planning scene once move_group is up, then exits.
    planning_scene_setup = Node(
        package="arm_bringup",
        executable="planning_scene_setup",
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare("arm_bringup"),
            "config",
            "planning_scene.yaml"
        ])],
    )

    # The effort_controller spawner lives inside sim.launch.py, so it is matched by its
    # command line rather than by reference.
    def start_control_nodes(event, context):
        cmd = " ".join(str(part) for part in event.cmd)
        if "spawner" in cmd and "effort_controller" in cmd:
            if event.returncode != 0:
                return [LogInfo(msg="effort_controller failed to load; not starting trajectory_action_server.")]
            return [trajectory_action_server]
        return []

    start_after_effort_controller = RegisterEventHandler(
        OnProcessExit(on_exit=start_control_nodes)
    )

    return LaunchDescription([
        declare_headless,
        start_after_effort_controller,
        sim,
        controller_node,
        moveit,
        planning_scene_setup,
    ])
