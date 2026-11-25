"""Full FR3 system: Gazebo simulation, controller_node and traj_gen_node.

controller_node starts with the simulation, so its gravity-compensation torques are already
being published when effort_controller activates and the arm never hangs unpowered.
traj_gen_node starts only after the effort_controller spawner from sim.launch.py has finished,
so the trajectory starts from a state the controller is already holding.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
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

    controller_node = Node(
        package="arm_control",
        executable="controller_node",
        output="screen",
    )

    traj_gen_node = Node(
        package="arm_traj_gen",
        executable="traj_gen_node",
        output="screen",
    )

    # The effort_controller spawner lives inside sim.launch.py, so it is matched by its
    # command line rather than by reference.
    def start_control_nodes(event, context):
        cmd = " ".join(str(part) for part in event.cmd)
        if "spawner" in cmd and "effort_controller" in cmd:
            if event.returncode != 0:
                return [LogInfo(msg="effort_controller failed to load; not starting traj_gen_node.")]
            return [traj_gen_node]
        return []

    start_after_effort_controller = RegisterEventHandler(
        OnProcessExit(on_exit=start_control_nodes)
    )

    return LaunchDescription([
        declare_headless,
        start_after_effort_controller,
        sim,
        controller_node,
    ])
