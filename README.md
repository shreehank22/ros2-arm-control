# ROS 2 UR5e Arm Control

A ROS 2 Jazzy simulation and control stack for a UR5e robot arm using Pinocchio-based rigid-body dynamics, PD control with gravity compensation, Gazebo Sim, and `ros2_control`.

## Packages

- `arm_sim` — Gazebo Sim simulation, UR5e model, world, and simulation utilities
- `arm_control` — Pinocchio-based PD controller with gravity compensation
- `arm_description` — URDF-based robot description and RViz visualization
- `arm_bringup` — launch and bringup configuration
- `arm_traj_gen` — trajectory generation utilities

## Requirements

- ROS 2 Jazzy
- Gazebo Sim Harmonic
- `colcon`
- Pinocchio
- `ros2_control`
- `gz_ros2_control`
- `ros_gz_sim`
- Universal Robots ROS description package (`ur_description`)

## Build

```bash
cd ~/ros2-arm-control
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
````

## Run

Start the Gazebo simulation:

```

```bash
ros2 launch arm_sim sim.launch.py
```

Start the controller in a separate terminal:

```

```bash
ros2 run arm_control controller_node
```

The controller publishes effort commands to:

```

```text
/effort_controller/commands
```

Visualize the robot in RViz:

```

```bash
ros2 launch arm_description display.launch.py
```

Or launch the interactive Foxglove workflow:

```

```bash
./scripts/launch_robot.sh
```

## Useful Commands

Check controller status:

```

```bash
ros2 control list_controllers
```

Check joint states:

```

```bash
ros2 topic echo /joint_states
```

Check effort commands:

```

```bash
ros2 topic echo /effort_controller/commands
```

Check simulation time:

```

```bash
ros2 topic echo /clock
```

## License

MIT

```
