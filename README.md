# ROS 2 UR5e Arm Control

A ROS 2 Jazzy simulation of a UR5e robot arm with Pinocchio-based rigid-body dynamics and a PD controller with gravity compensation.

## Packages

- `arm_sim` — integrates UR5e dynamics and publishes `/joint_states`
- `arm_control` — publishes torque commands to `/joint_efforts`
- `arm_description` — URDF-based RViz visualization

## Requirements

- ROS 2 Jazzy
- `colcon`
- Pinocchio
- Universal Robots ROS description package (`ur_description`)

## Build

```bash
cd ~/ros2-arm-control
colcon build --symlink-install
source install/setup.zsh
```

## Run

Start the simulator and controller in separate terminals:

```bash
ros2 run arm_sim sim_node
ros2 run arm_control controller_node
```

Visualize the robot in RViz:

```bash
ros2 launch arm_description display.launch.py
```

Or launch the interactive Foxglove workflow:

```bash
./scripts/launch_robot.sh
```

## License

MIT
