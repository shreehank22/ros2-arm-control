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
```

## Run

### Launch the simulation

`sim.launch.py` starts Gazebo Sim with the UR5e world, the `/clock` bridge, `robot_state_publisher`, spawns the robot, and starts the `joint_state_broadcaster`:

```bash
ros2 launch arm_sim sim.launch.py
```

### Spawn the robot manually

The launch file spawns the UR5e at the home configuration with `ros_gz_sim create`. To spawn it by hand (e.g. into an already running world):

```bash
ros2 run ros_gz_sim create \
  -name ur5e \
  -file src/arm_sim/urdf/ur5e.urdf \
  -x 0.0 -y 0.0 -z 0.4 \
  -J shoulder_pan_joint 0.0 \
  -J shoulder_lift_joint -1.571 \
  -J elbow_joint 0.0 \
  -J wrist_1_joint -1.571 \
  -J wrist_2_joint 0.0 \
  -J wrist_3_joint 0.0
```

Spawn the joint state broadcaster:

```bash
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager-timeout 60
```

### Start the controller

In a separate terminal:

```bash
ros2 run arm_control controller_node
```

The controller publishes effort commands to `/effort_controller/commands`.

### Visualize in RViz

```bash
ros2 launch arm_description display.launch.py
```

## Useful Commands

Check controller status:

```bash
ros2 control list_controllers
```

Check joint states:

```bash
ros2 topic echo /joint_states
```

Check effort commands:

```bash
ros2 topic echo /effort_controller/commands
```

Check simulation time:

```bash
ros2 topic echo /clock
```

## License

MIT
