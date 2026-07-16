# arm_ws — 3R Planar Arm: ROS2 PD Control

ROS2 Jazzy project porting kinematics/dynamics from `Study-of-Robot-Dynamics`
into a live pub/sub control loop.

## Packages
- `arm_description` — URDF/xacro of the 3R planar arm, RViz display launch
- `arm_sim` — dynamics-based integrator node (owns true M(q), C(q,qdot), G(q))
- `arm_control` — PD + gravity-compensation controller node
- `arm_traj_gen` — quintic polynomial joint-space trajectory generator
- `arm_bringup` — launch file bringing up the full system

## Build
```bash
mamba activate ros2_jazzy
cd ~/arm_ws
colcon build --symlink-install
source install/setup.zsh
```

## Run
```bash
ros2 launch arm_bringup full_system.launch.py
```

## Build order
1. `arm_description` — verify URDF renders correctly in RViz
2. `arm_sim` alone with zero torque — check gravity-driven motion matches RNEA ground truth
3. `arm_control` with fixed setpoint — check step response settles
4. `arm_traj_gen` — full trajectory tracking
