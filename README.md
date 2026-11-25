# ROS 2 Franka FR3 Arm Control

A ROS 2 Jazzy simulation and control stack for a Franka FR3 arm with the Franka Hand. It uses Pinocchio for rigid-body dynamics, a PD controller with gravity compensation, quintic joint trajectories, Gazebo Sim and `ros2_control`.

## Packages

- `arm_sim`: FR3 + hand URDF and meshes, Gazebo world, simulation launch
- `arm_control`: PD + gravity-compensation controller (`controller_node`) and the `ros2_control` controller configuration
- `arm_traj_gen`: quintic joint trajectory generator (`traj_gen_node`) with URDF joint-limit checks
- `arm_kinodynamics`: Pinocchio library: arm model loading, kinematics, Jacobian and dynamics
- `arm_bringup`: launch file for the full system

## How it fits together

```
traj_gen_node ──/desired_joint_states──▶ controller_node ──/effort_controller/commands──▶ Gazebo (gz_ros2_control)
      ▲                                        ▲                                              │
      └───────────────/joint_states────────────┴──────────── joint_state_broadcaster ◀────────┘
```

- The controller and trajectory generator load the FR3 URDF with the finger joints locked. That gives a 7-joint arm model that still includes the hand's mass for gravity compensation.
- `controller_node` runs a small state machine:
  - `WAITING`: no joint state received yet
  - `FALLBACK`: gravity compensation only (no fresh trajectory, or stale data)
  - `TRACKING`: PD + gravity compensation on the desired trajectory
- `traj_gen_node` reads the arm's current position, checks the start and the target against the URDF joint limits, and publishes a 5 s quintic move of `fr3_joint1` by +0.1 rad.

## Requirements

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Gazebo Sim Harmonic
- `ros2_control`, `gz_ros2_control`, `ros_gz_sim`, `ros_gz_bridge`, `xacro`
- Pinocchio 4 (e.g. from robotpkg in `/opt/openrobots`)
- `colcon`

Build and run from the system ROS installation. If a conda environment is active, run `conda deactivate` first: mixing conda's Python or linker with `/opt/ros/jazzy` breaks both the build and `ros2`.

## Build

```bash
cd ~/ros2-arm-control
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

These commands are for zsh. In bash, source `setup.bash` instead of `setup.zsh` (both for `/opt/ros/jazzy` and `install/`). The zsh files fail in bash with `bad substitution`.

## Run

### Full system

Starts Gazebo with the FR3, the controllers, `controller_node`, and then `traj_gen_node` once the effort controller is active:

```bash
ros2 launch arm_bringup full_system.launch.py
```

Without the Gazebo GUI:

```bash
ros2 launch arm_bringup full_system.launch.py headless:=true
```

The arm spawns at the home pose `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`, then rotates joint 1 by 0.1 rad over 5 s and holds.

### Simulation only

Starts Gazebo, the `/clock` bridge and `robot_state_publisher`, spawns the FR3, and loads `joint_state_broadcaster` and `effort_controller`:

```bash
ros2 launch arm_sim sim.launch.py            # add headless:=true for no GUI
```

Then start the nodes in separate terminals, controller first so the arm is held against gravity:

```bash
ros2 run arm_control controller_node
ros2 run arm_traj_gen traj_gen_node
```

## Tests

```bash
colcon test --packages-select arm_kinodynamics arm_traj_gen arm_control
colcon test-result --verbose
```

The `arm_traj_gen` integration test runs `traj_gen_node` on its own ROS domain, so it doesn't interfere with a running simulation.

## Useful Commands

```bash
ros2 control list_controllers                 # controller status
ros2 topic echo /joint_states                 # measured joint states (arm + fingers)
ros2 topic echo /desired_joint_states         # trajectory generator output
ros2 topic echo /effort_controller/commands   # controller torques (fr3_joint1..7)
ros2 topic echo /clock                        # simulation time
```

## Troubleshooting

- **`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`**: a conda environment is active. Run `conda deactivate`, then source `/opt/ros/jazzy/setup.zsh` and `install/setup.zsh` again. `which ros2` should print `/opt/ros/jazzy/bin/ros2`.
- **`bad substitution` / `No module named 'ament_package'` during the build**: the `.zsh` setup files were sourced from bash. Use `setup.bash` in bash, or switch to zsh.
- **The arm shakes**: check for duplicate nodes left over from an earlier launch with `ros2 node list | sort | uniq -c`, and stop them (`pkill -f controller_node`, `pkill -f traj_gen_node`, `pkill -f "gz sim"`).
- **`[Err] [Physics.cc] ... mimic constraint for joint [fr3_finger_joint2]`**: harmless. Gazebo's default physics engine does not support the URDF mimic tag, so both fingers are driven directly through `ros2_control` instead.

## Robot model

The FR3 + Franka Hand URDF and meshes in `arm_sim` come from [franka_description](https://github.com/frankarobotics/franka_description) 2.9.0 (Apache-2.0). Their license and notice are in `src/arm_sim/meshes/`, and the hand edits are listed at the top of `src/arm_sim/urdf/fr3.urdf`.

## License

MIT
