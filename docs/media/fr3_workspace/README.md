# FR3 workspace and arm diagrams

Labelled images of the simulated Franka FR3 workspace and arm, plus diagrams of its joint ranges and reachable workspace. The renders come from Gazebo Sim (ogre2) running the project's own `arm_bringup full_system.launch.py`, unchanged, captured after the default trajectory (+0.1 rad on `fr3_joint1` over 5 s) while the PD + gravity controller holds the arm. Captions are read from `arm_sim/worlds/fr3_world.sdf`, `arm_sim/launch/sim.launch.py`, `arm_sim/urdf/fr3.urdf`, `arm_traj_gen/src/traj_gen_node.cpp` and `arm_control/config/gz_controllers.yaml`.

## Regenerate

From the repo root, in zsh with no conda env active and the workspace built:

```zsh
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
python3 docs/media/fr3_workspace/make_media.py .
```

It takes about 20–25 s. It runs on its own ROS domain (86) and Gazebo partition, so it doesn't interfere with a simulation you have open. To get images out of Gazebo, it temporarily overlays the `arm_sim` package with a copy of `fr3_world.sdf` that also has Gazebo's Sensors system and three cameras. The overlay lives in a temporary directory; nothing in `src/` or `install/` is modified.

## Stills (Gazebo renders, 1920×1080)

| File | Shows |
|---|---|
| `fr3_workspace_overview.png` | The whole scene: FR3 arm, Franka Hand, robot base fixed at (0, 0, 0.40) m, the 1.20 × 1.20 × 0.40 m platform, and the ground plane |
| `fr3_joint_diagram.png` | Side view with all 7 joints labelled with their range and torque limit (e.g. `fr3_joint4` −176° to −7°, 87 Nm; `fr3_joint6` +25° to +265°, 12 Nm), plus the `fr3_hand_tcp` tool frame |
| `fr3_top_view_reach.png` | Top-down view: platform outline and the arm's horizontal reach circle (≈ 0.94 m) around the joint-1 axis |

## Diagrams (from the arm model)

| File | Shows |
|---|---|
| `plot_joint_ranges.png` | Range of each joint, with the home pose marked. Joint 4 is always negative and joint 6 always above +25°, so the all-zero pose is outside the limits |
| `plot_reachable_workspace_side.png` | Side view (x–z) of 20,000 sampled tool-centre-point positions: the workspace reaches up to ≈ 1.68 m above the ground |
| `plot_reachable_workspace_top.png` | Top view (x–y) of the same samples: reach ≈ 0.94 m from the joint-1 axis, well beyond the platform edge (0.60 m) |

## Notes and anomalies

- **The joint markers aren't all at the joint-frame origins.** On the FR3, `fr3_joint1`/`fr3_joint2` and `fr3_joint5`/`fr3_joint6` share an origin, and joint 1's frame sits 0.333 m up its axis. Joints 1, 3 and 5 are marked further along their own rotation axes (towards the base, the upper arm and the forearm), where the actuator is. All positions come from Pinocchio forward kinematics of the measured `/joint_states`.
- **The pose isn't exactly the nominal home + 0.1 rad.** Joint 1 is at 0.099 rad (the PD controller's steady-state error), and joint 4 is 1.4 mrad below home (sag at startup, before the controller engages).
- **The reach estimate uses joint limits only.** Self-collision and collisions with the platform aren't checked, so some sampled points lie inside the platform or the arm. 0.94 m is the reach of the hand's tool centre point. Franka quotes 855 mm for the bare arm.
- **The ground plane renders black**, because the project world's ground plane has no material. The soft bright area in the top right of the top view is the world's directional light reflecting off it. Both are how the world looks, and they're left as they are.
- **Why there's an overlay:** loading Gazebo's Sensors system into the running world doesn't work, because cameras added that way only see entities created afterwards, so the frames come out empty.

## Embed in the main README

```markdown
![FR3 simulation workspace](docs/media/fr3_workspace/fr3_workspace_overview.png)

![FR3 joints and tool frame](docs/media/fr3_workspace/fr3_joint_diagram.png)

![FR3 joint ranges](docs/media/fr3_workspace/plot_joint_ranges.png)
```
