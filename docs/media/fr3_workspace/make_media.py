#!/usr/bin/env python3
"""Regenerate the labelled FR3 workspace and arm diagrams in this folder.

Usage (zsh, no conda env active, ROS and the built workspace sourced):

    source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
    python3 docs/media/fr3_workspace/make_media.py .

How it works
- Runs the project's own `arm_bringup full_system.launch.py` (headless), unchanged.
- To get pictures out of Gazebo, it puts a temporary overlay of the `arm_sim` package first on
  AMENT_PREFIX_PATH: links to the real installed URDF, meshes and launch files, plus a copy of
  `fr3_world.sdf` with Gazebo's Sensors system and three cameras added. Nothing in the repo or in
  install/ is modified.
- Runs on its own ROS domain and Gazebo partition, so it cannot disturb a simulation you have open.
- Joint labels are placed by projecting the measured joint positions (Pinocchio forward kinematics
  of /joint_states) into each camera image.
- Every number in a caption is read from the project's files (world, launch, URDF, node sources).
"""
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
ROS_DOMAIN = "86"
GZ_PARTITION = "fr3_media_render"
W, H = 1920, 1080
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
BLUE, ORANGE, AQUA, GREY = "#2a78d6", "#eb6834", "#1baf7a", "#8a8a8a"
LOCKED = ["fr3_finger_joint1", "fr3_finger_joint2"]


# ----------------------------------------------------------------------------------------------
# settings read from the project
# ----------------------------------------------------------------------------------------------
def read_settings(repo):
    src = repo / "src"
    s = {}

    world = ET.parse(src / "arm_sim/worlds/fr3_world.sdf").getroot()
    plat = next(m for m in world.iter("model") if m.get("name") == "robot_platform")
    s["platform_pose"] = [float(v) for v in plat.find("pose").text.split()]
    s["platform_size"] = [float(v) for v in plat.find(".//collision/geometry/box/size").text.split()]

    launch = (src / "arm_sim/launch/sim.launch.py").read_text()
    s["spawn_xyz"] = [float(re.search(rf'"-{a}",\s*"([-\d.]+)"', launch).group(1)) for a in "xyz"]
    s["home"] = {j: float(v) for j, v in re.findall(r'"-J",\s*"(\w+)",\s*"([-\d.]+)"', launch)}

    tg = (src / "arm_traj_gen/src/traj_gen_node.cpp").read_text()
    s["traj_duration"] = float(re.search(r"duration_\s*=\s*([\d.]+)", tg).group(1))
    s["traj_offset"] = float(re.search(r"\+=\s*([\d.]+);", tg).group(1))
    s["traj_joint"] = re.search(r'joint_names_\.end\(\),\s*"(\w+)"', tg).group(1)

    yaml_text = (src / "arm_control/config/gz_controllers.yaml").read_text()
    s["update_rate"] = int(re.search(r"update_rate:\s*(\d+)", yaml_text).group(1))
    return s


def load_arm(urdf_path):
    import pinocchio as pin
    full = pin.buildModelFromUrdf(str(urdf_path))
    ids = [full.getJointId(n) for n in LOCKED]
    model = pin.buildReducedModel(full, ids, pin.neutral(full))
    return pin, full, model


# ----------------------------------------------------------------------------------------------
# camera rig
# ----------------------------------------------------------------------------------------------
def look_at(eye, target):
    d = np.subtract(target, eye).astype(float)
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))  # gz: +pitch tilts the view down
    return yaw, pitch


def rot(yaw, pitch):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rz @ ry


class Camera:
    def __init__(self, name, eye, target, hfov, w=W, h=H, top_down=False):
        self.name, self.eye, self.hfov, self.w, self.h = name, np.array(eye, float), hfov, w, h
        if top_down:
            self.yaw, self.pitch = 0.0, math.pi / 2 - 1e-4
        else:
            self.yaw, self.pitch = look_at(eye, target)
        self.R = rot(self.yaw, self.pitch)
        self.f = (w / 2) / math.tan(hfov / 2)
        self.topic = f"media/{name}"

    def sdf(self):
        x, y, z = self.eye
        return f"""
    <model name="media_cam_{self.name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 {self.pitch} {self.yaw}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>true</always_on>
          <update_rate>2</update_rate>
          <topic>{self.topic}</topic>
          <camera>
            <horizontal_fov>{self.hfov}</horizontal_fov>
            <image><width>{self.w}</width><height>{self.h}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>40</far></clip>
          </camera>
        </sensor>
      </link>
    </model>"""

    def project(self, p):
        """World point -> pixel (u, v). gz camera frame: x forward, y left, z up."""
        c = self.R.T @ (np.asarray(p, float) - self.eye)
        return (self.w / 2 - self.f * c[1] / c[0], self.h / 2 - self.f * c[2] / c[0])


def build_overlay(repo, cams, tmp):
    share = repo / "install/arm_sim/share/arm_sim"
    if not (share / "urdf/fr3.urdf").exists():
        sys.exit(f"{share} not found: build the workspace first (colcon build --symlink-install)")
    ov = tmp / "overlay"
    (ov / "share/ament_index/resource_index/packages").mkdir(parents=True)
    (ov / "share/ament_index/resource_index/packages/arm_sim").touch()
    (ov / "share/arm_sim/worlds").mkdir(parents=True)
    for d in ["urdf", "meshes", "launch", "package.xml"]:
        os.symlink(share / d, ov / "share/arm_sim" / d)
    world = (repo / "src/arm_sim/worlds/fr3_world.sdf").read_text()
    rig = ('\n    <!-- media render rig (temporary): rendering system + cameras -->\n'
           '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n'
           '      <render_engine>ogre2</render_engine>\n    </plugin>'
           + "".join(c.sdf() for c in cams) + "\n  ")
    assert world.count("</world>") == 1
    (ov / "share/arm_sim/worlds/fr3_world.sdf").write_text(world.replace("</world>", rig + "</world>"))
    return ov


# ----------------------------------------------------------------------------------------------
# run the simulation and capture
# ----------------------------------------------------------------------------------------------
def capture(cams, overlay, settle_s, log_path, timeout_s=120):
    env = dict(os.environ, ROS_DOMAIN_ID=ROS_DOMAIN, GZ_PARTITION=GZ_PARTITION,
               AMENT_PREFIX_PATH=f"{overlay}:{os.environ.get('AMENT_PREFIX_PATH', '')}")
    os.environ.update(ROS_DOMAIN_ID=ROS_DOMAIN)
    log = open(log_path, "w")
    procs = [subprocess.Popen(["ros2", "launch", "arm_bringup", "full_system.launch.py", "headless:=true"],
                              env=env, stdout=log, stderr=log, start_new_session=True)]
    time.sleep(3)
    bridge_args = [f"/{c.topic}@sensor_msgs/msg/Image[gz.msgs.Image" for c in cams]
    procs.append(subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args],
                                  env=env, stdout=log, stderr=log, start_new_session=True))

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage, JointState

    frames, js, desired_t0 = {}, {}, [None]
    try:
        rclpy.init()
        node = Node("fr3_media_capture")
        node.create_subscription(JointState, "/joint_states",
                                 lambda m: js.update(zip(m.name, m.position)), 10)
        node.create_subscription(JointState, "/desired_joint_states",
                                 lambda m: desired_t0.__setitem__(0, desired_t0[0] or time.monotonic()), 10)
        for c in cams:
            node.create_subscription(RosImage, f"/{c.topic}",
                                     lambda m, n=c.name: frames.__setitem__(n, (time.monotonic(), m)), 2)
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            t0 = desired_t0[0]
            if t0 and time.monotonic() - t0 > settle_s and all(
                    c.name in frames and frames[c.name][0] > t0 + settle_s for c in cams):
                break
        else:
            raise RuntimeError(f"timed out waiting for the trajectory and camera frames (see {log_path})")
        q_meas = dict(js)
        images = {}
        for c in cams:
            m = frames[c.name][1]
            images[c.name] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()
        node.destroy_node()
        rclpy.shutdown()
        return images, q_meas
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(4)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


# ----------------------------------------------------------------------------------------------
# drawing helpers
# ----------------------------------------------------------------------------------------------
def fonts(scale):
    return (ImageFont.truetype(FONT_BOLD, int(30 * scale)), ImageFont.truetype(FONT, int(22 * scale)),
            ImageFont.truetype(FONT_BOLD, int(21 * scale)), ImageFont.truetype(FONT, int(18 * scale)))


def caption(d, w, title, subtitle):
    s = w / 1920
    ft, fs, _, _ = fonts(s)
    pad = int(22 * s)
    tw = max(ft.getlength(title), fs.getlength(subtitle))
    th = ft.getbbox(title)[3] + fs.getbbox(subtitle)[3] + int(10 * s)
    d.rounded_rectangle([pad, pad, 3 * pad + tw, 2 * pad + th + int(6 * s)], radius=int(10 * s), fill=(15, 15, 15, 185))
    d.text((2 * pad, pad + pad // 2), title, font=ft, fill="white")
    d.text((2 * pad, pad + pad // 2 + ft.getbbox(title)[3] + int(10 * s)), subtitle, font=fs, fill=(225, 225, 225))
    return 2 * pad + th + int(6 * s)  # bottom of the caption box


def callouts(d, w, h, items, top, color=(255, 196, 61)):
    """items: list of (point(u,v), title, detail[, side]). Labels go into left/right columns
    (side "L"/"R", default: the half of the image the point is in), spaced vertically in the
    order of their points so leader lines don't cross."""
    s = w / 1920
    _, _, fl, fd = fonts(s)
    pad, gap = int(12 * s), int(14 * s)
    sides = {"L": [], "R": []}
    for it in items:
        side = it[3] if len(it) > 3 else ("L" if it[0][0] < w * 0.5 else "R")
        sides[side].append(it[:3])
    for side, its in sides.items():
        # top to bottom; on a tie, the point nearer the column goes first so the lines don't cross
        its.sort(key=lambda it: (round(it[0][1]), it[0][0] if side == "L" else -it[0][0]))
        boxes = []
        for (u, v), title, detail in its:
            bw = max(fl.getlength(title), fd.getlength(detail) if detail else 0) + 2 * pad
            bh = fl.getbbox(title)[3] + (fd.getbbox(detail)[3] + int(6 * s) if detail else 0) + 2 * pad
            boxes.append([u, v, title, detail, bw, bh])
        # vertical placement: target the point's height, keep order, avoid overlap, stay on screen
        y = top + gap
        for b in boxes:
            b.append(max(y, min(b[1] - b[5] / 2, h - b[5] - gap)))
            y = b[6] + b[5] + gap
        overflow = boxes[-1][6] + boxes[-1][5] + gap - h if boxes else 0
        if overflow > 0:
            for b in boxes:
                b[6] -= overflow
        for u, v, title, detail, bw, bh, by in boxes:
            bx = int(40 * s) if side == "L" else w - bw - int(40 * s)
            anchor = (bx + bw, by + bh / 2) if side == "L" else (bx, by + bh / 2)
            d.line([anchor, (u, v)], fill=color + (255,), width=max(2, int(3 * s)))
            r = int(7 * s)
            d.ellipse([u - r, v - r, u + r, v + r], fill=color + (255,), outline=(20, 20, 20, 255), width=max(1, int(2 * s)))
            d.rounded_rectangle([bx, by, bx + bw, by + bh], radius=int(8 * s), fill=(15, 15, 15, 200),
                                outline=color + (255,), width=max(1, int(2 * s)))
            d.text((bx + pad, by + pad), title, font=fl, fill="white")
            if detail:
                d.text((bx + pad, by + pad + fl.getbbox(title)[3] + int(6 * s)), detail, font=fd, fill=(220, 220, 220))


def footer(d, w, h, text):
    s = w / 1920
    f = ImageFont.truetype(FONT, int(17 * s))
    tw = f.getlength(text)
    pad = int(10 * s)
    d.rounded_rectangle([w - tw - 4 * pad, h - int(44 * s), w - pad, h - pad], radius=int(6 * s), fill=(15, 15, 15, 170))
    d.text((w - tw - 2.5 * pad, h - int(38 * s)), text, font=f, fill=(210, 210, 210))


def polyline_3d(d, cam, pts, color, width):
    uv = [cam.project(p) for p in pts]
    d.line(uv, fill=color, width=width)


# ----------------------------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------------------------
def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    repo = Path(sys.argv[1]).resolve()
    s = read_settings(repo)
    urdf = repo / "src/arm_sim/urdf/fr3.urdf"
    pin, full, model = load_arm(urdf)
    data = model.createData()
    base = np.array(s["spawn_xyz"])
    names = [model.names[i] for i in range(1, model.njoints)]
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    eff = model.upperEffortLimit

    # reachable workspace (TCP) by sampling within the joint limits
    rng = np.random.default_rng(0)
    tcp = model.getFrameId("fr3_hand_tcp")
    samples = []
    for _ in range(20000):
        q = lo + rng.random(model.nq) * (hi - lo)
        pin.framesForwardKinematics(model, data, q)
        samples.append(data.oMf[tcp].translation + base)
    samples = np.array(samples)
    radial = np.hypot(samples[:, 0] - base[0], samples[:, 1] - base[1])
    reach = float(np.percentile(radial, 99.9))
    z_top = float(samples[:, 2].max())

    px, py, pz = s["platform_pose"][:3]
    sx, sy, sz = s["platform_size"]
    plat_top = pz + sz / 2

    cams = [
        Camera("overview", eye=(2.55, -2.05, 1.75), target=(0.05, 0.0, 0.62), hfov=0.95),
        Camera("joints", eye=(0.04, -2.5, 1.0), target=(0.04, 0.0, 0.82), hfov=0.72),
        # high and narrow, so the directional light's reflection on the ground stays out of frame
        Camera("top", eye=(base[0], base[1], 9.0), target=None, hfov=0.47, top_down=True),
    ]

    t_start = time.time()
    with tempfile.TemporaryDirectory(prefix="fr3_media_") as tmp:
        overlay = build_overlay(repo, cams, Path(tmp))
        settle = s["traj_duration"] + 3.0
        log_path = Path(tempfile.gettempdir()) / "fr3_media_launch.log"
        images, q_meas = capture(cams, overlay, settle_s=settle, log_path=log_path)
    q = np.array([q_meas[n] for n in names])

    pin.framesForwardKinematics(model, data, q)
    joint_pos = {names[i - 1]: data.oMi[i].translation + base for i in range(1, model.njoints)}
    joint_axis = {names[i - 1]: data.oMi[i].rotation[:, 2] for i in range(1, model.njoints)}  # FR3 joints rotate about z

    # Marker point for each joint. Some FR3 joint frames share an origin (joint1/joint2 at the
    # shoulder, joint5/joint6 at the wrist), and joint1's frame sits 0.333 m up its axis. Those
    # joints are marked further along their own rotation axis, towards the actuator housing.
    def along_axis(joint, towards, dist):
        o, a = joint_pos[joint], joint_axis[joint]
        return o + a * dist * np.sign(np.dot(a, np.asarray(towards) - o))
    marker = dict(joint_pos)
    marker["fr3_joint1"] = along_axis("fr3_joint1", base, 0.18)
    marker["fr3_joint3"] = along_axis("fr3_joint3", joint_pos["fr3_joint2"], 0.15)
    marker["fr3_joint5"] = along_axis("fr3_joint5", joint_pos["fr3_joint4"], 0.18)
    tcp_pos = data.oMf[tcp].translation + base
    hand_pos = data.oMf[model.getFrameId("fr3_hand")].translation + base

    pose_note = (f"pose: home + {s['traj_offset']} rad on {s['traj_joint']} "
                 f"(default {s['traj_duration']:.0f} s trajectory), held by the PD + gravity controller")
    foot = f"Gazebo Sim (ogre2) | full_system.launch.py | controller at {s['update_rate']} Hz"

    # ---- 1. workspace overview -----------------------------------------------------------------
    cam = cams[0]
    im = Image.fromarray(images["overview"]).convert("RGBA")
    d = ImageDraw.Draw(im, "RGBA")
    top = caption(d, W, "FR3 simulation workspace",
                  f"Franka FR3 + Franka Hand on a {sx:.2f} × {sy:.2f} × {sz:.2f} m platform, fr3_world in Gazebo Sim")
    callouts(d, W, H, [
        (cam.project(joint_pos["fr3_joint4"]), "Franka FR3 arm", "7 revolute joints, torque-controlled"),
        (cam.project(hand_pos), "Franka Hand", "2 fingers, 0–0.04 m each"),
        (cam.project(base), "Robot base (fr3_link0)",
         f"fixed at ({base[0]:.1f}, {base[1]:.1f}, {base[2]:.2f}) m"),
        (cam.project((px + sx / 2 * 0.35, py - sy / 2, plat_top - sz * 0.45)), "Robot platform",
         f"{sx:.2f} × {sy:.2f} × {sz:.2f} m, top at z = {plat_top:.2f} m"),
        (cam.project((-3.0, 1.0, 0.0)), "Ground plane", "z = 0 m, world origin below the base", "L"),
    ], top)
    footer(d, W, H, foot)
    im.convert("RGB").save(OUT / "fr3_workspace_overview.png")

    # ---- 2. joint diagram ------------------------------------------------------------------------
    cam = cams[1]
    im = Image.fromarray(images["joints"]).convert("RGBA")
    d = ImageDraw.Draw(im, "RGBA")
    top = caption(d, W, "FR3 joints and tool frame", pose_note)
    items = []
    for i, n in enumerate(names):
        rng_txt = f"{math.degrees(lo[i]):+.0f}° to {math.degrees(hi[i]):+.0f}°,  {eff[i]:.0f} Nm max"
        # joints 1-5 (base to forearm) on the left, the wrist joints 6-7 and the TCP on the right
        items.append((cam.project(marker[n]), n, rng_txt, "L" if i < 5 else "R"))
    items.append((cam.project(tcp_pos), "fr3_hand_tcp", "tool centre point (between the fingertips)", "R"))
    callouts(d, W, H, items, top)
    footer(d, W, H, "joint markers: Pinocchio forward kinematics of the measured /joint_states, placed on each joint's axis")
    im.convert("RGB").save(OUT / "fr3_joint_diagram.png")

    # ---- 3. top view with reach circle -------------------------------------------------------------
    cam = cams[2]
    im = Image.fromarray(images["top"]).convert("RGBA")
    d = ImageDraw.Draw(im, "RGBA")
    ang = np.linspace(0, 2 * np.pi, 181)
    circle = [(base[0] + reach * math.cos(a), base[1] + reach * math.sin(a), plat_top) for a in ang]
    polyline_3d(d, cam, circle, (42, 120, 214, 255), 4)
    corners = [(px + ax * sx / 2, py + ay * sy / 2, plat_top) for ax, ay in [(-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)]]
    polyline_3d(d, cam, corners, (235, 104, 52, 255), 3)
    top = caption(d, W, "Top view: platform and arm reach",
                  "blue circle: horizontal reach of fr3_hand_tcp")
    callouts(d, W, H, [
        (cam.project(circle[30]), f"Reach radius ≈ {reach:.2f} m", "99.9th percentile of 20,000 sampled poses"),
        (cam.project(corners[1]), "Platform edge", f"{sx:.2f} × {sy:.2f} m"),
        (cam.project(base), "Joint-1 axis", "base of the arm"),
        (cam.project(tcp_pos), "Tool centre point", "current pose"),
    ], top)
    footer(d, W, H, "top-down camera; no self-collision or platform-collision check in the reach estimate")
    im.convert("RGB").save(OUT / "fr3_top_view_reach.png")

    # ---- plots -----------------------------------------------------------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def style(ax, title, xlabel, ylabel):
        ax.set_title(title, loc="left", fontweight="bold", fontsize=14, pad=12)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, color="#e6e6e6", linewidth=0.8)
        ax.set_axisbelow(True)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)

    def foot_note(fig, text):
        fig.text(0.01, 0.01, text, color=GREY, fontsize=8.5, ha="left")

    home = np.array([s["home"][n] for n in names])

    # joint ranges
    fig, ax = plt.subplots(figsize=(10, 5.2), dpi=160)
    yy = np.arange(len(names))[::-1]
    ax.barh(yy, np.degrees(hi - lo), left=np.degrees(lo), height=0.5, color=BLUE, alpha=0.85, label="joint range")
    ax.scatter(np.degrees(home), yy, color=ORANGE, zorder=3, s=46, label="home pose")
    for y, l, h_, hm in zip(yy, lo, hi, home):
        ax.text(math.degrees(h_) + 4, y, f"{math.degrees(l):+.0f}° … {math.degrees(h_):+.0f}°", va="center", fontsize=9)
    ax.set_yticks(yy, names)
    ax.axvline(0, color="#bdbdbd", linewidth=1)
    style(ax, "FR3 joint ranges and home pose", "joint angle (deg)", "")
    ax.set_xlim(-200, 320)
    ax.legend(frameon=False, loc="lower right")
    foot_note(fig, "limits: arm_sim/urdf/fr3.urdf  |  home pose: arm_sim/launch/sim.launch.py spawn arguments")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(OUT / "plot_joint_ranges.png")
    plt.close(fig)

    # reachable workspace: side and top
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    def workspace_legend(ax, loc):
        handles = [Line2D([], [], marker="o", linestyle="", color=BLUE, markersize=6, label="reachable TCP positions"),
                   Patch(facecolor="#d9d9d9", edgecolor=ORANGE, linewidth=2, label="platform"),
                   Line2D([], [], marker="o", linestyle="", color=AQUA, markersize=8, label="robot base")]
        ax.legend(handles=handles, frameon=True, facecolor="white", edgecolor="#e0e0e0", framealpha=0.95, loc=loc)

    for view in ("side", "top"):
        fig, ax = plt.subplots(figsize=(8, 7) if view == "top" else (9, 6.5), dpi=160)
        if view == "side":
            ax.scatter(samples[:, 0], samples[:, 2], s=1.2, color=BLUE, alpha=0.18, linewidths=0, label="reachable TCP positions")
            ax.add_patch(plt.Rectangle((px - sx / 2, pz - sz / 2), sx, sz, facecolor="#d9d9d9", edgecolor=ORANGE, linewidth=2, label="platform"))
            ax.axhline(0, color="#7a7a7a", linewidth=1.5)
            ax.text(px + sx / 2 + 0.03, 0.02, "ground (z = 0)", color="#7a7a7a", fontsize=9, ha="left", va="bottom")
            ax.scatter([base[0]], [base[2]], color=AQUA, s=60, zorder=4, label="robot base")
            ax.annotate(f"max height {z_top:.2f} m", (samples[np.argmax(samples[:, 2]), 0], z_top),
                        xytext=(25, -5), textcoords="offset points", fontsize=9)
            style(ax, "FR3 reachable workspace: side view (x–z)", "x (m)", "z (m)")
        else:
            ax.scatter(samples[:, 0], samples[:, 1], s=1.2, color=BLUE, alpha=0.18, linewidths=0, label="reachable TCP positions")
            ax.add_patch(plt.Rectangle((px - sx / 2, py - sy / 2), sx, sy, facecolor="none", edgecolor=ORANGE, linewidth=2, label="platform"))
            ax.add_patch(plt.Circle(base[:2], reach, fill=False, color="#1b4f8f", linestyle="--", linewidth=1.5))
            ax.annotate(f"reach ≈ {reach:.2f} m", (base[0] + reach * 0.71, base[1] + reach * 0.71),
                        xytext=(10, 10), textcoords="offset points", fontsize=9)
            ax.scatter([base[0]], [base[1]], color=AQUA, s=60, zorder=4, label="robot base")
            ax.set_aspect("equal")
            style(ax, "FR3 reachable workspace: top view (x–y)", "x (m)", "y (m)")
        if view == "side":
            ax.set_aspect("equal")
        workspace_legend(ax, "upper left")
        foot_note(fig, "20,000 uniform samples within the URDF joint limits; fingers locked; "
                       "no self-collision or platform-collision check")
        fig.tight_layout(rect=(0, 0.03, 1, 1))
        fig.savefig(OUT / f"plot_reachable_workspace_{view}.png")
        plt.close(fig)

    print(f"done in {time.time() - t_start:.0f} s; reach ≈ {reach:.3f} m, max TCP height {z_top:.3f} m")
    print("measured pose (rad):", dict(zip(names, np.round(q, 4))))


if __name__ == "__main__":
    main()
