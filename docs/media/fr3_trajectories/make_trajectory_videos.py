#!/usr/bin/env python3
"""Record the FR3 tracking Cartesian shapes in simulation: videos, plots, trajectories and logs.

Usage (zsh, no conda env active, ROS and the built workspace sourced):

    source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
    python3 docs/media/fr3_trajectories/make_trajectory_videos.py .
    python3 docs/media/fr3_trajectories/make_trajectory_videos.py . circle square   # only some shapes

How it works
- Runs the project's own `arm_bringup moveit_system.launch.py` (headless): Gazebo, controller_node,
  trajectory_action_server and move_group, unchanged. RViz is not opened.
- No planner of its own: each shape is a list of tool (fr3_hand_tcp) waypoints, planned by MoveIt's
  /compute_cartesian_path (straight lines between waypoints) and executed through MoveIt's
  /execute_trajectory, i.e. trajectory_action_server -> controller_node -> Gazebo. MoveIt's timing
  is stretched uniformly so the tool speed stays below TCP_SPEED (see limit_tool_speed).
- Between shapes the arm moves back to the home tool pose the same way.
- A camera is added through the same temporary arm_sim overlay as ../fr3_workspace/make_media.py.
  The desired path and the measured tool trail are projected onto each video frame.
- Runs on its own ROS domain and Gazebo partition, so it cannot disturb a simulation you have open.

Outputs (per shape <key>)
- <key>.mp4                  video of the execution, 1280x720, 30 fps of simulation time
- <key>_final.png            last video frame (complete trail)
- <key>_plot.png             tool path (top view), tool tracking error, joint tracking errors
- trajectories/<key>.csv     the planned joint trajectory (t, q, qd, qdd for fr3_joint1..7)
- trajectories/<key>_goal.yaml   the same trajectory as a FollowJointTrajectory goal
- logs/<key>.csv             100 Hz log: desired and measured joints and tool positions
- summary.png, results.json  metrics for all shapes
"""
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parent / "fr3_workspace"))
from make_media import FONT, FONT_BOLD, Camera, build_overlay, load_arm, read_settings  # noqa: E402

ROS_DOMAIN = "87"
GZ_PARTITION = "fr3_trajectory_videos"
VW, VH, FPS = 1280, 720, 30

# Shape plane: horizontal (x-y) at a fixed height, in the robot base frame (MoveIt's planning
# frame "world"); the hand points down, as when drawing on a table. Shape coordinates (h, v) are
# offsets along x and y from CENTER.
CENTER = np.array([0.40, 0.0, 0.40])
HALF = 0.10                      # half the shape size (m): 0.20 m square, 0.10 m radius circle
TCP_SPEED = 0.08                 # tool speed limit along the path (m/s)
VEL_SCALE, ACC_SCALE = 0.5, 0.3  # joint velocity / acceleration scaling for MoveIt's timing
ORIENT = (1.0, 0.0, 0.0, 0.0)    # tool orientation (x, y, z, w): hand pointing down, as at home
GROUP, TCP = "fr3_arm", "fr3_hand_tcp"

# chart colours (categorical slots 1-8, chrome) from the reference palette
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6"]
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"


def shape_defs(a):
    n = 120
    circle = [(a * math.cos(t), a * math.sin(t)) for t in np.linspace(0.0, 2 * math.pi, n + 1)]
    hs = np.linspace(-a, a, n + 1)
    curve = [(h, 0.6 * a * math.sin(math.pi * h / a)) for h in hs]
    return {
        "line": ("Two-point line", f"diagonal, {2 * a * math.sqrt(2):.2f} m long", [(-a, -a), (a, a)]),
        "square": ("Square", f"{2 * a:.2f} m side", [(-a, -a), (a, -a), (a, a), (-a, a), (-a, -a)]),
        "diamond": ("Diamond", f"{2 * a:.2f} m diagonals", [(0, -a), (a, 0), (0, a), (-a, 0), (0, -a)]),
        "circle": ("Circle", f"{a:.2f} m radius", circle),
        "curve": ("Curved path", f"one sine period, {2 * a:.2f} m wide", curve),
    }


def plane_to_base(h, v):
    return np.array([CENTER[0] + h, CENTER[1] + v, CENTER[2]])


def read_gains(repo):
    src = (repo / "src/arm_control/src/controller_node.cpp").read_text()
    def vec(name):
        m = re.search(rf"{name}\s*=\s*\(Eigen::VectorXd\(\d+\)\s*<<\s*([^)]*)\)", src)
        return [float(x) for x in m.group(1).split(",")]
    rate = re.search(r"update_rate:\s*(\d+)", (repo / "src/arm_control/config/gz_controllers.yaml").read_text())
    return {"Kp": vec("Kp"), "Kd": vec("Kd"), "update_rate_hz": int(rate.group(1))}


class VideoCamera(Camera):
    def sdf(self):
        return super().sdf().replace("<update_rate>2</update_rate>", f"<update_rate>{FPS}</update_rate>")


# ----------------------------------------------------------------------------------------------
# kinematics
# ----------------------------------------------------------------------------------------------
class Kin:
    def __init__(self, urdf):
        self.pin, _, self.model = load_arm(urdf)
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(TCP)
        self.names = list(self.model.names)[1:]

    def tcp(self, q):
        self.pin.framesForwardKinematics(self.model, self.data, np.asarray(q, float))
        return self.data.oMf[self.fid].translation.copy()

    def tcp_speed(self, q, qd):
        J = self.pin.computeFrameJacobian(self.model, self.data, np.asarray(q, float), self.fid,
                                          self.pin.LOCAL_WORLD_ALIGNED)
        return float(np.linalg.norm(J[:3] @ np.asarray(qd, float)))


def limit_tool_speed(sol, kin, limit):
    """Slow MoveIt's trajectory down uniformly so the tool speed stays <= limit.

    MoveIt's /compute_cartesian_path does not apply max_cartesian_speed here, so the timing is
    stretched by k (times x k, velocities / k, accelerations / k^2). The path and its smoothness
    are unchanged. Returns (k, peak tool speed after scaling)."""
    jt = sol.joint_trajectory
    idx = [list(jt.joint_names).index(n) for n in kin.names]
    peak = max(kin.tcp_speed([p.positions[i] for i in idx], [p.velocities[i] for i in idx])
               for p in jt.points)
    k = max(1.0, peak / limit)
    for p in jt.points:
        t = (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9) * k
        p.time_from_start.sec = int(t)
        p.time_from_start.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
        p.velocities = [v / k for v in p.velocities]
        p.accelerations = [a / (k * k) for a in p.accelerations]
    return k, peak / k


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t * ab))


def path_deviation(points, polyline):
    return np.array([min(seg_dist(p, polyline[i], polyline[i + 1]) for i in range(len(polyline) - 1))
                     for p in points])


# ----------------------------------------------------------------------------------------------
# ROS side: planning, execution and recording
# ----------------------------------------------------------------------------------------------
def make_recorder(kin, cam, spawn):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage, JointState
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.srv import GetCartesianPath

    font_b = ImageFont.truetype(FONT_BOLD, 26)
    font = ImageFont.truetype(FONT, 20)

    class Recorder(Node):
        def __init__(self):
            super().__init__("fr3_trajectory_recorder")
            self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
            self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
            self.create_subscription(JointState, "/joint_states", self.on_js, 50)
            self.create_subscription(JointState, "/desired_joint_states", self.on_des, 50)
            self.create_subscription(RosImage, f"/{cam.topic}", self.on_img, 30)
            self.q = self.q_des = None
            self.recording = False

        @staticmethod
        def stamp(m):
            return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        def pick(self, m):
            idx = {n: i for i, n in enumerate(m.name)}
            if not all(n in idx for n in kin.names):
                return None
            return np.array([m.position[idx[n]] for n in kin.names])

        def on_js(self, m):
            q = self.pick(m)
            if q is None:
                return
            self.q = q
            if self.recording:
                self.js_log.append((self.stamp(m), q))

        def on_des(self, m):
            q = self.pick(m)
            if q is None:
                return
            self.q_des = q
            if self.recording:
                self.des_log.append((self.stamp(m), q))

        # -- video ------------------------------------------------------------------------
        def start_recording(self, key, title, subtitle, ideal_world, path):
            self.js_log, self.des_log, self.trail = [], [], []
            self.title, self.subtitle, self.ideal_px = title, subtitle, [cam.project(p) for p in ideal_world]
            self.t0 = self.next_t = None
            self.last_frame = None
            self.ffmpeg = subprocess.Popen(
                ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", f"{VW}x{VH}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium",
                 "-crf", "26", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
                stdin=subprocess.PIPE)
            self.recording = True

        def stop_recording(self):
            self.recording = False
            self.ffmpeg.stdin.close()
            self.ffmpeg.wait()
            return self.last_frame

        def on_img(self, m):
            if not self.recording or self.q is None:
                return
            t = self.stamp(m)
            if self.t0 is None:
                self.t0 = self.next_t = t
            if t < self.next_t - 0.5 / FPS:
                return
            tcp = kin.tcp(self.q) + spawn
            self.trail.append(cam.project(tcp))
            err_mm = 1000 * np.linalg.norm(kin.tcp(self.q) - kin.tcp(self.q_des)) if self.q_des is not None else 0.0

            im = Image.frombuffer("RGB", (m.width, m.height), bytes(m.data), "raw", "RGB", 0, 1).convert("RGBA")
            d = ImageDraw.Draw(im, "RGBA")
            d.line(self.ideal_px, fill=(42, 120, 214, 255), width=4, joint="curve")
            if len(self.trail) > 1:
                d.line(self.trail, fill=(235, 104, 52, 255), width=4, joint="curve")
            u, v = self.trail[-1]
            d.ellipse([u - 7, v - 7, u + 7, v + 7], fill=(235, 104, 52, 255), outline=(255, 255, 255, 255), width=2)
            # caption
            d.rounded_rectangle([18, 16, 560, 128], 10, fill=(10, 10, 10, 170))
            d.text((34, 26), f"FR3 · {self.title}", font=font_b, fill="white")
            d.text((34, 62), self.subtitle, font=font, fill=(230, 230, 230))
            d.text((34, 92), f"t = {t - self.t0:5.2f} s     tool tracking error {err_mm:5.2f} mm",
                   font=font, fill=(230, 230, 230))
            # legend
            d.rounded_rectangle([VW - 330, 16, VW - 18, 92], 10, fill=(10, 10, 10, 170))
            d.line([(VW - 314, 40), (VW - 270, 40)], fill=(42, 120, 214, 255), width=5)
            d.text((VW - 260, 28), "desired tool path", font=font, fill="white")
            d.line([(VW - 314, 70), (VW - 270, 70)], fill=(235, 104, 52, 255), width=5)
            d.text((VW - 260, 58), "measured tool path", font=font, fill="white")
            frame = im.convert("RGB")
            raw = frame.tobytes()
            while self.next_t <= t + 0.5 / FPS:
                self.ffmpeg.stdin.write(raw)
                self.next_t += 1.0 / FPS
            self.last_frame = frame

        # -- MoveIt calls -----------------------------------------------------------------
        def spin_for(self, seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.02)

        def plan(self, waypoints_base):
            from geometry_msgs.msg import Pose
            req = GetCartesianPath.Request()
            req.header.frame_id = "world"
            req.start_state.is_diff = True
            req.group_name, req.link_name = GROUP, TCP
            for p in waypoints_base:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, p)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = ORIENT
                req.waypoints.append(pose)
            req.max_step = 0.005
            req.jump_threshold = 0.0
            req.avoid_collisions = True
            req.max_velocity_scaling_factor = VEL_SCALE
            req.max_acceleration_scaling_factor = ACC_SCALE
            req.cartesian_speed_limited_link = TCP
            req.max_cartesian_speed = TCP_SPEED
            fut = self.cartesian.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            res = fut.result()
            if res is None or res.fraction < 0.999:
                raise RuntimeError(f"Cartesian planning failed (fraction {getattr(res, 'fraction', None)})")
            return res.solution

        def run(self, solution):
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = solution
            fut = self.execute.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("execute_trajectory goal rejected")
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut, timeout_sec=120.0)
            code = res_fut.result().result.error_code.val
            if code != 1:
                raise RuntimeError(f"execution failed with MoveIt error code {code}")

    return rclpy, Recorder


# ----------------------------------------------------------------------------------------------
# outputs
# ----------------------------------------------------------------------------------------------
def save_trajectory(key, sol, names):
    jt = sol.joint_trajectory
    idx = [list(jt.joint_names).index(n) for n in names]
    (OUT / "trajectories").mkdir(exist_ok=True)
    rows = []
    for p in jt.points:
        t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        get = lambda arr: [arr[i] if len(arr) else 0.0 for i in idx]
        rows.append([t] + get(p.positions) + get(p.velocities) + get(p.accelerations))
    hdr = (["t"] + [f"q_{n}" for n in names] + [f"qd_{n}" for n in names] + [f"qdd_{n}" for n in names])
    np.savetxt(OUT / "trajectories" / f"{key}.csv", np.array(rows), delimiter=",",
               header=",".join(hdr), comments="", fmt="%.6f")
    pts = []
    for r in rows:
        sec = int(r[0]); nsec = int(round((r[0] - sec) * 1e9))
        pts.append("    - {positions: [%s], velocities: [%s], time_from_start: {sec: %d, nanosec: %d}}"
                   % (", ".join(f"{x:.6f}" for x in r[1:8]), ", ".join(f"{x:.6f}" for x in r[8:15]), sec, nsec))
    (OUT / "trajectories" / f"{key}_goal.yaml").write_text(
        "# FollowJointTrajectory goal planned by MoveIt for this shape; it starts at the shape's start pose.\n"
        "# Send with: ros2 action send_goal /fr3_arm_controller/follow_joint_trajectory "
        "control_msgs/action/FollowJointTrajectory \"$(cat <this file>)\"\n"
        "trajectory:\n  joint_names: [%s]\n  points:\n%s\n" % (", ".join(names), "\n".join(pts)))
    return np.array(rows)


def analyse(key, kin, js_log, des_log, ideal_base, t_exec):
    tj = np.array([t for t, _ in js_log]); qa = np.array([q for _, q in js_log])
    td = np.array([t for t, _ in des_log]); qd = np.array([q for _, q in des_log])
    t0 = td[0]
    qdi = np.column_stack([np.interp(tj, td, qd[:, j]) for j in range(qd.shape[1])])
    keep = (tj >= td[0]) & (tj <= td[-1])
    tj, qa, qdi = tj[keep] - t0, qa[keep], qdi[keep]
    pa = np.array([kin.tcp(q) for q in qa]); pd = np.array([kin.tcp(q) for q in qdi])
    track = np.linalg.norm(pa - pd, axis=1)
    jerr = qdi - qa
    moving = tj <= t_exec
    dev = path_deviation(pa[moving], ideal_base)
    metrics = {
        "duration_s": round(float(t_exec), 3),
        "tool_tracking_error_mm": {"max": round(1000 * float(track[moving].max()), 3),
                                   "rms": round(1000 * float(np.sqrt(np.mean(track[moving] ** 2))), 3)},
        "tool_path_deviation_mm": {"max": round(1000 * float(dev.max()), 3),
                                   "rms": round(1000 * float(np.sqrt(np.mean(dev ** 2))), 3)},
        "joint_tracking_error_mrad_max": {n: round(1000 * float(np.abs(jerr[moving, j]).max()), 3)
                                          for j, n in enumerate(kin.names)},
        "final_tool_error_mm": round(1000 * float(track[-1]), 3),
    }
    # 100 Hz log
    (OUT / "logs").mkdir(exist_ok=True)
    sel = np.searchsorted(tj, np.arange(0.0, tj[-1], 0.01))
    sel = sel[sel < len(tj)]
    cols = np.column_stack([tj[sel], qdi[sel], qa[sel], pd[sel], pa[sel]])
    hdr = (["t"] + [f"q_des_{n}" for n in kin.names] + [f"q_meas_{n}" for n in kin.names]
           + ["tcp_des_x", "tcp_des_y", "tcp_des_z", "tcp_meas_x", "tcp_meas_y", "tcp_meas_z"])
    np.savetxt(OUT / "logs" / f"{key}.csv", cols, delimiter=",", header=",".join(hdr), comments="", fmt="%.6f")
    return metrics, tj, pa, track, jerr


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.xaxis.label.set_color(INK2); ax.yaxis.label.set_color(INK2)
    ax.title.set_color(INK)


def plot_shape(key, title, subtitle, ideal_plane, kin, tj, pa, track, jerr, t_exec, metrics):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(15, 4.8), dpi=130, facecolor=SURFACE)
    gs = fig.add_gridspec(1, 3, width_ratios=[1.0, 1.25, 1.25], wspace=0.28)
    ax0, ax1, ax2 = (fig.add_subplot(gs[0, i]) for i in range(3))

    h = pa[:, 0] - CENTER[0]; v = pa[:, 1] - CENTER[1]
    ih, iv = zip(*ideal_plane)
    ax0.plot(np.array(ih) * 1000, np.array(iv) * 1000, color=SERIES[0], lw=2, ls="--", label="desired")
    ax0.plot(h * 1000, v * 1000, color=SERIES[1], lw=2, label="measured")
    ax0.set_aspect("equal"); ax0.set_xlabel("x offset (mm)"); ax0.set_ylabel("y offset (mm)")
    ax0.set_title(f"Tool path, top view (z = {CENTER[2]:.2f} m)", fontsize=11, loc="left")
    ax0.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="upper left")
    pad = HALF * 1000 * 1.25
    ax0.set_xlim(-pad, pad); ax0.set_ylim(-pad, pad + 25)

    ax1.plot(tj, track * 1000, color=SERIES[0], lw=2)
    ax1.axvline(t_exec, color=AXIS, lw=1, ls=":")
    ax1.text(t_exec, ax1.get_ylim()[1] * 0.96, " trajectory ends", color=MUTED, fontsize=8, va="top")
    ax1.set_xlabel("time (s)"); ax1.set_ylabel("error (mm)")
    ax1.set_title("Tool tracking error |desired − measured|", fontsize=11, loc="left")

    for j, n in enumerate(kin.names):
        ax2.plot(tj, jerr[:, j] * 1000, color=SERIES[j], lw=1.6, label=n.replace("fr3_", ""))
    ax2.axvline(t_exec, color=AXIS, lw=1, ls=":")
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("error (mrad)")
    ax2.set_title("Joint tracking error (desired − measured)", fontsize=11, loc="left")
    ax2.legend(frameon=False, fontsize=8, labelcolor=INK2, ncol=4, loc="upper right")

    for ax in (ax0, ax1, ax2):
        style(ax)
    m = metrics
    fig.suptitle(f"FR3 · {title} ({subtitle}) — max tool tracking error {m['tool_tracking_error_mm']['max']:.2f} mm, "
                 f"RMS {m['tool_tracking_error_mm']['rms']:.2f} mm, max path deviation "
                 f"{m['tool_path_deviation_mm']['max']:.2f} mm", x=0.01, ha="left", fontsize=12, color=INK)
    fig.savefig(OUT / f"{key}_plot.png", bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)


def plot_summary(results, defs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    keys = list(results)
    labels = [defs[k][0] for k in keys]
    mx = [results[k]["tool_tracking_error_mm"]["max"] for k in keys]
    rms = [results[k]["tool_tracking_error_mm"]["rms"] for k in keys]
    x = np.arange(len(keys)); w = 0.34
    fig, ax = plt.subplots(figsize=(9, 4.2), dpi=130, facecolor=SURFACE)
    b1 = ax.bar(x - w / 2 - 0.01, mx, w, color=SERIES[0], label="max")
    b2 = ax.bar(x + w / 2 + 0.01, rms, w, color=SERIES[1], label="RMS")
    for bars in (b1, b2):
        for b in bars:
            ax.text(b.get_x() + b.get_width() / 2, b.get_height(), f"{b.get_height():.2f}",
                    ha="center", va="bottom", fontsize=8, color=INK2)
    ax.set_xticks(x, labels)
    ax.set_ylabel("tool tracking error (mm)")
    ax.set_title("Tool tracking error per shape while the trajectory runs", fontsize=11, loc="left")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2)
    style(ax)
    fig.savefig(OUT / "summary.png", bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)


# ----------------------------------------------------------------------------------------------
def main():
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    defs = shape_defs(HALF)
    wanted = sys.argv[2:] or list(defs)
    unknown = [k for k in wanted if k not in defs]
    if unknown:
        sys.exit(f"unknown shape(s) {unknown}; choose from {list(defs)}")

    settings = read_settings(repo)
    spawn = np.array(settings["spawn_xyz"])
    gains = read_gains(repo)
    kin = Kin(repo / "src/arm_sim/urdf/fr3.urdf")
    home_q = np.array([settings["home"][n] for n in kin.names])
    home_tcp = kin.tcp(home_q)

    target = CENTER + spawn + np.array([-0.12, 0.0, 0.12])
    cam = VideoCamera("traj", eye=target + np.array([1.35, -0.95, 1.05]), target=target, hfov=0.80, w=VW, h=VH)

    results = {}
    old = OUT / "results.json"
    if old.exists():
        results = json.loads(old.read_text()).get("shapes", {})

    with tempfile.TemporaryDirectory(prefix="fr3_traj_") as tmp:
        tmp = Path(tmp)
        overlay = build_overlay(repo, [cam], tmp)
        env = dict(os.environ, ROS_DOMAIN_ID=ROS_DOMAIN, GZ_PARTITION=GZ_PARTITION,
                   AMENT_PREFIX_PATH=f"{overlay}:{os.environ.get('AMENT_PREFIX_PATH', '')}")
        for var in ("DISPLAY", "WAYLAND_DISPLAY"):   # keeps RViz from opening a window
            env.pop(var, None)
        os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN
        log_path = tmp / "launch.log"
        log = open(log_path, "w")
        procs = [subprocess.Popen(["ros2", "launch", "arm_bringup", "moveit_system.launch.py", "headless:=true"],
                                  env=env, stdout=log, stderr=log, start_new_session=True)]
        time.sleep(3)
        procs.append(subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                       f"/{cam.topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
                                      env=env, stdout=log, stderr=log, start_new_session=True))
        rclpy, Recorder = make_recorder(kin, cam, spawn)
        rclpy.init()
        node = Recorder()
        try:
            print("waiting for move_group and the trajectory action server ...")
            if not node.cartesian.wait_for_service(timeout_sec=90) or not node.execute.wait_for_server(timeout_sec=90):
                raise RuntimeError(f"MoveIt did not come up (see {log_path})")
            start = time.monotonic()
            while node.q_des is None and time.monotonic() - start < 60:
                rclpy.spin_once(node, timeout_sec=0.05)
            node.spin_for(2.0)

            for key in wanted:
                title, subtitle, plane_pts = defs[key]
                ideal = [plane_to_base(h, v) for h, v in plane_pts]
                print(f"[{key}] approach")
                node.run(node.plan([ideal[0]]))
                node.spin_for(1.0)
                sol = node.plan(ideal[1:])
                k, peak = limit_tool_speed(sol, kin, TCP_SPEED)
                print(f"[{key}] timing stretched x{k:.2f}, peak tool speed {peak:.3f} m/s")
                traj = save_trajectory(key, sol, kin.names)
                t_exec = float(traj[-1, 0])
                print(f"[{key}] recording ({t_exec:.1f} s of motion)")
                node.start_recording(key, title, f"{subtitle} · tool speed ≤ {TCP_SPEED:.2f} m/s",
                                     [p + spawn for p in ideal], OUT / f"{key}.mp4")
                node.run(sol)
                node.spin_for(1.5)
                final = node.stop_recording()
                if final is not None:
                    final.save(OUT / f"{key}_final.png")
                metrics, tj, pa, track, jerr = analyse(key, kin, node.js_log, node.des_log, ideal, t_exec)
                plot_shape(key, title, subtitle, plane_pts, kin, tj, pa, track, jerr, t_exec, metrics)
                results[key] = metrics
                print(f"[{key}] max tool tracking error {metrics['tool_tracking_error_mm']['max']:.2f} mm")
                print(f"[{key}] return home")
                node.run(node.plan([home_tcp]))
                node.spin_for(1.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()
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

    ordered = {k: results[k] for k in defs if k in results}
    plot_summary(ordered, defs)
    (OUT / "results.json").write_text(json.dumps({
        "settings": {"shape_plane_center_base_m": CENTER.tolist(), "shape_half_size_m": HALF,
                     "tool_speed_limit_m_s": TCP_SPEED, "velocity_scaling": VEL_SCALE,
                     "acceleration_scaling": ACC_SCALE, "tool_frame": TCP, "controller": gains},
        "shapes": ordered}, indent=2) + "\n")
    print(f"done: outputs in {OUT}")


if __name__ == "__main__":
    main()
