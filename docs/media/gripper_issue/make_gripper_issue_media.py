#!/usr/bin/env python3
"""Record the Franka Hand gripper problem in simulation: a close-up video, stills and plots.

Usage (zsh, no conda env active, ROS and the built workspace sourced):

    source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
    python3 docs/media/gripper_issue/make_gripper_issue_media.py .            # A once, B three times
    python3 docs/media/gripper_issue/make_gripper_issue_media.py . --only-b --b-runs=5

Scenario A (full MoveIt stack, arm held by the controller), recorded on video:
  MoveIt open -> MoveIt closed -> direct open (0.04) -> direct close to exactly 0.0
  -> MoveIt open (expected to open; times out, fingers do not move) -> direct open (no motion).
Scenario B (sim.launch.py alone, nothing holds the arm), finger log only, repeated (it is intermittent):
  direct open (0.04) -> direct close (0.0) -> direct close (0.001) -> direct open (0.04).

Uses the same temporary arm_sim overlay (Sensors system + camera) as ../fr3_workspace/make_media.py.
Runs on its own ROS domain and Gazebo partition, so it cannot disturb a simulation you have open.
"""
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parent / "fr3_workspace"))
from make_media import FONT, FONT_BOLD, Camera, build_overlay, load_arm, read_settings  # noqa: E402

ROS_DOMAIN = "88"
GZ_PARTITION = "fr3_gripper_issue"
VW, VH, FPS = 1280, 720, 30
F1, F2 = "fr3_finger_joint1", "fr3_finger_joint2"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"

SCENARIO_A = [
    ("moveit", 0.04, "MoveIt: hand -> open"),
    ("moveit", 0.0, "MoveIt: hand -> closed"),
    ("direct", 0.04, "Direct gripper goal: 0.04 (open)"),
    ("direct", 0.0, "Direct gripper goal: 0.0 (close to the joint limit)"),
    ("moveit", 0.04, "MoveIt: hand -> open (after the direct 0.0)"),
    ("direct", 0.04, "Direct gripper goal: 0.04 (open)"),
]
SCENARIO_B = [
    ("direct", 0.04, "direct 0.04"),
    ("direct", 0.0, "direct 0.0"),
    ("direct", 0.001, "direct 0.001"),
    ("direct", 0.04, "direct 0.04"),
]


class VideoCamera(Camera):
    def sdf(self):
        return super().sdf().replace("<update_rate>2</update_rate>", f"<update_rate>{FPS}</update_rate>")


def start(cmd, env, log):
    return subprocess.Popen(cmd, env=env, stdout=log, stderr=log, start_new_session=True)


def stop(procs):
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


def make_node(cam=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage, JointState
    from control_msgs.action import ParallelGripperCommand
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint

    font_b = ImageFont.truetype(FONT_BOLD, 26)
    font = ImageFont.truetype(FONT, 21)
    font_s = ImageFont.truetype(FONT, 18)

    class Rec(Node):
        def __init__(self):
            super().__init__("gripper_issue_recorder")
            self.last, self.log = {}, []
            self.sim_t = 0.0
            self.caption = ("", "", "")
            self.ffmpeg = None
            self.frame = None
            self.next_t = None
            self.create_subscription(JointState, "/joint_states", self.on_js, 100)
            if cam is not None:
                self.create_subscription(RosImage, f"/{cam.topic}", self.on_img, 30)
            self.grip = ActionClient(self, ParallelGripperCommand, "/hand_controller/gripper_cmd")
            self.mg = ActionClient(self, MoveGroup, "/move_action")

        def on_js(self, m):
            d = dict(zip(m.name, m.position))
            if F1 in d and F2 in d:
                self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                self.last = d
                self.log.append((self.sim_t, d[F1], d[F2]))

        def start_video(self, path):
            self.ffmpeg = subprocess.Popen(
                ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", f"{VW}x{VH}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium",
                 "-crf", "24", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
                stdin=subprocess.PIPE)

        def stop_video(self):
            f, self.ffmpeg = self.ffmpeg, None
            if f:
                f.stdin.close()
                f.wait()

        def on_img(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            im = Image.frombuffer("RGB", (m.width, m.height), bytes(m.data), "raw", "RGB", 0, 1).convert("RGBA")
            d = ImageDraw.Draw(im, "RGBA")
            title, cmd, result = self.caption
            d.rounded_rectangle([18, 16, VW - 18, 150], 10, fill=(10, 10, 10, 175))
            d.text((34, 26), title, font=font_b, fill="white")
            d.text((34, 66), cmd, font=font, fill=(230, 230, 230))
            col = (255, 140, 120) if ("TIMED_OUT" in result or "no motion" in result or "stalled" in result) else (170, 230, 170)
            d.text((34, 102), result, font=font, fill=col if result else (230, 230, 230))
            if self.last:
                d.rounded_rectangle([18, VH - 70, 560, VH - 18], 10, fill=(10, 10, 10, 175))
                d.text((34, VH - 58), f"finger 1: {self.last[F1]*1000:5.1f} mm    finger 2: {self.last[F2]*1000:5.1f} mm",
                       font=font_s, fill="white")
            frame = im.convert("RGB")
            self.frame = frame
            if self.ffmpeg is None:
                return
            if self.next_t is None:
                self.next_t = t
            raw = frame.tobytes()
            while self.next_t <= t + 0.5 / FPS:
                self.ffmpeg.stdin.write(raw)
                self.next_t += 1.0 / FPS

        # -- commands ------------------------------------------------------------------------
        @staticmethod
        def wait(fut, timeout):
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.005)
            return fut.result() if fut.done() else None

        def direct(self, target, timeout=3.0):
            g = ParallelGripperCommand.Goal()
            g.command.name, g.command.position = [F1], [float(target)]
            h = self.wait(self.grip.send_goal_async(g), 5.0)
            res = self.wait(h.get_result_async(), timeout)
            if res is None:
                self.wait(h.cancel_goal_async(), 2.0)
                return "no result in %.0f s (cancelled)" % timeout
            r = res.result
            name = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(res.status, str(res.status))
            return f"{name} (reached_goal={r.reached_goal}, stalled={r.stalled})"

        def moveit(self, target):
            g = MoveGroup.Goal()
            r = g.request
            r.group_name, r.allowed_planning_time = "hand", 5.0
            r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 1.0
            r.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
                joint_name=F1, position=float(target), tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)])]
            h = self.wait(self.mg.send_goal_async(g), 5.0)
            res = self.wait(h.get_result_async(), 30.0)
            code = res.result.error_code.val
            return {1: "SUCCESS", -6: "TIMED_OUT", 99999: "FAILURE"}.get(code, f"error code {code}")

    rclpy.init()
    node = Rec()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    return rclpy, node


def run_steps(node, steps, use_caption, frames_dir=None):
    events = []
    for i, (kind, target, label) in enumerate(steps, 1):
        t0 = node.sim_t
        cmd = (f"ros2 action send_goal /hand_controller/gripper_cmd ... position: [{target}]" if kind == "direct"
               else f"MoveIt MoveGroup goal: group 'hand', fr3_finger_joint1 = {target}")
        if use_caption:
            node.caption = (f"Step {i}/{len(steps)} · {label}", cmd, "running ...")
        f_before = node.last.get(F1, float("nan"))
        result = node.direct(target) if kind == "direct" else node.moveit(target)
        time.sleep(1.0)
        f_after = node.last.get(F1, float("nan"))
        moved = abs(f_after - f_before) > 0.003
        expect_move = abs(target - f_before) > 0.003
        note = "" if moved or not expect_move else "  ->  no motion"
        if use_caption:
            node.caption = (f"Step {i}/{len(steps)} · {label}", cmd,
                            f"result: {result}{note}   (finger 1 {f_before*1000:.1f} -> {f_after*1000:.1f} mm)")
            time.sleep(1.2)
            if frames_dir is not None and node.frame is not None:
                node.frame.save(frames_dir / f"step{i}.png")
        events.append({"step": i, "kind": kind, "target": target, "label": label, "t0": t0, "t1": node.sim_t,
                       "result": result, "finger_before_mm": round(f_before * 1000, 2),
                       "finger_after_mm": round(f_after * 1000, 2), "moved": moved, "expected_motion": expect_move})
        print(f"  step {i}: {label:52s} {result}{note}")
    return events


def plot(log, events, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([x[0] for x in log]); f1 = np.array([x[1] for x in log]) * 1000; f2 = np.array([x[2] for x in log]) * 1000
    t0 = events[0]["t0"]
    keep = t >= t0 - 0.5
    fig, ax = plt.subplots(figsize=(12, 4.6), dpi=130, facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    for e in events:
        ok = e["moved"] or not e["expected_motion"]
        ax.axvspan(e["t0"] - t0, e["t1"] - t0, color=("#e1e0d9" if ok else "#f6d5d5"), alpha=0.6, lw=0)
        ax.hlines(e["target"] * 1000, e["t0"] - t0, e["t1"] - t0, colors=INK2, linestyles=":", lw=1.5)
        ax.text((e["t0"] + e["t1"]) / 2 - t0, 45, f"step {e['step']}\n{e['result'].split(' (')[0]}",
                ha="center", va="bottom", fontsize=8, color=(INK if ok else "#b3261e"))
    ax.plot(t[keep] - t0, f1[keep], color=SERIES[0], lw=2, label="finger 1 (commanded joint)")
    ax.plot(t[keep] - t0, f2[keep], color=SERIES[1], lw=2, ls="--", label="finger 2 (mimic)")
    ax.plot([], [], color=INK2, ls=":", label="commanded position")
    ax.set_ylim(-3, 56); ax.set_xlabel("simulation time (s)"); ax.set_ylabel("finger position (mm)")
    ax.set_title(title, loc="left", fontsize=11, color=INK, pad=28)
    ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.legend(frameon=False, fontsize=8.5, labelcolor=INK2, loc="lower left", bbox_to_anchor=(0.0, 1.06), ncol=3)
    steps = "\n".join(f"step {e['step']}: {e['label']}" for e in events)
    fig.text(0.01, -0.02, steps, ha="left", va="top", fontsize=8.5, color=INK2, family="monospace")
    fig.savefig(path, bbox_inches="tight", facecolor=SURFACE)
    np.savetxt(Path(path).with_suffix(".csv"), np.column_stack([t[keep] - t0, f1[keep] / 1000, f2[keep] / 1000]),
               delimiter=",", header="t,fr3_finger_joint1,fr3_finger_joint2", comments="", fmt="%.6f")
    plt.close(fig)


def side_by_side(a, b, label_a, label_b, path):
    A, B = Image.open(a), Image.open(b)
    w, h = A.size
    out = Image.new("RGB", (w * 2 + 12, h + 56), (252, 252, 251))
    out.paste(A, (0, 56)); out.paste(B, (w + 12, 56))
    d = ImageDraw.Draw(out)
    f = ImageFont.truetype(FONT_BOLD, 28)
    d.text((16, 12), label_a, font=f, fill=(11, 11, 11))
    d.text((w + 28, 12), label_b, font=f, fill=(179, 38, 30))
    out.save(path)


def scenario_a(repo, base_env, cam, tmp):
    overlay = build_overlay(repo, [cam], tmp)
    env = dict(base_env, AMENT_PREFIX_PATH=f"{overlay}:{os.environ.get('AMENT_PREFIX_PATH', '')}")
    log = open(tmp / "a.log", "w")
    print("scenario A (moveit_system.launch.py), recording video ...")
    procs = [start(["ros2", "launch", "arm_bringup", "moveit_system.launch.py", "headless:=true"], env, log)]
    time.sleep(3)
    procs.append(start(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                        f"/{cam.topic}@sensor_msgs/msg/Image[gz.msgs.Image"], env, log))
    rclpy, node = make_node(cam)
    try:
        node.grip.wait_for_server(timeout_sec=90)
        node.mg.wait_for_server(timeout_sec=90)
        while not node.last or node.frame is None:
            time.sleep(0.1)
        time.sleep(6.0)          # let the arm settle under the controller
        node.caption = ("FR3 · Franka Hand gripper in Gazebo", "full MoveIt stack: moveit_system.launch.py", "")
        node.start_video(OUT / "gripper_issue.mp4")
        time.sleep(2.0)
        frames = OUT / "frames"
        frames.mkdir(exist_ok=True)
        events = run_steps(node, SCENARIO_A, True, frames)
        time.sleep(1.0)
        node.stop_video()
        plot(node.log, events, OUT / "gripper_issue_plot.png",
             "Scenario A (moveit_system.launch.py): finger positions; red steps expected motion but got none")
    finally:
        node.stop_video()
        rclpy.shutdown()
        stop(procs)
    side_by_side(OUT / "frames/step1.png", OUT / "frames/step5.png",
                 "Step 1: MoveIt 'open' before the direct 0.0 goal: opens",
                 "Step 5: the same MoveIt 'open' after it: no motion",
                 OUT / "gripper_issue_comparison.png")
    return events


def scenario_b(base_env, tmp, run):
    print(f"scenario B (sim.launch.py alone) run {run}, logging fingers ...")
    log = open(tmp / f"b{run}.log", "w")
    procs = [start(["ros2", "launch", "arm_sim", "sim.launch.py", "headless:=true"], base_env, log)]
    rclpy, node = make_node(None)
    try:
        node.grip.wait_for_server(timeout_sec=90)
        while not node.last:
            time.sleep(0.1)
        time.sleep(3.0)
        events = run_steps(node, SCENARIO_B, False)
        failed = any(e["expected_motion"] and not e["moved"] for e in events)
        outcome = "stuck" if failed else "ok"
        n = len(list(OUT.glob(f"gripper_issue_sim_only_{outcome}_*.png"))) + 1
        plot(node.log, events, OUT / f"gripper_issue_sim_only_{outcome}_{n}.png",
             f"Scenario B (sim.launch.py alone, arm not held), run {run}: "
             f"{'fingers stuck on some goals' if failed else 'all goals moved the fingers'}")
    finally:
        rclpy.shutdown()
        stop(procs)
    return events


def main():
    import json
    repo = Path(sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else ".").resolve()
    b_runs = 3
    for arg in sys.argv[1:]:
        if arg.startswith("--b-runs="):
            b_runs = int(arg.split("=", 1)[1])
    only_b = "--only-b" in sys.argv

    settings = read_settings(repo)
    spawn = np.array(settings["spawn_xyz"])
    pin, _, model = load_arm(repo / "src/arm_sim/urdf/fr3.urdf")
    data = model.createData()
    q = np.array([settings["home"][n] for n in list(model.names)[1:]])
    pin.framesForwardKinematics(model, data, q)
    tcp = data.oMf[model.getFrameId("fr3_hand_tcp")].translation + spawn
    target = tcp + np.array([0.0, 0.0, 0.04])
    cam = VideoCamera("hand", eye=target + np.array([0.42, -0.30, 0.10]), target=target, hfov=0.62, w=VW, h=VH)

    base_env = dict(os.environ, ROS_DOMAIN_ID=ROS_DOMAIN, GZ_PARTITION=GZ_PARTITION)
    for var in ("DISPLAY", "WAYLAND_DISPLAY"):
        base_env.pop(var, None)
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN

    path = OUT / "results.json"
    results = json.loads(path.read_text()) if path.exists() else {}
    with tempfile.TemporaryDirectory(prefix="fr3_grip_") as tmp:
        tmp = Path(tmp)
        if not only_b:
            results["scenario_a_moveit_system"] = scenario_a(repo, base_env, cam, tmp)
        runs = results.get("scenario_b_sim_only", []) if only_b else []
        for r in range(1, b_runs + 1):
            runs.append(scenario_b(base_env, tmp, r))
            path.write_text(json.dumps(dict(results, scenario_b_sim_only=runs), indent=2) + "\n")
        results["scenario_b_sim_only"] = runs
    path.write_text(json.dumps(results, indent=2) + "\n")
    print(f"done: outputs in {OUT}")


if __name__ == "__main__":
    main()
