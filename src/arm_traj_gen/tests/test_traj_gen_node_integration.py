"""Integration test: fake /joint_states -> traj_gen_node -> /desired_joint_states.

Runs on an isolated ROS domain so it cannot touch a running simulation.
"""
import os
import signal
import subprocess
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '77'

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

ORDER = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -1.571, 'elbow_joint': 0.0,
        'wrist_1_joint': -1.571, 'wrist_2_joint': 0.0, 'wrist_3_joint': 0.0}
NODE = os.environ['TRAJ_GEN_NODE']


def run_node(names, positions, seconds):
    """Publish a fixed /joint_states while traj_gen_node runs; return (messages, log)."""
    rclpy.init()
    node = Node('traj_gen_test')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    got = []
    node.create_subscription(JointState, '/desired_joint_states',
                             lambda m: got.append((time.monotonic(), m)), 10)
    proc = subprocess.Popen([NODE], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    try:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            m = JointState()
            m.name = names
            m.position = positions
            m.velocity = [0.0] * len(positions)
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.01)
        alive = proc.poll() is None
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            log, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            log, _ = proc.communicate()
        node.destroy_node()
        rclpy.shutdown()
    return got, log, alive


def pose(**overrides):
    values = dict(HOME, **overrides)
    return ORDER, [values[n] for n in ORDER]


class TestTrajGenNode(unittest.TestCase):
    def test_valid_home_publishes_pan_trajectory(self):
        names, pos = pose()
        got, log, alive = run_node(names, pos, 8.5)
        self.assertTrue(alive)
        self.assertIn('Loaded limits for 6 joints', log)
        self.assertGreater(len(got), 100)
        t0 = got[0][0]
        pan = ORDER.index('shoulder_pan_joint')
        first, last = got[0][1], got[-1][1]
        self.assertEqual(list(first.name), ORDER)
        for n, p in zip(first.name, first.position):
            self.assertAlmostEqual(p, HOME[n], delta=1e-3)
        for i, n in enumerate(last.name):
            self.assertAlmostEqual(last.position[i], HOME[n] + (0.1 if i == pan else 0.0),
                                   delta=1e-6)
            self.assertAlmostEqual(last.velocity[i], 0.0, delta=1e-6)
        mid = min(got, key=lambda g: abs((g[0] - t0) - 2.5))[1]
        self.assertAlmostEqual(mid.position[pan], 0.05, delta=2e-3)
        self.assertAlmostEqual(mid.velocity[pan], 0.0375, delta=1e-3)

    def _assert_rejected(self, names, pos, expected_log):
        got, log, alive = run_node(names, pos, 3.0)
        self.assertTrue(alive, 'node must survive invalid input')
        self.assertEqual(len(got), 0, 'no trajectory may be published')
        self.assertIn(expected_log, log)

    def test_start_beyond_limit_is_rejected(self):
        self._assert_rejected(*pose(shoulder_pan_joint=6.4), 'outside limits')

    def test_target_beyond_limit_is_rejected(self):
        self._assert_rejected(*pose(shoulder_pan_joint=6.2), 'shoulder_pan_joint target')

    def test_elbow_tighter_limit_is_rejected(self):
        self._assert_rejected(*pose(elbow_joint=3.2), 'elbow_joint start')

    def test_unknown_joint_is_rejected(self):
        names = ORDER[:5] + ['bogus_joint']
        self._assert_rejected(names, [HOME[n] for n in ORDER[:5]] + [0.0], 'bogus_joint')

    def test_non_finite_is_rejected(self):
        names, pos = pose()
        pos[0] = float('nan')
        self._assert_rejected(names, pos, 'Non-finite')


if __name__ == '__main__':
    unittest.main()
