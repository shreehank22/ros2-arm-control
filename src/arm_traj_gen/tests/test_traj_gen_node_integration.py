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

# /joint_states order as a simulator might send it: shuffled, with the finger joints included
ORDER = ['fr3_joint4', 'fr3_joint2', 'fr3_joint1', 'fr3_finger_joint1', 'fr3_joint5',
         'fr3_joint6', 'fr3_joint7', 'fr3_joint3', 'fr3_finger_joint2']
# traj_gen_node publishes only the arm joints, in model order
ARM_JOINTS = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
              'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
HOME = {'fr3_joint1': 0.0, 'fr3_joint2': -0.7854, 'fr3_joint3': 0.0, 'fr3_joint4': -2.3562,
        'fr3_joint5': 0.0, 'fr3_joint6': 1.5708, 'fr3_joint7': 0.7854,
        'fr3_finger_joint1': 0.0, 'fr3_finger_joint2': 0.0}
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
    def test_valid_home_publishes_joint1_trajectory(self):
        names, pos = pose()
        got, log, alive = run_node(names, pos, 8.5)
        self.assertTrue(alive)
        self.assertIn('Loaded limits for 7 joints', log)
        self.assertGreater(len(got), 100)
        t0 = got[0][0]
        joint1 = ARM_JOINTS.index('fr3_joint1')
        first, last = got[0][1], got[-1][1]
        self.assertEqual(list(first.name), ARM_JOINTS)
        for n, p in zip(first.name, first.position):
            self.assertAlmostEqual(p, HOME[n], delta=1e-3)
        for i, n in enumerate(last.name):
            self.assertAlmostEqual(last.position[i], HOME[n] + (0.1 if i == joint1 else 0.0),
                                   delta=1e-6)
            self.assertAlmostEqual(last.velocity[i], 0.0, delta=1e-6)
        mid = min(got, key=lambda g: abs((g[0] - t0) - 2.5))[1]
        self.assertAlmostEqual(mid.position[joint1], 0.05, delta=2e-3)
        self.assertAlmostEqual(mid.velocity[joint1], 0.0375, delta=1e-3)

    def _assert_rejected(self, names, pos, expected_log):
        got, log, alive = run_node(names, pos, 3.0)
        self.assertTrue(alive, 'node must survive invalid input')
        self.assertEqual(len(got), 0, 'no trajectory may be published')
        self.assertIn(expected_log, log)

    def test_start_beyond_limit_is_rejected(self):
        self._assert_rejected(*pose(fr3_joint1=3.0), 'outside limits')

    def test_target_beyond_limit_is_rejected(self):
        # 2.85 is valid, but the +0.1 target exceeds the 2.9007 limit
        self._assert_rejected(*pose(fr3_joint1=2.85), 'fr3_joint1 target')

    def test_joint4_asymmetric_limit_is_rejected(self):
        # joint4 range is [-3.077, -0.1169], so 0.0 is out of range
        self._assert_rejected(*pose(fr3_joint4=0.0), 'fr3_joint4 start')

    def test_unknown_joint_is_rejected(self):
        # an arm joint replaced by an unknown name: the arm joint is then missing
        names, pos = pose()
        names = ['bogus_joint' if n == 'fr3_joint7' else n for n in names]
        self._assert_rejected(names, pos, 'fr3_joint7 not found')

    def test_non_finite_is_rejected(self):
        names, pos = pose()
        pos[0] = float('nan')
        self._assert_rejected(names, pos, 'Non-finite')


if __name__ == '__main__':
    unittest.main()
