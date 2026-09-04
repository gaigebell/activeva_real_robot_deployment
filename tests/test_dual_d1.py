import argparse
import unittest

import numpy as np

from robot.dual_d1_test import build_parser, run_hold, run_jog, valid_feedback


class FakeClient:
    def __init__(self, state):
        self.state = list(state)
        self.targets = []

    def set_target(self, target):
        self.targets.append(np.asarray(target))


class FakeRig:
    def __init__(self, arms=("left", "right")):
        self.arms = tuple(arms)
        self.clients = {
            arm: FakeClient([0, 0, 0, 0, 0, 0, 20]) for arm in self.arms
        }

    def wait_feedback(self, _timeout):
        return {arm: client.state for arm, client in self.clients.items()}


class DualD1TestCliTests(unittest.TestCase):
    def test_feedback_validation(self):
        self.assertTrue(valid_feedback([0] * 7))
        self.assertFalse(valid_feedback([0] * 6))
        self.assertFalse(valid_feedback([0, 0, 0, 0, 0, 0, float("nan")]))

    def test_jog_requires_confirmation(self):
        args = argparse.Namespace(
            yes=False, delta=1.0, joint=0, arm="left",
            feedback_timeout=0.0, hold_seconds=0.0)
        with self.assertRaisesRegex(RuntimeError, "--yes"):
            run_jog(FakeRig(), args)

    def test_jog_only_targets_selected_arm(self):
        rig = FakeRig()
        args = argparse.Namespace(
            yes=True, delta=1.0, joint=0, arm="left",
            feedback_timeout=0.0, hold_seconds=0.0)
        run_jog(rig, args)
        self.assertEqual(len(rig.clients["left"].targets), 1)
        self.assertEqual(len(rig.clients["right"].targets), 0)
        self.assertEqual(rig.clients["left"].targets[0][0], 1.0)

    def test_single_arm_cli_selection(self):
        args = build_parser().parse_args(["status", "--arm", "right"])
        self.assertEqual(args.arm, "right")

    def test_hold_operates_with_only_one_connected_arm(self):
        rig = FakeRig(("left",))
        args = argparse.Namespace(
            yes=True, arm="left", feedback_timeout=0.0, hold_seconds=0.0)
        run_hold(rig, args)
        self.assertEqual(len(rig.clients["left"].targets), 1)

    def test_jog_rejects_large_step(self):
        args = argparse.Namespace(
            yes=True, delta=5.1, joint=0, arm="left",
            feedback_timeout=0.0, hold_seconds=0.0)
        with self.assertRaisesRegex(RuntimeError, "5"):
            run_jog(FakeRig(), args)

    def test_hold_echoes_feedback_to_both_arms(self):
        rig = FakeRig()
        args = argparse.Namespace(
            yes=True, arm="both", feedback_timeout=0.0, hold_seconds=0.0)
        run_hold(rig, args)
        for arm in ("left", "right"):
            np.testing.assert_allclose(
                rig.clients[arm].targets[0], rig.clients[arm].state)


if __name__ == "__main__":
    unittest.main()
