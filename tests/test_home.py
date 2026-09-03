import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml

from robot.home import capture_home, load_home, move_single_arm_to_home


class FakeHomeClient:
    def __init__(self, arm="right", state=None):
        self.arm = arm
        self.state = np.asarray(state or [0, 0, 0, 0, 0, 0, 20], dtype=float)
        self.commands = []
        self.powered = False
        self.enabled = False

    def get_state(self):
        return self.state.tolist()

    def set_target(self, target, mode=0):
        self.state = np.asarray(target, dtype=float)
        self.commands.append((self.state.copy(), mode))

    def power_on(self):
        self.powered = True

    def enable(self):
        self.enabled = True


class HomeTests(unittest.TestCase):
    def test_capture_preserves_other_arm_and_loads_seven_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "home.yaml"
            path.write_text(yaml.safe_dump({"left": [1] * 7}), encoding="utf-8")
            client = FakeHomeClient(state=[2, 3, 4, 5, 6, 7, 8])
            capture_home(client, path, timeout_s=0.1)
            np.testing.assert_allclose(load_home(path, "right"), [2, 3, 4, 5, 6, 7, 8])
            np.testing.assert_allclose(load_home(path, "left"), [1] * 7)

    @patch("robot.home.time.sleep", return_value=None)
    def test_move_home_holds_then_reaches_target(self, _sleep):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "home.yaml"
            path.write_text(
                yaml.safe_dump({"right": [5, -5, 3, 0, 0, 0, 25]}),
                encoding="utf-8")
            client = FakeHomeClient()
            move_single_arm_to_home(client, path, {
                "feedback_timeout_s": 0.1,
                "max_initial_delta_deg": 10,
                "joint_speed_deg_s": 100,
                "gripper_speed_unit_s": 100,
                "command_hz": 10,
                "settle_timeout_s": 0.1,
                "settle_tolerance_deg": 0.1,
            })
            self.assertTrue(client.powered)
            self.assertTrue(client.enabled)
            self.assertEqual(client.commands[0][1], 1)
            np.testing.assert_allclose(client.state, [5, -5, 3, 0, 0, 0, 25])

    def test_move_home_rejects_distant_start_before_powering(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "home.yaml"
            path.write_text(yaml.safe_dump({"right": [40, 0, 0, 0, 0, 0, 20]}),
                            encoding="utf-8")
            client = FakeHomeClient()
            with self.assertRaisesRegex(RuntimeError, "离 Home 太远"):
                move_single_arm_to_home(client, path, {
                    "feedback_timeout_s": 0.1,
                    "max_initial_delta_deg": 35,
                })
            self.assertFalse(client.powered)
            self.assertFalse(client.commands)


if __name__ == "__main__":
    unittest.main()
