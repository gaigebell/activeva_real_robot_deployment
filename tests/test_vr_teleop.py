import socket
import time
import unittest

import numpy as np

from robot.control import D1Teleop, ForwardReceiver
from robot.controller_mapping import ControllerMapping, matrix_to_quat, quat_to_matrix
from robot.d1_ik import D1IK, TCP_OFFSET_FROM_LINK6


class FakeClient:
    def __init__(self):
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0]
        self.commands = []

    def get_state(self):
        return self.state

    def set_target(self, target, mode=0):
        self.commands.append((np.asarray(target), mode))


class FakeIK:
    def __init__(self):
        self.last_orientation = None
        self.last_position = None

    def forward_kinematics(self, _q):
        return np.eye(4)

    def home_full(self, q):
        return [0.0] + list(q) + [0.0]

    def solve(self, _position, _orientation, _initial):
        self.last_position = np.asarray(_position)
        self.last_orientation = _orientation
        return np.deg2rad(np.full(6, 10.0))

    def solve_continuous_step(self, _position, _orientation, _current, **_kwargs):
        self.last_position = np.asarray(_position)
        self.last_orientation = _orientation
        return np.deg2rad(np.full(6, 1.0))


class FakeReceiver:
    def __init__(self, message):
        self.message = message

    def get_latest(self, hand, _max_age=None):
        return self.message if hand == "left" else None


def teleop_config():
    return {
        "deadman_threshold": 0.5,
        "controller_timeout_s": 0.25,
        "position_scale": 0.2,
        "max_position_delta_m": 0.05,
        "max_joint_step_deg": 1.0,
        "max_joint_delta_from_engage_deg": 30.0,
        "max_gripper_step": 2.0,
        "control_orientation": True,
        "orientation_scale": 1.0,
        "max_orientation_delta_deg": 45.0,
        "position_axes": [0, 1, 2],
        "position_signs": [1, 1, 1],
        "gripper_open": 65.0,
        "gripper_closed": 0.0,
    }


class VrTeleopTests(unittest.TestCase):
    def test_installed_mirror_mapping_matches_world_directions(self):
        left = ControllerMapping(
            position_axes=[0, 2, 1], position_signs=[1, -1, 1])
        right = ControllerMapping(
            position_axes=[0, 2, 1], position_signs=[-1, -1, 1])
        for mapping in (left, right):
            mapping.reset([0, 0, 0], [0, 0, 0, 1],
                          [0, 0, 0], [0, 0, 0, 1])

        # WebXR +X=向右、+Y=向上、-Z=向前。右臂符号以现场单轴
        # 平移结果为准：向前对应右臂局部 +Y。
        np.testing.assert_allclose(left.map([.01, 0, 0], [0, 0, 0, 1])[0],
                                   [.01, 0, 0])
        np.testing.assert_allclose(right.map([.01, 0, 0], [0, 0, 0, 1])[0],
                                   [-.01, 0, 0])
        np.testing.assert_allclose(left.map([0, 0, -.01], [0, 0, 0, 1])[0],
                                   [0, .01, 0])
        np.testing.assert_allclose(right.map([0, 0, -.01], [0, 0, 0, 1])[0],
                                   [0, .01, 0])
        np.testing.assert_allclose(left.map([0, .01, 0], [0, 0, 0, 1])[0],
                                   [0, 0, .01])
        np.testing.assert_allclose(right.map([0, .01, 0], [0, 0, 0, 1])[0],
                                   [0, 0, .01])

    def test_position_sign_fix_does_not_change_right_orientation_basis(self):
        right = ControllerMapping(
            position_axes=[0, 2, 1], position_signs=[-1, -1, 1],
            orientation_axes=[0, 2, 1], orientation_signs=[-1, 1, 1])
        right.reset([0, 0, 0], [0, 0, 0, 1],
                    [0, 0, 0], [0, 0, 0, 1])
        angle = np.deg2rad(10.0)
        controller_q = [0, np.sin(angle / 2), 0, np.cos(angle / 2)]
        _, target_q = right.map([0, 0, 0], controller_q)
        self.assertAlmostEqual(np.linalg.det(right.basis), 1.0)
        self.assertFalse(np.allclose(target_q, [0, 0, 0, 1]))

    def test_forward_receiver_reads_controller_json(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        receiver = ForwardReceiver("127.0.0.1", port)
        receiver.start()
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1) as sock:
                sock.sendall(
                    b'{"type":"controller_pose","hand":"left","p":{"x":0,"y":0,"z":0},'
                    b'"q":{"x":0,"y":0,"z":0,"w":1},"squeeze":1,"trigger":0}\n')
            deadline = time.monotonic() + 1.0
            while receiver.get_latest("left") is None and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertIsNotNone(receiver.get_latest("left", max_age_s=1.0))
        finally:
            receiver.stop()

    def test_no_command_without_deadman(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 0.0, "trigger": 0.0,
        }
        clients = {arm: FakeClient() for arm in ("left", "right")}
        teleop = D1Teleop(
            clients, {arm: FakeIK() for arm in clients},
            FakeReceiver(message), teleop_config())
        teleop.step()
        self.assertFalse(clients["left"].commands)
        self.assertFalse(clients["right"].commands)
        self.assertIsNone(teleop.ik["left"].last_orientation)

    def test_single_arm_teleop_only_reads_matching_controller(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
        }
        clients = {"right": FakeClient()}

        class RightReceiver:
            def __init__(self):
                self.requested = []

            def get_latest(self, hand, _max_age=None):
                self.requested.append(hand)
                return message

        receiver = RightReceiver()
        teleop = D1Teleop(clients, {"right": FakeIK()}, receiver, teleop_config())
        teleop.step()
        self.assertEqual(receiver.requested, ["right"])
        self.assertEqual(len(clients["right"].commands), 1)

    def test_home_alignment_runs_without_squeeze_and_b_pauses(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 0.0, "trigger": 0.0,
            "buttons": [False, False, False, False, True, False],
        }
        cfg = teleop_config()
        cfg["home_align_hold_s"] = 0.0
        client = FakeClient()
        teleop = D1Teleop(
            {"left": client}, {"left": FakeIK()}, FakeReceiver(message),
            cfg, "position", "home")

        teleop.step()  # 建立 A 按下时间
        self.assertFalse(client.commands)
        teleop.step()  # 长按达到阈值，完成对齐并开始
        self.assertEqual(len(client.commands), 1)
        message["buttons"] = [False, False, False, False, False, False]
        message["p"]["x"] = 0.02
        teleop.step()
        self.assertEqual(len(client.commands), 2)

        message["buttons"] = [False, False, False, False, False, True]
        teleop.step()
        self.assertEqual(len(client.commands), 2)
        self.assertFalse(teleop.held["left"])

    def test_home_brief_tracking_gap_stops_then_auto_realigns(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 0.0, "trigger": 0.0,
            "buttons": [False, False, False, False, True, False],
            "_received_monotonic": time.monotonic(),
        }
        cfg = teleop_config()
        cfg.update({"home_align_hold_s": 0.0, "home_rearm_timeout_s": 3.0})
        client = FakeClient()
        teleop = D1Teleop(
            {"left": client}, {"left": FakeIK()}, FakeReceiver(message),
            cfg, "position", "home")
        teleop.step()
        teleop.step()
        self.assertEqual(len(client.commands), 1)

        message["_received_monotonic"] = time.monotonic() - 1.0
        teleop.step()
        self.assertEqual(len(client.commands), 1)
        self.assertFalse(teleop.held["left"])

        message["buttons"] = [False] * 6
        message["_received_monotonic"] = time.monotonic()
        teleop.step()
        self.assertEqual(len(client.commands), 2)
        self.assertTrue(teleop.held["left"])

    def test_continuous_ik_integrates_from_last_command_not_stale_feedback(self):
        class IncrementalIK(FakeIK):
            def __init__(self):
                super().__init__()
                self.references = []

            def solve_continuous_step(self, _position, _orientation, current, **_kwargs):
                self.references.append(np.asarray(current).copy())
                return np.asarray(current) + np.deg2rad(np.full(6, 0.5))

        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
        }
        client, ik = FakeClient(), IncrementalIK()
        teleop = D1Teleop(
            {"left": client}, {"left": ik}, FakeReceiver(message),
            teleop_config(), "position")
        teleop.step()
        teleop.step()
        np.testing.assert_allclose(np.rad2deg(ik.references[0]), 0.0)
        np.testing.assert_allclose(np.rad2deg(ik.references[1]), 0.5)
        np.testing.assert_allclose(client.commands[-1][0][:6], 1.0)

    def test_position_mode_locks_engagement_orientation(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
        }
        receiver = FakeReceiver(message)
        client, ik = FakeClient(), FakeIK()
        teleop = D1Teleop(
            {"left": client}, {"left": ik}, receiver, teleop_config(), "position")
        teleop.step()
        message["p"]["x"] = 0.05
        message["q"] = {"x": 0, "y": 0, "z": 1, "w": 0}
        teleop.step()
        np.testing.assert_allclose(ik.last_orientation, [0, 0, 0, 1])
        np.testing.assert_allclose(ik.last_position, [0.01, 0, 0])
        self.assertEqual(client.commands[-1][0][6], 20.0)

    def test_orientation_mode_locks_engagement_position(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
        }
        receiver = FakeReceiver(message)
        client, ik = FakeClient(), FakeIK()
        teleop = D1Teleop(
            {"left": client}, {"left": ik}, receiver, teleop_config(), "orientation")
        teleop.step()
        message["p"]["x"] = 0.05
        message["q"] = {"x": 0, "y": 0, "z": 1, "w": 0}
        teleop.step()
        np.testing.assert_allclose(ik.last_position, [0, 0, 0])
        self.assertFalse(np.allclose(ik.last_orientation, [0, 0, 0, 1]))

    def test_calibration_rejects_discontinuous_ik_solution(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
        }
        client, ik = FakeClient(), FakeIK()
        ik.solve_continuous_step = lambda *_args, **_kwargs: np.deg2rad(
            [0, 0, 0, 0, 0, 6])
        teleop = D1Teleop(
            {"left": client}, {"left": ik}, FakeReceiver(message),
            teleop_config(), "position")
        teleop.step()
        self.assertFalse(client.commands)

    def test_timeout_requires_release_before_reengagement(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
            "_received_monotonic": time.monotonic(),
        }
        receiver = FakeReceiver(message)
        client = FakeClient()
        teleop = D1Teleop(
            {"left": client}, {"left": FakeIK()}, receiver, teleop_config())
        teleop.step()
        self.assertEqual(len(client.commands), 1)

        message["_received_monotonic"] = time.monotonic() - 1.0
        teleop.step()
        message["_received_monotonic"] = time.monotonic()
        teleop.step()
        self.assertEqual(len(client.commands), 1)

        message["squeeze"] = 0.0
        teleop.step()
        message["squeeze"] = 1.0
        message["_received_monotonic"] = time.monotonic()
        teleop.step()
        self.assertEqual(len(client.commands), 2)

    def test_brief_stale_gap_stops_then_realigns_without_release(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
            "_received_monotonic": time.monotonic(),
        }
        cfg = teleop_config()
        cfg["controller_rearm_timeout_s"] = 3.0
        receiver, client = FakeReceiver(message), FakeClient()
        teleop = D1Teleop(
            {"left": client}, {"left": FakeIK()}, receiver, cfg)
        teleop.step()

        message["_received_monotonic"] = time.monotonic() - 1.0
        teleop.step()
        self.assertEqual(len(client.commands), 1)
        self.assertFalse(teleop.timeout_latched["left"])

        message["_received_monotonic"] = time.monotonic()
        teleop.step()
        self.assertEqual(len(client.commands), 2)

    def test_orientation_basis_transform_and_angle_limit(self):
        mapping = ControllerMapping(
            position_axes=[2, 0, 1], position_signs=[-1, 1, 1],
            orientation_scale=1.0, max_orientation_delta_deg=30.0)
        mapping.reset([0, 0, 0], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0, 1])
        angle = np.deg2rad(90.0)
        controller_q = [np.sin(angle / 2), 0, 0, np.cos(angle / 2)]
        _, target_q = mapping.map([0, 0, 0], controller_q)
        target_angle = np.rad2deg(
            np.arccos(np.clip((np.trace(quat_to_matrix(target_q)) - 1) / 2, -1, 1)))
        self.assertAlmostEqual(target_angle, 30.0, places=5)

    def test_total_joint_delta_is_bounded_from_engagement(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
        }
        clients = {arm: FakeClient() for arm in ("left", "right")}
        cfg = teleop_config()
        cfg["teleop_max_joint_step_deg"] = 100.0
        cfg["max_joint_step_deg"] = 100.0
        teleop = D1Teleop(
            clients, {arm: FakeIK() for arm in clients},
            FakeReceiver(message), cfg)
        teleop.ik["left"].solve_continuous_step = (
            lambda *_args, **_kwargs: np.deg2rad(np.full(6, 40.0)))
        teleop.step()
        clients["left"].state[:6] = [30.0] * 6
        teleop.step()
        np.testing.assert_allclose(clients["left"].commands[-1][0][:6], 30.0)

    def test_deadman_limits_joint_and_gripper_steps(self):
        message = {
            "p": {"x": 0, "y": 0, "z": 0},
            "q": {"x": 0, "y": 0, "z": 0, "w": 1},
            "squeeze": 1.0, "trigger": 0.0,
        }
        clients = {arm: FakeClient() for arm in ("left", "right")}
        teleop = D1Teleop(
            clients, {arm: FakeIK() for arm in clients},
            FakeReceiver(message), teleop_config())
        teleop.step()
        target, mode = clients["left"].commands[0]
        np.testing.assert_allclose(target[:6], 1.0)
        self.assertEqual(target[6], 22.0)
        self.assertEqual(mode, 0)
        self.assertFalse(clients["right"].commands)
        self.assertIsNotNone(teleop.ik["left"].last_orientation)

    def test_real_urdf_ik_round_trip_at_zero(self):
        ik = D1IK()
        frame = ik.forward_kinematics([0.0] * 6)
        from robot.controller_mapping import matrix_to_quat
        solved = ik.solve(
            frame[:3, 3], matrix_to_quat(frame[:3, :3]), q_init=ik.home_full())
        np.testing.assert_allclose(solved, 0.0, atol=1e-4)

    def test_d1_chain_uses_symmetric_fixed_tcp_not_one_finger(self):
        ik = D1IK()
        self.assertEqual(ik.chain.links[-1].name, "gripper_center_tcp")
        np.testing.assert_allclose(
            ik.chain.links[-1].origin_translation, TCP_OFFSET_FROM_LINK6)
        self.assertFalse(ik.chain.active_links_mask[-1])

    def test_continuous_ik_does_not_jump_at_singular_pose(self):
        ik = D1IK()
        q = np.zeros(6)
        frame = ik.forward_kinematics(q)
        target_position = frame[:3, 3].copy()
        target_position[1] += 0.005
        solved = ik.solve_continuous_step(
            target_position, matrix_to_quat(frame[:3, :3]), q)
        self.assertLess(np.max(np.abs(np.rad2deg(solved - q))), 5.0)

    def test_condition_number_rejects_logged_singular_home(self):
        ik = D1IK()
        singular_home = np.deg2rad([19.2, -13.9, 16.4, 3.6, 2.2, -1.2])
        usable_home = np.deg2rad([20.0, 0.0, 42.6, 2.3, -27.1, -3.1])
        self.assertGreater(ik.condition_number(singular_home), 80.0)
        self.assertLess(ik.condition_number(usable_home), 80.0)


if __name__ == "__main__":
    unittest.main()
