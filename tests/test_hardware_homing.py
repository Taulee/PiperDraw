"""以假反馈验证回零边界，不连接实体 CAN。"""

import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from piper_draw.homing import move_to_joint_target
from piper_draw.piper_backend import Feedback


class HomingTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.feedback = Feedback((1.0, 0, 0, 0, 0.2, 0), 0, (False,) * 6, 0, 0, "S-V1.8-9")
        self.sdk = Mock()
        self.sdk.EnableArm.side_effect = self.enable
        self.transport = SimpleNamespace(frames=Mock(), read=lambda: self.feedback, sdk=self.sdk)
        self.move = True
        self.velocity = 0.06

    def enable(self, motor):
        self.feedback.enabled = (True,) * 6

    def sleep(self, seconds):
        self.now += seconds
        if self.move and all(self.feedback.enabled):
            self.feedback.positions = tuple(max(0, p - seconds * self.velocity) for p in self.feedback.positions)

    def run_home(self, **kwargs):
        return move_to_joint_target(self.transport, clock=lambda: self.now, sleep=self.sleep, **kwargs)

    def test_zero_is_reached_with_only_low_speed_zero_targets(self):
        report = self.run_home()
        self.assertEqual(report["status"], "reached")
        self.assertLessEqual(max(map(abs, report["after"]["positions"])), math.radians(0.5))
        for call in self.sdk.JointCtrl.call_args_list:
            self.assertEqual(call.args, (0,) * 6)
        for call in self.sdk.MotionCtrl_2.call_args_list:
            self.assertEqual(call.args, (1, 1, 3, 0))
        self.sdk.ResetPiper.assert_not_called()
        self.sdk.DisableArm.assert_not_called()
        self.sdk.GripperCtrl.assert_not_called()
        self.sdk.MotionCtrl_1.assert_not_called()

    def test_no_motion_on_initial_controller_error(self):
        self.feedback.error_code = 1
        with self.assertRaisesRegex(RuntimeError, "控制器异常"):
            self.run_home()
        self.assertEqual(self.sdk.mock_calls, [])

    def test_emergency_stop_is_never_cleared_automatically(self):
        self.feedback.arm_status = 1
        with self.assertRaisesRegex(RuntimeError, "控制器异常"):
            self.run_home()
        self.assertEqual(self.sdk.mock_calls, [])

    def test_stalled_motion_requests_stop(self):
        self.move = False
        with self.assertRaisesRegex(RuntimeError, "向目标收敛"):
            self.run_home()
        self.sdk.MotionCtrl_1.assert_called_once_with(1, 0, 0)
        self.sdk.ResetPiper.assert_not_called()

    def test_feedback_loss_requests_stop(self):
        self.transport.frames.require.side_effect = lambda *a: (
            (_ for _ in ()).throw(RuntimeError("反馈超时")) if self.now > 0.1 else None
        )
        with self.assertRaisesRegex(RuntimeError, "反馈超时"):
            self.run_home()
        self.sdk.MotionCtrl_1.assert_called_once_with(1, 0, 0)

    def test_speed_above_slow_range_is_rejected_without_sdk_calls(self):
        with self.assertRaises(ValueError):
            self.run_home(speed_percent=100)
        self.assertEqual(self.sdk.mock_calls, [])

    def test_final_angle_jump_is_recorded_without_stopping(self):
        self.feedback.positions = (0, 0, 0, math.radians(0.8), 0, 0)
        self.feedback.enabled = (True,) * 6
        self.velocity = math.radians(20)
        report = self.run_home(speed_percent=1)
        self.assertEqual(report["status"], "reached")
        self.assertGreater(report["peak_measured_speed_deg_s"], 12)
        self.sdk.MotionCtrl_1.assert_not_called()

    def test_parking_target_is_sent_and_used_for_completion(self):
        self.move = False
        degrees = (25, 40, -30, 0, 5, -10)
        self.feedback.positions = tuple(map(math.radians, degrees))
        report = self.run_home(target_rad=self.feedback.positions, speed_percent=1)
        self.assertEqual(report["status"], "reached")
        for call in self.sdk.JointCtrl.call_args_list:
            self.assertEqual(call.args, tuple(angle * 1000 for angle in degrees))
        self.sdk.MotionCtrl_1.assert_not_called()

    def test_interruption_requests_stop_without_reset_or_disable(self):
        def interrupted_sleep(seconds):
            raise KeyboardInterrupt

        with self.assertRaises(KeyboardInterrupt):
            move_to_joint_target(self.transport, clock=lambda: self.now, sleep=interrupted_sleep)
        self.sdk.MotionCtrl_1.assert_called_once_with(1, 0, 0)
        self.sdk.ResetPiper.assert_not_called()
        self.sdk.DisableArm.assert_not_called()


if __name__ == "__main__":
    unittest.main()
