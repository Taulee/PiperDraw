"""恢复命令只在明确执行时发一次，且绝不自动使能或运动。"""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from piper_draw.piper_backend import Feedback
from piper_draw.recovery import resume


class ResumeTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.state = Feedback((0,) * 6, 0, (True,) * 6, 1, 0, "S-V1.8-9")
        self.sdk = Mock()
        self.sdk.ResetPiper.side_effect = self.recover
        self.transport = SimpleNamespace(frames=Mock(), read=lambda: self.state, sdk=self.sdk)

    def recover(self):
        self.state.arm_status = 0
        self.state.enabled = (False,) * 6

    def sleep(self, seconds):
        self.now += seconds

    def run_resume(self, **kwargs):
        return resume(self.transport, clock=lambda: self.now, sleep=self.sleep, **kwargs)

    def test_explicit_restore_is_once_without_enable_or_motion(self):
        report = self.run_resume()
        self.assertEqual(report["status"], "resumed")
        self.assertEqual(report["before"]["arm_status"], 1)
        self.assertEqual(report["after"]["arm_status"], 0)
        self.sdk.ResetPiper.assert_called_once_with()
        self.assertEqual(len(self.sdk.mock_calls), 1)

    def test_already_normal_never_resets(self):
        self.state.arm_status = 0
        self.assertEqual(self.run_resume()["status"], "already_normal")
        self.assertEqual(self.sdk.mock_calls, [])

    def test_other_faults_are_not_cleared(self):
        self.state.error_code = 1
        with self.assertRaises(RuntimeError):
            self.run_resume()
        self.assertEqual(self.sdk.mock_calls, [])

    def test_no_automatic_retry_after_restore(self):
        self.sdk.ResetPiper.side_effect = None
        with self.assertRaisesRegex(RuntimeError, "不自动重发"):
            self.run_resume()
        self.sdk.ResetPiper.assert_called_once_with()

    def test_missing_feedback_prevents_restore(self):
        self.transport.frames.require.side_effect = RuntimeError("反馈超时")
        with self.assertRaises(RuntimeError):
            self.run_resume()
        self.assertEqual(self.sdk.mock_calls, [])


if __name__ == "__main__":
    unittest.main()
