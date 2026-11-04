"""独立 SDK 入口的连接检查；使用假传输，绝不连接实体 CAN。"""

import json
import unittest
from contextlib import ExitStack, redirect_stdout
from io import StringIO
from pathlib import Path
from unittest.mock import MagicMock, patch

from piper_draw.device import DeviceConfig
from piper_draw.piper_backend import STATE_FRAMES, Feedback, FeedbackClock
from piper_draw.robot import doctor


class HardwareConnectionTests(unittest.TestCase):
    def setUp(self):
        self.config = DeviceConfig("can0", "v2", "new", Path("unused-locks"))
        self.details = [{
            "flags": ["UP"],
            "linkinfo": {
                "info_kind": "can", "info_data": {"bittiming": {"bitrate": 1_000_000}},
            },
        }]
        frames = FeedbackClock()
        for frame in (*STATE_FRAMES, 0x2A8):
            frames.received(frame)
        self.transport = MagicMock()
        self.transport.__enter__.return_value = self.transport
        self.transport.frames = frames
        self.transport.read.return_value = Feedback(
            (0.0, 0.7, -1.0, 0.0, 0.3, 0.0), 10.0, (False,) * 6, 0, 0, "S-V1.7-8"
        )

    def invoke(self, *, configure_slave=False):
        with ExitStack() as stack:
            stack.enter_context(patch("piper_draw.robot.platform.system", return_value="Linux"))
            stack.enter_context(patch.object(Path, "exists", return_value=True))
            stack.enter_context(patch("piper_draw.robot.subprocess.check_output",
                                      return_value=json.dumps(self.details)))
            stack.enter_context(patch("piper_draw.robot.subprocess.run"))
            constructor = stack.enter_context(patch("piper_draw.robot.SDKTransport",
                                                   return_value=self.transport))
            self.constructor = constructor
            stack.enter_context(redirect_stdout(StringIO()))
            result = doctor(self.config, configure_slave=configure_slave)
            return result

    def test_feedback_is_saved_without_motion_or_enable(self):
        snapshot = self.invoke()
        self.assertEqual(snapshot["enabled"], (False,) * 6)
        self.assertEqual(snapshot["firmware"], "S-V1.7-8")
        self.assertEqual(snapshot["protocol"], "v2")
        self.assertIn("checked_at", snapshot)
        self.transport.__exit__.assert_called_once()
        self.transport.send.assert_not_called()
        self.assertEqual(self.transport.sdk.mock_calls, [])

    def test_inactive_can_is_rejected_before_sdk_opens(self):
        self.details[0]["flags"] = []
        with self.assertRaisesRegex(RuntimeError, "尚未启用"):
            self.invoke()
        self.constructor.assert_not_called()

    def test_wrong_bitrate_is_rejected(self):
        self.details[0]["linkinfo"]["info_data"]["bittiming"]["bitrate"] = 500_000
        with self.assertRaisesRegex(RuntimeError, "1 Mbit/s"):
            self.invoke()

    def test_failed_firmware_check_closes_transport(self):
        self.transport.read.return_value.firmware = "S-V1.8-8"
        with (
            patch("piper_draw.robot.time.monotonic", side_effect=[0.0, 0.0, 4.0]),
            patch("piper_draw.robot.time.sleep") as sleep,
            self.assertRaisesRegex(RuntimeError, "protocol=v3"),
        ):
            self.invoke()
        sleep.assert_not_called()
        self.transport.__exit__.assert_called_once()
        self.transport.send.assert_not_called()

    def test_feedback_configuration_only_changes_role_when_requested(self):
        self.invoke(configure_slave=True)
        self.assertEqual(self.transport.sdk.mock_calls,
                         [unittest.mock.call.MasterSlaveConfig(0xFC, 0, 0, 0)])
        self.transport.send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
