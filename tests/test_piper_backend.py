"""验证真机边界行为，不连接 CAN，也不发送实体指令。"""

import math
from types import SimpleNamespace

import pytest

from piper_draw.piper_backend import (
    STATE_FRAMES,
    Feedback,
    FeedbackClock,
    PiperSession,
    validate_firmware,
)


@pytest.fixture
def environment():
    now = [10.0]
    calls = []
    frames = FeedbackClock(lambda: now[0])
    for frame in (*STATE_FRAMES, 0x2A8):
        frames.received(frame)
    feedback = Feedback((0.0, 0.7, -1.0, 0.0, 0.3, 0.0), 10.0, (True,) * 6, 0, 0, "S-V1.7-8")
    transport = SimpleNamespace(
        frames=frames,
        read=lambda: feedback,
        send=lambda values: calls.append(("joints", values)),
        sdk=SimpleNamespace(
            MotionCtrl_1=lambda *values: calls.append(("stop", values)),
            EnableArm=lambda *values: calls.append(("enable", values)),
            GripperCtrl=lambda *values: calls.append(("gripper", values)),
        ),
    )
    session = PiperSession(
        transport,
        [-3.0, 0.0, -3.0, -2.0, -2.0, -3.0],
        [3.0, 3.14, 0.0, 2.0, 2.0, 3.0],
        clock=lambda: now[0],
    )
    return SimpleNamespace(session=session, now=now, calls=calls, feedback=feedback, frames=frames)


def test_default_is_read_only_even_if_motors_were_enabled(environment):
    e = environment
    e.session.command(e.feedback.positions)
    e.session.watchdog()
    assert e.calls == []
    with pytest.raises(RuntimeError, match="当前会话未使能"):
        e.session.preflight()


def test_enable_seeds_current_feedback_before_enabling(environment):
    e = environment
    e.session.command(e.feedback.positions)
    e.session.enable()
    assert e.calls == [("joints", e.feedback.positions), ("enable", (7,))]
    assert e.session.enabled


def test_enable_rejects_old_controller_target(environment):
    e = environment
    e.session.command([0.0] * 6)
    with pytest.raises(RuntimeError, match="未与当前反馈对齐"):
        e.session.enable()
    assert not e.calls


def test_failure_during_enable_still_requests_stop(environment):
    e = environment

    def fail_enable(*values):
        raise RuntimeError("使能通信失败")

    e.session.transport.sdk.EnableArm = fail_enable
    e.session.command(e.feedback.positions)
    with pytest.raises(RuntimeError, match="使能通信失败"):
        e.session.enable()
    assert e.calls[-1] == ("stop", (1, 0, 0))
    assert not e.session.enabled


def test_one_frozen_joint_pair_cannot_hide_behind_other_frames(environment):
    e = environment
    e.now[0] += 0.6
    for frame in (*STATE_FRAMES, 0x2A8):
        if frame != 0x2A6:
            e.frames.received(frame)
    with pytest.raises(RuntimeError, match="0x2a6"):
        e.session.feedback()


def test_command_timeout_fires_even_with_fresh_feedback(environment):
    e = environment
    e.session.command(e.feedback.positions)
    e.session.enable()
    e.now[0] += 0.3
    with pytest.raises(RuntimeError, match="指令断流"):
        e.session.watchdog()


@pytest.mark.parametrize("target", [[math.nan] * 6, [math.inf] * 6, [4.0] * 6, [0.0] * 5])
def test_invalid_targets_never_reach_sdk(environment, target):
    e = environment
    with pytest.raises(ValueError):
        e.session.command(target)
    assert e.calls == []


def test_tracking_error_does_not_send_new_target(environment):
    e = environment
    e.session.command(e.feedback.positions)
    e.session.enable()
    e.session.last_sent = (0.2, *e.feedback.positions[1:])
    e.calls.clear()
    with pytest.raises(RuntimeError, match="跟踪偏差"):
        e.session.command((0.2, *e.feedback.positions[1:]))
    assert not e.calls


def test_stop_latches_session_without_releasing_gripper(environment):
    e = environment
    e.session.stop("断流")
    e.session.command(e.feedback.positions)
    assert e.calls == [("stop", (1, 0, 0))]
    assert not e.session.enabled
    assert e.session.fault == "断流"


def test_hold_ignores_old_controller_targets_and_continues_without_stream(environment):
    e = environment
    e.session.command(e.feedback.positions)
    e.session.enable()
    e.calls.clear()
    e.session.hold("轨迹故障")
    e.session.command((0.1, *e.feedback.positions[1:]))
    e.now[0] += 0.3
    e.session.watchdog()
    assert e.calls == [("joints", e.feedback.positions)] * 2
    assert e.session.enabled and e.session.holding
    assert e.session.hold_reason == "轨迹故障"


def test_hold_cannot_hide_missing_feedback(environment):
    e = environment
    e.session.command(e.feedback.positions)
    e.session.enable()
    e.session.hold()
    e.now[0] += 0.6
    with pytest.raises(RuntimeError, match="超时"):
        e.session.watchdog()


def test_resume_requires_controller_alignment_before_unlocking_hold(environment):
    e = environment
    e.session.command(e.feedback.positions)
    e.session.enable()
    e.session.hold()
    e.session.command((0.1, *e.feedback.positions[1:]))
    with pytest.raises(RuntimeError, match="未与控制器目标对齐"):
        e.session.resume_commands()
    assert e.session.holding
    e.session.command(e.feedback.positions)
    e.session.resume_commands()
    assert not e.session.holding


def test_gripper_units_are_total_width_and_motor_torque(environment):
    e = environment
    e.session.gripper(12.5, 0.3)
    assert e.calls == [("gripper", (12500, 300, 1, 0))]
    assert not e.session.enabled


@pytest.mark.parametrize(
    "width,torque", [(-1, 0.3), (71, 0.3), (10, 0), (10, 1.1), (math.nan, 0.3)]
)
def test_invalid_gripper_request_has_no_side_effect(environment, width, torque):
    with pytest.raises(ValueError):
        environment.session.gripper(width, torque)
    assert not environment.calls


@pytest.mark.parametrize(
    "version,protocol,dh",
    [
        ("S-V1.6-2", "v2", "old"),
        ("S-V1.6-3", "v2", "new"),
        ("S-V1.8-7", "v2", "new"),
        ("S-V1.8-8", "v3", "new"),
    ],
)
def test_firmware_boundary_matches_official_model_and_protocol(version, protocol, dh):
    validate_firmware(version, protocol, dh)


@pytest.mark.parametrize(
    "version,protocol,dh",
    [
        ("-1199", "v2", "new"),
        ("S-V1.6-2", "v2", "new"),
        ("S-V1.8-8", "v2", "new"),
        ("S-V1.8-7", "v3", "new"),
    ],
)
def test_unknown_or_mismatched_firmware_is_rejected(version, protocol, dh):
    with pytest.raises(RuntimeError):
        validate_firmware(version, protocol, dh)
