"""显式解除软件急停；只发送一次 ResetPiper，不自动使能。"""

import time

from .piper_backend import STATE_FRAMES, validate_firmware


def resume(transport, *, clock=time.monotonic, sleep=time.sleep, gripper_present=True):
    required = (*STATE_FRAMES, 0x2A8) if gripper_present else STATE_FRAMES

    def feedback():
        transport.frames.require(required)
        return transport.read()

    deadline = clock() + 3
    while True:
        try:
            transport.frames.require(required)
            break
        except RuntimeError:
            if clock() >= deadline:
                raise
            sleep(0.05)
    before = feedback()
    validate_firmware(before.firmware, "v3", "new")
    result = {"before": dict(vars(before)), "resume_sent": False}
    if before.error_code or before.arm_status not in (0, 1):
        raise RuntimeError("控制器不是单纯软件急停状态，请先检查故障")
    if before.arm_status == 0:
        return {**result, "status": "already_normal"}
    # ResetPiper 可能失能并导致下落；恢复不串联重新使能。
    transport.sdk.ResetPiper()
    result["resume_sent"] = True
    sleep(0.6)
    deadline = clock() + 3
    while True:
        after = feedback()
        if after.arm_status == 0 and after.error_code == 0:
            return {**result, "status": "resumed", "after": dict(vars(after))}
        if clock() >= deadline:
            raise RuntimeError("已发送一次恢复指令，仍未确认急停解除；不自动重发")
        sleep(0.05)
