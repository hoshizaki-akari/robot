"""Bounded recovery of software communication faults, without hardware resets."""

import logging
import math
import time

from .ros_bridge import RosBridgeError

LOGGER = logging.getLogger(__name__)
RECOVERABLE_FAULTS = frozenset({"WRENCH_TIMEOUT", "EE_STATE_TIMEOUT", "UI_HEARTBEAT_TIMEOUT"})


class TransientFaultRecovery:
    def __init__(self):
        self.code = None
        self.stable_since = None
        self.attempts = 0
        self.retry_at = 0.0
        self.reset_sent = False

    def poll(self, bridge, now=None):
        now = time.monotonic() if now is None else now
        snapshot = bridge.snapshot()
        traction = snapshot.get("traction", {})
        code = traction.get("fault_code")
        if traction.get("state") != 9 or code not in RECOVERABLE_FAULTS:
            self.__init__()
            return
        if code != self.code:
            self.__init__()
            self.code = code
        speeds = snapshot.get("fr5", {}).get("joint_velocity_rad_s", [])
        stationary = len(speeds) == 6 and all(
            math.isfinite(float(value)) for value in speeds
        ) and sum(float(value) ** 2 for value in speeds) <= 0.02 ** 2
        if not snapshot.get("connected") or not stationary:
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
        if (now - self.stable_since < 1.0 or now < self.retry_at
                or self.attempts >= 3 or self.reset_sent):
            return
        self.attempts += 1
        self.retry_at = now + 2.0
        try:
            # This service reads cached driver status only. It cannot clear
            # controller errors, change enable/mode, calibrate or start motion.
            bridge.call("auto_recovery_check")
            current = bridge.snapshot().get("traction", {})
            if current.get("state") != 9 or current.get("fault_code") != code:
                return
            bridge.call("reset_transient_fault")
            self.reset_sent = True
            LOGGER.info("Transient software fault %s recovered; waiting for operator Start.", code)
        except RosBridgeError as error:
            LOGGER.warning("Automatic software recovery deferred (%d/3): %s", self.attempts, error)
