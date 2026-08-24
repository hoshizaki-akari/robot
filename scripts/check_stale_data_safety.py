#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path
from time import monotonic


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from platform_b.force_control import ConstantForceController  # noqa: E402
from platform_b.gateway import UpstreamCache  # noqa: E402
from state_service.app import MAX_SNAPSHOT_AGE_MS, StateStore  # noqa: E402


def main() -> int:
    store = StateStore("replay")
    snapshot = store.source.sample()
    old_time = monotonic() - (MAX_SNAPSHOT_AGE_MS + 500) / 1000.0
    for value in snapshot.values():
        if isinstance(value, dict) and "_updated_monotonic" in value:
            value["_updated_monotonic"] = old_time
    store._snapshot = snapshot
    stale = store.get()
    assert not stale["system"]["valid"]
    assert all(not stale[name]["valid"] for name in ("fr5", "kwr75d", "ag95", "d435"))

    controller = ConstantForceController()
    assert controller._extract_wrench(stale) is None

    gateway = UpstreamCache()
    gateway._snapshot = stale
    gateway._received_at = monotonic()
    forwarded = gateway.get()
    assert not forwarded["gateway"]["valid"]
    assert not forwarded["system"]["valid"]
    print("通过：共同服务会把超过1.5秒的旧数据标记为无效。")
    print("通过：B平台和力控不会接受仍能访问但已经过期的数据。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
