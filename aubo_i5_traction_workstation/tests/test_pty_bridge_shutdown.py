"""Offline PTY lifecycle regression: local TCP only, no serial or robot."""

import os
import select
import socket
import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.skipif(os.name != "posix", reason="Linux PTY bridge")
def test_supervisor_sigterm_removes_owned_pty_link(tmp_path):
    """A restart must not inherit a dead bridge's still-existing PTY link."""
    script = Path(__file__).resolve().parents[1] / "scripts/kwr75_wsl_pty_bridge.py"
    link = tmp_path / "sensor-tty"
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(3)
        child = subprocess.Popen(
            [sys.executable, str(script), "--port", str(listener.getsockname()[1]),
             "--link", str(link)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        try:
            connection, _ = listener.accept()
            with connection:
                readable, _, _ = select.select([child.stdout], [], [], 3)
                assert readable, "bridge did not publish PTY readiness"
                assert child.stdout.readline().startswith(b"PTY_READY=")
                assert link.is_symlink()
                child.terminate()
                child.communicate(timeout=3)
                assert child.returncode == 0
                assert not link.is_symlink()
                assert not link.exists()
        finally:
            if child.poll() is None:
                child.kill()
                child.communicate(timeout=3)
