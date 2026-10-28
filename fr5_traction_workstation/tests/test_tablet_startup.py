"""Exercise the real supervisor with fake services/browser; no ROS or robot IO."""

import os
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

STUB = r"""
import os
import signal
import sys
import time
from pathlib import Path

signal.signal(signal.SIGINT, signal.SIG_DFL)
root = Path(os.environ["STARTUP_FIXTURE"])
kind = sys.argv[1]
if kind == "timeout":
    sys.exit(0 if (root / "robot_ready").exists() else 1)
if kind == "curl":
    sys.exit(0 if (root / "web_started").exists() else 1)
if kind == "snap":
    sys.exit(1)
if kind in ("notify-send", "zenity"):
    sys.exit(0)
(root / f"{kind}_pid").write_text(str(os.getpid()))
if kind == "web":
    (root / "web_started").touch()
elif kind == "firefox":
    assert (root / "web_started").exists(), "Browser opened before web server"
    (root / "browser_started").touch()
elif kind == "ros":
    count = root / "ros_attempts"
    count.write_text(str(int(count.read_text()) + 1) if count.exists() else "1")
    if not (root / "sdk_ready").exists():
        print("Traceback (most recent call last): cold controller", flush=True)
        sys.exit(1)
    print("FR5 direct driver connected", flush=True)
while True:
    if kind == "firefox" and (root / "close_browser").exists():
        break
    if kind == "ros" and (root / "drop_ros").exists():
        break
    time.sleep(0.03)
"""


@unittest.skipUnless(
    Path("/opt/ros/humble/setup.bash").exists() and shutil.which("setsid"),
    "Linux deployment tools are required",
)
class TabletStartupTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="fr5-web-first-")
        self.root = Path(self.temp.name)
        project = self.root / "workspace" / "workstation"
        scripts = project / "scripts"
        scripts.mkdir(parents=True)
        self.launcher = scripts / "tablet_start.sh"
        shutil.copyfile(ROOT / "scripts/tablet_start.sh", self.launcher)
        for relative in (
            "ros2_overlay/install/setup.bash",
            "vendor/fairino-python-sdk/linux/fairino/Robot.py",
            "../.venv/bin/activate",
        ):
            path = project / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        stub = self.root / "stub.py"
        stub.write_text(STUB)
        bin_dir = self.root / "bin"
        bin_dir.mkdir()
        for name, kind in (
            ("firefox", "firefox"), ("curl", "curl"), ("timeout", "timeout"),
            ("snap", "snap"), ("notify-send", "notify-send"), ("zenity", "zenity"),
        ):
            path = bin_dir / name
            path.write_text(
                "#!/bin/bash\nexec " + shlex.quote(sys.executable) + " "
                + shlex.quote(str(stub)) + " " + kind + ' "$@"\n'
            )
            path.chmod(0o755)
        for path, kind in (
            (project / "run_workstation.sh", "web"),
            (scripts / "start_ros_stack.sh", "ros"),
        ):
            path.write_text(
                "#!/bin/bash\nexec " + shlex.quote(sys.executable) + " "
                + shlex.quote(str(stub)) + " " + kind + "\n"
            )
        home = self.root / "home"
        runtime = self.root / "runtime"
        home.mkdir()
        runtime.mkdir()
        self.environment = dict(
            os.environ, HOME=str(home), XDG_RUNTIME_DIR=str(runtime),
            STARTUP_FIXTURE=str(self.root), PATH=str(bin_dir) + ":" + os.environ["PATH"],
        )
        self.process = None
        self.output = (self.root / "supervisor.log").open("w")

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=9)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        # A failing assertion must still leave no fake component running.
        for path in self.root.glob("*_pid"):
            try:
                os.kill(int(path.read_text()), signal.SIGTERM)
            except ProcessLookupError:
                pass
        self.output.close()
        self.temp.cleanup()

    def start(self):
        self.process = subprocess.Popen(
            ["bash", str(self.launcher)], env=self.environment,
            stdout=self.output, stderr=subprocess.STDOUT, start_new_session=True,
        )
        self.wait_for(lambda: (self.root / "browser_started").exists())

    def wait_for(self, condition, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            if self.process.poll() is not None:
                self.fail((self.root / "supervisor.log").read_text())
            time.sleep(0.04)
        self.fail("Supervisor condition timed out:\n" + self.startup_log())

    def startup_log(self):
        return "\n".join(
            path.read_text() for path in
            (self.root / "home/.local/state/fr5-traction/logs").glob("startup_*.log")
        )

    def attempts(self):
        path = self.root / "ros_attempts"
        return int(path.read_text()) if path.exists() else 0

    def test_web_and_browser_open_before_robot_and_connect_when_ready(self):
        self.start()
        self.assertEqual(0, self.attempts())
        (self.root / "robot_ready").touch()
        (self.root / "sdk_ready").touch()
        self.wait_for(lambda: "机械臂驱动已连接" in self.startup_log())
        self.assertEqual(1, self.attempts())
        self.assertIsNone(self.process.poll())
        (self.root / "close_browser").touch()
        self.assertEqual(0, self.process.wait(timeout=8))

    def test_controller_initialization_failure_retries_without_closing_ui(self):
        (self.root / "robot_ready").touch()
        self.start()
        self.wait_for(lambda: self.attempts() == 1)
        self.wait_for(lambda: "清理本次启动后重试" in self.startup_log())
        self.assertIsNone(self.process.poll())
        (self.root / "sdk_ready").touch()
        self.wait_for(lambda: "机械臂驱动已连接" in self.startup_log())
        self.assertEqual(2, self.attempts())
        self.assertIsNone(self.process.poll())

    def test_closing_browser_while_waiting_cancels_late_robot_connection(self):
        self.start()
        (self.root / "close_browser").touch()
        self.assertEqual(0, self.process.wait(timeout=8))
        (self.root / "robot_ready").touch()
        (self.root / "sdk_ready").touch()
        time.sleep(0.2)
        self.assertEqual(0, self.attempts())

    def test_supervisor_shutdown_cancels_waiting_and_cleans_components(self):
        self.start()
        self.process.terminate()
        self.assertEqual(0, self.process.wait(timeout=8))
        (self.root / "robot_ready").touch()
        (self.root / "sdk_ready").touch()
        time.sleep(0.2)
        self.assertEqual(0, self.attempts())
        for kind in ("firefox", "web"):
            pid = int((self.root / f"{kind}_pid").read_text())
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)

    def test_runtime_exit_keeps_ui_without_restarting_robot(self):
        (self.root / "robot_ready").touch()
        (self.root / "sdk_ready").touch()
        self.start()
        self.wait_for(lambda: "机械臂驱动已连接" in self.startup_log())
        (self.root / "drop_ros").touch()
        self.wait_for(lambda: "运行中的控制栈已退出" in self.startup_log())
        time.sleep(3.6)
        self.assertIsNone(self.process.poll())
        self.assertEqual(1, self.attempts())
