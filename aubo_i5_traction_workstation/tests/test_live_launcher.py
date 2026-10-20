"""Offline launch contracts: these tests never source ROS or open hardware."""

import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'run_live_workstation.sh'


class LiveLauncherTest(unittest.TestCase):
    def test_mirrored_network_datagrams_stay_below_mtu(self):
        config = ROOT / 'config' / 'cyclonedds_local.xml'
        namespace = {'c': 'https://cdds.io/config'}
        general = ET.parse(config).find('c:Domain/c:General', namespace)
        maximum = int(general.find('c:MaxMessageSize', namespace).text.rstrip('B'))
        fragment = int(general.find('c:FragmentSize', namespace).text.rstrip('B'))
        self.assertLessEqual(maximum + 28, 1500)
        self.assertLess(fragment, maximum)
        self.assertIn('export CYCLONEDDS_URI=', SCRIPT.read_text(encoding='utf-8'))
        # Evaluate just the export, never the hardware launcher, with an invalid
        # ambient profile: full startup must still select the validated file.
        assignment = next(line for line in SCRIPT.read_text(encoding='utf-8').splitlines()
                          if line.startswith('export CYCLONEDDS_URI='))
        environment = dict(os.environ, CYCLONEDDS_URI='file:///invalid/ambient.xml',
                           PROJECT_DIR=str(ROOT))
        effective = subprocess.check_output(
            ['bash', '-c', assignment + '\nprintf "%s" "$CYCLONEDDS_URI"'],
            env=environment, text=True,
        )
        self.assertEqual(effective, 'file://' + str(config))

    def test_shell_syntax(self):
        subprocess.run(['bash', '-n', str(SCRIPT)], check=True, capture_output=True)

    def test_missing_credentials_exits_before_startup(self):
        environment = os.environ.copy()
        environment.pop('AUBO_V4_USERNAME', None)
        environment.pop('AUBO_V4_PASSWORD', None)
        with tempfile.TemporaryDirectory(prefix='aubo_launcher_offline_') as temporary:
            profile = Path(temporary) / 'profile.yaml'
            profile.write_text('{}\n', encoding='utf-8')
            log_directory = Path(temporary) / 'logs'
            result = subprocess.run(
                ['bash', str(SCRIPT), str(profile), str(log_directory)],
                env=environment, capture_output=True, timeout=5, check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(log_directory.exists())

    def test_startup_has_no_automatic_traction_request(self):
        script = SCRIPT.read_text(encoding='utf-8')
        self.assertNotIn('/api/traction/start', script)
        self.assertNotIn('/api/traction/prepare', script)
        self.assertIn('/api/traction/stop', script)
        self.assertEqual(script.count('launch robot '), 1)
        self.assertIn('flock -n 9', script)
        self.assertIn('wait -n', script)

    def test_full_stack_uses_the_current_session_baseline(self):
        script = SCRIPT.read_text(encoding='utf-8')
        self.assertIn('--params-file "$PROFILE" -p session_managed_tare:=true', script)
        config = (ROOT / 'ros2_overlay/src/fr_traction/config/traction_params.yaml').read_text()
        self.assertIn('wrench_topic: /traction/corrected_wrench', config)

    def test_direct_entry_and_restart_check_before_stopping(self):
        entry = ROOT / 'start_workstation.sh'
        subprocess.run(['bash', '-n', str(entry)], check=True, capture_output=True)
        self.assertIn('run_workstation.sh" --restart-live', entry.read_text())
        launcher = (ROOT / 'run_workstation.sh').read_text()
        self.assertLess(launcher.index('"${arguments[@]}" -CheckLocalConfig'),
                        launcher.index('bash "$PROJECT_DIR/scripts/stop_live_workstation.sh"'))

    def test_windows_background_is_explicit_and_hidden(self):
        script = (ROOT / 'scripts' / 'start_live_workstation.ps1').read_text(encoding='utf-8')
        self.assertIn('[switch]$Background', script)
        self.assertEqual(script.count('Start-Process $shellExecutable'), 2)
        self.assertEqual(script.count('-WindowStyle Hidden'), 2)
        self.assertNotIn('-ExecutionPolicy', script)


if __name__ == '__main__':
    unittest.main()
