"""Export-content tests using explicitly synthetic fixtures, never live ROS."""

import json
import zipfile
from unittest.mock import patch

import pytest

import app
from backend.runtime_recorder import RuntimeRecorder


@pytest.mark.parametrize('mode', [0, 1, 2])
def test_session_zip_contains_fault_and_stop_feedback_without_page_clicks(tmp_path, mode):
    sessions = tmp_path / 'sessions'
    record = sessions / 'session_100000' / 'traction.csv'
    record.parent.mkdir(parents=True)
    record.write_text('state,fault_code\nFAULT,TEST_ONLY_FAULT\n', encoding='utf-8')
    summary = {'session_id': 'session_100000', 'operation_mode': mode,
               'start_time': {'sec': 100}, 'end_time': {'sec': 102},
               'record_path': str(record), 'stop_reason': 'TEST_ONLY_FAULT'}
    recorder = RuntimeRecorder(tmp_path / 'feedback')
    fixture = {'connected': True, 'robot': {
        'identity_verified': True, 'valid': True, 'data_age_ms': 10,
        'joint_position_rad': [0.0] * 6, 'joint_velocity_rad_s': [0.0] * 6,
        'diagnostic': {'values': {'stepwise_phase': 'STOPPING', 'motion_idle': '0'}}},
        'traction': {'operation_mode': mode, 'state': 9, 'fault_code': 'TEST_ONLY_FAULT'}}
    recorder.append(fixture, timestamp=101.0)
    fixture['robot']['diagnostic']['values'] = {'stepwise_phase': 'DISABLED', 'motion_idle': '1'}
    recorder.append(fixture, timestamp=103.0)
    with (patch.object(app, 'ROOT', tmp_path), patch.object(app, 'SESSION_ROOT', sessions),
          patch.object(app, 'runtime_recorder', recorder),
          patch.object(app, '_read_operations', return_value=[]),
          patch.object(app, 'history', return_value={'summaries': [summary]})):
        response = app.export_session('session_100000')
    with zipfile.ZipFile(response.path) as archive:
        rows = [json.loads(line) for line in archive.read('runtime_feedback.jsonl').splitlines()]
        assert len(rows) == 2
        assert rows[0]['snapshot']['traction']['fault_code'] == 'TEST_ONLY_FAULT'
        assert rows[1]['snapshot']['robot']['diagnostic']['values']['motion_idle'] == '1'
        assert rows[0]['snapshot']['robot']['identity_verified'] is True
        assert json.loads(archive.read('summary.json'))['operation_mode'] == mode
        assert json.loads(archive.read('runtime_evidence.json'))['sample_count'] == 2
        assert len(archive.read('operations.csv').decode('utf-8-sig').splitlines()) == 1


def test_old_session_missing_feedback_is_explicit_not_reconstructed(tmp_path):
    sessions = tmp_path / 'sessions'
    record = sessions / 'session_1000' / 'traction.csv'
    record.parent.mkdir(parents=True)
    record.write_text('state\nREADY\n', encoding='utf-8')
    summary = {'session_id': 'session_1000', 'start_time': {'sec': 1},
               'end_time': {'sec': 2}, 'record_path': str(record)}
    with (patch.object(app, 'ROOT', tmp_path), patch.object(app, 'SESSION_ROOT', sessions),
          patch.object(app, 'runtime_recorder', RuntimeRecorder(tmp_path / 'feedback')),
          patch.object(app, '_read_operations', return_value=[]),
          patch.object(app, 'history', return_value={'summaries': [summary]})):
        response = app.export_session('session_1000')
    with zipfile.ZipFile(response.path) as archive:
        assert archive.read('runtime_feedback.jsonl') == b''
        evidence = json.loads(archive.read('runtime_evidence.json'))
        assert evidence['sample_count'] == 0
        assert evidence['first_capture_unix_s'] is None
