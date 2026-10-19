#!/usr/bin/env python3
"""Record one explicitly requested, bounded manual-drag trial through the real web API."""

import argparse
import json
import math
import time
import urllib.request
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-prepared-drag', action='store_true', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=30.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or not 5 <= args.duration_s <= 30:
        parser.error('--duration-s must be between 5 and 30 seconds')
    base = 'http://127.0.0.1:8082'

    def request(path, post=False):
        call = urllib.request.Request(base + path, data=b'{}' if post else None,
                                      headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(call, timeout=3) as response:
            return json.load(response)

    before = request('/api/state')
    if (not before['connected'] or before['traction']['state'] != 2 or
            before['traction']['operation_mode'] != 2 or
            before['traction']['actual_force_n'] > 5):
        raise RuntimeError('Requires prepared, connected drag mode and gentle manual force')
    origin = before['traction']['ee_position_base_m']
    if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise RuntimeError('Invalid real starting position')
    result = {'status': 'NOT_ACCEPTED', 'max_measured_displacement_m': 0.0,
              'peak_force_n': 0.0, 'loaded_motion_observed': False,
              'release_observed': False}
    samples = []
    started = time.monotonic()
    attempted = False
    try:
        attempted = True
        result['start_response'] = request('/api/traction/start', True)['message']
        while time.monotonic() - started < args.duration_s:
            current = request('/api/state')
            traction = current['traction']
            force = float(traction['actual_force_n'])
            position = traction['ee_position_base_m']
            driver = current['robot']['diagnostic']['values']
            samples.append({'elapsed_s': time.monotonic() - started, 'force_n': force,
                            'position_m': position, 'state': traction['state'],
                            'fault_code': traction.get('fault_code', ''),
                            'driver_phase': driver['stepwise_phase'],
                            'driver_motion_idle': driver['motion_idle']})
            if (not current['connected'] or not traction['valid'] or
                    traction['state'] in (9, 10) or not math.isfinite(force) or
                    len(position) != 3 or not all(math.isfinite(v) for v in position)):
                raise RuntimeError('Real hardware feedback invalid or faulted')
            distance = math.dist(position, origin)
            result['max_measured_displacement_m'] = max(
                result['max_measured_displacement_m'], distance)
            result['peak_force_n'] = max(result['peak_force_n'], force)
            if distance > 0.015 or force > 5:
                raise RuntimeError('Trial displacement/force limit reached')
            if force > 1 and distance > 0.0005:
                result['loaded_motion_observed'] = True
            if result['loaded_motion_observed'] and force < 0.3 and driver['motion_idle'] == '1':
                result['release_observed'] = True
            time.sleep(0.05)
    except (Exception, KeyboardInterrupt) as error:  # noqa: BLE001 - always stop a started trial
        result['error'] = str(error)
    finally:
        if attempted:
            try:
                result['stop_response'] = request('/api/traction/stop', True)['message']
            except Exception as error:  # noqa: BLE001 - retain failure and still inspect hardware
                result['stop_error'] = str(error)
        deadline = time.monotonic() + 5
        stable = 0
        while time.monotonic() < deadline and stable < 20:
            try:
                final = request('/api/state')
                stopped = (final['connected'] and final['robot']['identity_verified'] and
                           final['robot']['diagnostic']['values']['motion_idle'] == '1')
                result['final_manager_state'] = final['traction']['state']
                result['final_fault_code'] = final['traction'].get('fault_code', '')
                stable = stable + 1 if stopped else 0
            except Exception:  # noqa: BLE001 - polling failure must never count as stopped
                stable = 0
            time.sleep(0.1)
        result['measured_stop_confirmed'] = stable >= 20
        if ('error' not in result and 'stop_error' not in result and
                result['loaded_motion_observed'] and result['release_observed'] and
                stable >= 20 and result.get('final_manager_state') == 1):
            result['status'] = 'LOADED_DRAG_AND_RELEASE_OBSERVED'
        result['samples'] = samples
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({key: value for key, value in result.items() if key != 'samples'}, indent=2))
    return 0 if result['status'] == 'LOADED_DRAG_AND_RELEASE_OBSERVED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
