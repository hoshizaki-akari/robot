"""Run the real page script against a small DOM, without a robot or web server."""

import json
import os
import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = os.environ.get("NODE_BINARY") or shutil.which("node")

HARNESS = r"""
const vm = require('node:vm');
const assert = require('node:assert/strict');
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    value: '', disabled: false, textContent: '', innerHTML: '',
    classList: { values: new Set(), add(v) { this.values.add(v); },
      remove(v) { this.values.delete(v); },
      toggle(v, enabled) { if (enabled) this.values.add(v); else this.values.delete(v); } },
    style: {setProperty() {}}, addEventListener() {},
    getBoundingClientRect: () => ({width: 500, height: 200}),
    getContext: () => new Proxy({}, {get: () => () => {}}),
  });
  return elements.get(id);
}
const modes = [2, 1, 3].map(mode => ({...element(`mode${mode}`), dataset: {mode: String(mode)}}));
const calls = [];
const arrows = [];
const sandbox = {
  console, requestLog: calls, arrowLog: arrows, document: {getElementById: element, activeElement: null,
    querySelectorAll: selector => selector === '.mode-btn' ? modes : []},
  window: {addEventListener() {}, updateTractionDirection: (...args) => arrows.push(args)},
  location: {protocol: 'http:', host: 'localhost'},
  sessionStorage: {getItem: () => null, setItem() {}, removeItem() {}},
  WebSocket: class {},
  setInterval() {return 1;}, setTimeout() {return 1;}, clearTimeout() {}, clearInterval() {},
  fetch: async (url, options) => {
    calls.push({url, options});
    return {ok: true, json: async () => ({summaries: [], traction_force_limit_n: 100})};
  },
};
vm.createContext(sandbox);
vm.runInContext(SOURCE, sandbox);
(async () => {
  await vm.runInContext(TEST, sandbox);
})().catch(error => {console.error(error); process.exitCode = 1;});
"""


@unittest.skipUnless(NODE, "node is required for browser state tests; set NODE_BINARY")
class UiBehaviorTest(unittest.TestCase):
    def test_opens_in_local_mode_and_can_operate_without_login(self):
        html = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        self.assertNotIn('id="loginModal"', html)
        self.assertNotIn('id="logoutBtn"', html)
        self.run_page(r"""
          (async () => {
            if (!sessionUser || sessionUser.role !== '本地操作')
              throw Error('Page still requires a login session');
            handleState({connected: true, traction: {
              state: 1, operation_mode: 1, calibrated: true,
              direction_available: true, direction_locked: false,
              actual_force_n: 6, target_force_n: 10
            }});
            if ($('startBtn').disabled || $('settingsBtn').disabled)
              throw Error('Local-mode controls are still blocked by login');
            await startTraction();
            const request = requestLog.find(call => call.url === '/api/traction/start');
            if (!request || decodeURIComponent(request.options.headers['X-Operator']) !== '本机操作员')
              throw Error('Local operator was not recorded');
          })()
        """)

    def run_page(self, assertions):
        source = (ROOT / "static" / "090105.js").read_text(encoding="utf-8")
        script = (
            "const SOURCE = " + json.dumps(source) + ";\n"
            "const TEST = " + json.dumps(assertions) + ";\n" + HARNESS
        )
        result = subprocess.run(
            [NODE, "-"], input=script, text=True, capture_output=True, timeout=20, check=False
        )
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)

    def test_start_without_direction_uses_manager_search_without_recalibrating(self):
        self.run_page(r"""
          (async () => {
            sessionUser = {name: '医生', role: '医生'};
            operationMode = 1; tractionState = 2; dataOnline = true; calibrated = true;
            directionAvailable = false;
            applyPermissions();
            if ($('startBtn').disabled) throw Error('No-direction start was blocked');
            await startTraction();
            if (!pendingStart) throw Error('Must await real control state');
            const commands = requestLog.filter(call => call.options?.method === 'POST');
            if (commands.map(call => call.url).join(',') !== '/api/traction/target,/api/traction/start')
              throw Error('Start recalibrated or invoked a direction confirmation');
          })()
        """)

    def test_search_hold_and_unload_button_permissions(self):
        self.run_page(r"""
          sessionUser = {name: '医生', role: '医生'};
          dataOnline = true; calibrated = true; operationMode = 1;
          tractionState = 3; actualForce = 8; directionAvailable = true;
          applyPermissions();
          if (!$('prepareBtn').disabled || !$('unloadBtn').disabled || $('stopBtn').disabled)
            throw Error('Search must allow stopping and block calibration/unload');
          tractionState = 5; directionLocked = true;
          applyPermissions();
          if (!$('prepareBtn').disabled || $('stopBtn').disabled || !$('startBtn').disabled)
            throw Error('Activation handoff must remain a locked active run');
          tractionState = 12; directionLocked = true;
          applyPermissions();
          if (!$('prepareBtn').disabled || $('targetForceVal').disabled || $('unloadBtn').disabled)
            throw Error('Hold must allow target adjustment/unload and block calibration');
          actualForce = 4.99; applyPermissions();
          if (!$('unloadBtn').disabled) throw Error('Below-five-newton unload accepted');
          actualForce = 5; directionAvailable = false; applyPermissions();
          if (!$('unloadBtn').disabled) throw Error('Missing-direction unload accepted');
          directionAvailable = true; tractionState = 7; applyPermissions();
          if (!$('unloadBtn').disabled || $('stopBtn').disabled)
            throw Error('Active unload must allow stop and block duplicate unload');
        """)

    def test_end_keeps_calibration_and_allows_the_next_run_immediately(self):
        self.run_page(r"""
          (async () => {
            sessionUser = {name: '医生', role: '医生'};
            handleState({connected: true, traction: {
              state: 6, operation_mode: 1, calibrated: true,
              direction_available: true, direction_locked: true,
              actual_force_n: 20, target_force_n: 20, locked_direction_base: [1, 0, 0]
            }});
            await finishTraction();
            handleState({connected: true, traction: {
              state: 1, operation_mode: 1, calibrated: true,
              direction_available: true, direction_locked: false,
              actual_force_n: 20, target_force_n: 20, locked_direction_base: [1, 0, 0]
            }});
            if (activeRecord || !calibrated || directionLocked)
              throw Error('Normal end lost calibration or kept the run active');
            if ($('startBtn').disabled || $('targetForceVal').disabled || $('prepareBtn').disabled)
              throw Error('Ready controls require another calibration after end');
            if (!$('stopBtn').classList.values.has('slot-hidden'))
              throw Error('End did not restore the start action');
            await startTraction();
            const commands = requestLog.filter(call => call.options?.method === 'POST');
            if (commands.map(call => call.url).join(',') !==
                '/api/traction/stop,/api/traction/target,/api/traction/start')
              throw Error('The second run unexpectedly recalibrated');
          })()
        """)

    def test_active_traction_can_unload_and_return_to_ready_without_calibration(self):
        self.run_page(r"""
          (async () => {
            sessionUser = {name: '医生', role: '医生'};
            handleState({connected: true, traction: {
              state: 6, operation_mode: 1, calibrated: true,
              direction_available: true, direction_locked: true,
              actual_force_n: 50, target_force_n: 50, locked_direction_base: [1, 0, 0]
            }});
            await unloadTraction();
            const commands = requestLog.filter(call => call.options?.method === 'POST');
            if (commands.map(call => call.url).join(',') !== '/api/traction/unload')
              throw Error('One-click unload was not sent directly to manager');
            handleState({connected: true, traction: {
              state: 7, operation_mode: 3, calibrated: true,
              direction_available: true, direction_locked: true,
              actual_force_n: 20, target_force_n: 50
            }});
            handleState({connected: true, traction: {
              state: 1, operation_mode: 3, calibrated: true,
              direction_available: true, direction_locked: false,
              actual_force_n: 4, target_force_n: 50
            }});
            if (activeRecord || !calibrated || directionLocked)
              throw Error('Unload completion lost calibration or kept the run active');
          })()
        """)

    def test_force_range_can_exceed_100_and_active_arrow_remains_locked(self):
        self.run_page(r"""
          sessionUser = {name: '医生', role: '医生'};
          applyForceLimit(160);
          if (tractionForceLimit !== 160 || Number($('targetForceVal').max) !== 160)
            throw Error('100 N target cap still active');
          handleState({connected: true, traction: {
            state: 6, operation_mode: 1, calibrated: true,
            direction_available: true, direction_locked: true,
            actual_force_n: 155, target_force_n: 140,
            locked_direction_base: [1, 0, 0], force_direction_base: [0, 1, 0]
          }});
          if ($('directionState').textContent !== '方向已锁定') throw Error('Not locked');
          if (tractionState !== 6 || actualForce !== 155) throw Error('Local force fault introduced');
          if (JSON.stringify(arrowLog.at(-1)) !== '[[1,0,0]]')
            throw Error('Live force rotated the locked direction arrow');
        """)


if __name__ == "__main__":
    unittest.main()
