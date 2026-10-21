import argparse
import contextlib
import io
import json
from pathlib import Path
import tempfile
import os
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import types
import unittest
from unittest.mock import patch

import storage_guard as guard


class StorageProtectionTests(unittest.TestCase):
    def setUp(self):
        scratch = os.environ.get('RMUL_STORAGE_TEST_SCRATCH')
        self.wsl = os.name != 'nt' and 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower()
        if scratch is None and self.wsl:
            scratch = next((Path('/mnt', drive) for drive in ('e', 'd', 'c') if Path('/mnt', drive).is_mount()), None)
            if scratch is None:
                self.skipTest('Storage guard tests on WSL require a mounted Windows volume')
        self.temp = tempfile.TemporaryDirectory(dir=scratch or guard.HERE)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.policy = json.loads((guard.HERE / 'storage-policy.json').read_text())

    def args(self):
        path = self.root / 'policy.json'
        path.write_text(json.dumps(self.policy))
        return argparse.Namespace(policy=path, artifact_root=self.root / 'artifacts',
                                  state_dir=self.root / 'state', no_notify=True,
                                  host_root=[str(self.root)], check_only=False,
                                  command=['dummy-child'], profile='light')

    def execute(self, free_values, process=None, advance=None):
        values = iter(free_values)
        last = [free_values[-1]]

        def disk_usage(path):
            last[0] = next(values, last[0])
            return types.SimpleNamespace(free=last[0])

        with patch.object(guard.shutil, 'disk_usage', side_effect=disk_usage), \
                patch.object(guard.subprocess, 'Popen', return_value=process) as popen, \
                patch.object(guard, 'stop_owned') as stop, \
                patch.object(guard.time, 'sleep', side_effect=advance), \
                contextlib.redirect_stdout(io.StringIO()):
            result = guard.run(self.args())
        events = [json.loads(row) for row in (self.root / 'state/events.jsonl').read_text().splitlines()]
        return result, events, popen, stop

    def test_wsl_rejects_vhd_artifact_directory(self):
        if not self.wsl:
            self.skipTest('WSL-specific VHD protection')
        args = self.args()
        args.artifact_root = Path('/tmp/codex-storage-must-not-create')
        with patch.object(guard.subprocess, 'Popen') as popen:
            with self.assertRaisesRegex(ValueError, 'outside the VHD'):
                guard.run(args)
        popen.assert_not_called()

    def test_start_and_stop_thresholds(self):
        self.assertFalse(guard.check_reasons({'D': 50 * guard.GIB}, self.policy, True))
        self.assertTrue(guard.check_reasons({'D': 50 * guard.GIB - 1}, self.policy, True))
        self.assertFalse(guard.check_reasons({'D': 30 * guard.GIB}, self.policy))
        self.assertTrue(guard.check_reasons({'D': 30 * guard.GIB - 1}, self.policy))

    def test_start_failure_does_not_launch_child(self):
        result, events, popen, _ = self.execute([18 * guard.MIB])
        self.assertEqual(result, 73)
        popen.assert_not_called()
        self.assertEqual(events[-1]['kind'], 'blocked')

    def test_low_output_volume_also_blocks(self):
        result, _, popen, _ = self.execute([100 * guard.GIB, 10 * guard.GIB])
        self.assertEqual(result, 73)
        popen.assert_not_called()

    def test_failed_space_read_does_not_launch_child(self):
        with patch.object(guard.shutil, 'disk_usage', side_effect=OSError('unreadable')), \
                patch.object(guard.subprocess, 'Popen') as popen, \
                contextlib.redirect_stdout(io.StringIO()):
            result = guard.run(self.args())
        self.assertEqual(result, 75)
        popen.assert_not_called()

    def test_runtime_low_space_stops_owned_command(self):
        process = types.SimpleNamespace(pid=123, poll=lambda: None)
        result, events, _, stop = self.execute([60 * guard.GIB] * 2 + [29 * guard.GIB] * 4, process)
        self.assertEqual(result, 74)
        self.assertIn('stop', [event['kind'] for event in events])
        stop.assert_called_once_with(process)

    def test_warning_and_then_stop(self):
        self.policy['profiles']['light']['max_run_mib'] = 50 * 1024
        self.policy['max_host_growth_gib'] = 50
        process = types.SimpleNamespace(pid=123, poll=lambda: None)
        result, events, _, _ = self.execute([60 * guard.GIB] * 2 + [39 * guard.GIB] * 2 +
                                          [29 * guard.GIB] * 4, process)
        self.assertEqual(result, 74)
        self.assertEqual([event['kind'] for event in events], ['start', 'warning', 'stop', 'stopped'])

    def test_budget_growth_outside_artifact_directory(self):
        process = types.SimpleNamespace(pid=123, poll=lambda: None)
        result, events, _, _ = self.execute([60 * guard.GIB] * 2 + [57 * guard.GIB] * 4, process)
        self.assertEqual(result, 74)
        self.assertIn('budget', events[-2]['detail'])

    def test_session_budget_blocks_without_deleting_files(self):
        self.policy['max_session_gib'] = 1 / guard.GIB
        artifacts = self.root / 'artifacts'
        artifacts.mkdir()
        evidence = artifacts / 'evidence.txt'
        evidence.write_text('keep this')
        result, _, popen, _ = self.execute([60 * guard.GIB] * 2)
        self.assertEqual(result, 73)
        popen.assert_not_called()
        self.assertEqual(evidence.read_text(), 'keep this')

    def test_child_completion_cleans_owned_group(self):
        process = types.SimpleNamespace(pid=123, poll=lambda: 0)
        result, events, _, stop = self.execute([60 * guard.GIB] * 4, process)
        self.assertEqual(result, 0)
        stop.assert_called_once_with(process)
        self.assertEqual(events[-1]['kind'], 'finished')

    def test_invalid_policy_fails_before_launch(self):
        self.policy['stop_free_gib'] = 100
        with patch.object(guard.subprocess, 'Popen') as popen:
            with self.assertRaises(ValueError):
                guard.run(self.args())
        popen.assert_not_called()

    def test_duration_limit_stops_child(self):
        process = types.SimpleNamespace(pid=123, poll=lambda: None)
        with patch.object(guard.time, 'monotonic', side_effect=[0, 1801]):
            result, events, _, _ = self.execute([60 * guard.GIB] * 6, process)
        self.assertEqual(result, 74)
        self.assertIn('duration', events[-2]['detail'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
