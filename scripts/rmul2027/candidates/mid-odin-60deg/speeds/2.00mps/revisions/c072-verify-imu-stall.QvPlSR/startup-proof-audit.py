"""Check startup fault fixtures against the real supervisor event stream."""
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
LIVE_LOGS = Path('/tmp/rmul-original-20261119/workspace/logs')
IMU = 'point_lio_imu_input_fault_before_odometry'
COMPONENT = 'component_load_response_timeout_before_odometry'


def inspect(label, reference, releases, triggers):
    case = Path((ROOT / reference).read_text().strip())
    match = re.search(r'Logs: .*/logs/([^\s]+)', (case / 'launcher.log').read_text())
    events_path = LIVE_LOGS / match[1] / 'imu-recovery/events.jsonl'
    events = [json.loads(line) for line in events_path.read_text().splitlines()]
    started = [row for row in events if row['kind'] == 'navigation_started']
    retried = [row for row in events if row['kind'] == 'startup_restart_authorized']
    released = [row for row in events if row['kind'] == 'release_verified']
    assert [row['epoch'] for row in released] == releases, label
    assert [(row['epoch'], row['trigger']) for row in retried] == triggers, label
    assert all(row['terminal'] and row['stationary'] for row in retried), label
    assert all(all(row['evidence']['gates'].values()) for row in released), label
    assert len({row['pid'] for row in started}) == len(started), label
    audits = [row for row in events if row['kind'] == 'owned_tree_audit']
    stopped = [row['pid'] for row in events if row['kind'] == 'navigation_stopped']
    assert len(audits) in (len(started) - 1, len(started)), label
    assert stopped[:len(started) - 1] == [row['pid'] for row in started[:-1]], label
    assert all(not row['remaining'] for row in audits), label
    for retry in retried:
        continued = next(row for row in released if row['steady'] > retry['steady'])
        assert not any(row['kind'] == 'gate_status' and row['status']['permit']
                       and retry['steady'] <= row['steady'] < continued['steady'] for row in events), label
    return dict(label=label, fixture=str(case), event_path=str(events_path), retries=retried,
                released=released, starts=[row['pid'] for row in started], stopped=stopped)


evidence = [
    inspect('cold IMU', 'startup-cold-final.txt', [0], [(0, IMU)]),
    inspect('component timeout', 'startup-component-owned.txt', [0], [(0, COMPONENT)]),
    inspect('IMU timeout followed by a latch after odometry had started', 'startup-late-odometry.txt',
            [0, 1], [(1, "point_lio_imu_input_fault_during_verification")]),
]
recovery_path = Path(evidence[-1]['event_path'])
recovery = [json.loads(line) for line in recovery_path.read_text().splitlines()]
ack = [row for row in recovery if row['kind'] == 'seed_ack' and row['matched'] and row['epoch'] == 1]
assert ack and evidence[-1]['retries'][-1]['steady'] < ack[-1]['steady'] < evidence[-1]['released'][-1]['steady']
assert [row['attempt'] for row in evidence[-1]['retries']] == [1]
assert 'IMU_INPUT_FAULT_LATCH previous=' in (recovery_path.parent / 'nav-epoch-1.log').read_text()
(ROOT / 'startup-proof-summary.json').write_text(json.dumps(evidence, indent=2) + '\n')
for case in evidence:
    print('PASS', case['label'], 'owned launch PIDs', case['starts'])
