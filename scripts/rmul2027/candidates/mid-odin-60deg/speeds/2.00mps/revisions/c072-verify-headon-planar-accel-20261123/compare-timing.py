"""Compare wall-clock controller cadence and physical corner motion."""
import bisect
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
LOGS = Path('/tmp/rmul-original-20261119/workspace/logs')
AREA = ((.71, 1.30), (-4.70, -4.20))
WARN = 'Control loop missed its desired rate of 20.0000Hz'


def percentile(values, percent):
    values = sorted(values)
    index = max(0, min(len(values)-1, round((len(values)-1) * percent)))
    return round(values[index] * 1000., 2)


def jsonlines(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def in_corner(row):
    return (AREA[0][0] <= row['x'] <= AREA[0][1] and AREA[1][0] <= row['y'] <= AREA[1][1])


def inspect(label):
    data = Path((ROOT / f'{label}.txt').read_text().strip())
    result = json.loads((data / 'review.json').read_text())['legs'][0]
    motion = [row for row in jsonlines(data / 'samples.jsonl') if row['leg'] == result['leg']]
    stamps = [row['sim'] for row in motion]
    controls = jsonlines(data / 'controller-command.jsonl')
    assert controls and all(row['leg'] == result['leg'] for row in controls)
    intervals = [next_['steady'] - previous['steady'] for previous, next_ in zip(controls, controls[1:])]
    corner = []
    for index, command in enumerate(controls[:-1]):
        nearest = min(bisect.bisect_left(stamps, command['sim']), len(stamps)-1)
        if in_corner(motion[nearest]):
            corner.append(intervals[index])
    match = re.search(r'Logs: .*/logs/([^\s]+)', (data/'launcher.log').read_text())
    logs = LOGS / match[1] / 'imu-recovery'
    lines = [line for file in sorted(logs.glob('nav-epoch-*.log')) for line in file.read_text().splitlines()]
    missed = sum(WARN in line for line in lines)
    contact = result['contacts']
    near_still = sum(right['sim'] - left['sim'] for left, right in zip(motion, motion[1:])
                     if in_corner(left) and left['speed'] < .03)
    transit = sum(right['sim'] - left['sim'] for left, right in zip(motion, motion[1:]) if in_corner(left))
    goals_complete = result['goal_status'] == 4 and result['stop_verified']
    unexpected = sum(contact.get(key, 0) for key in ('wall','box','unknown','elevated'))
    return dict(label=label,path=str(data),nav_logs=str(logs),goal=result['goal'],passed=goals_complete and unexpected == 0, goal_completed=goals_complete,
                contacts=contact,
                missed_20hz_warnings=missed,wall_run_s=round(sum(intervals),3),
                all_command_intervals=len(intervals),median_wall_ms=percentile(intervals,.5),
                p95_wall_ms=percentile(intervals,.95),p99_wall_ms=percentile(intervals,.99),
                over_75ms=sum(dt>.075 for dt in intervals),
                corner_command_intervals=len(corner),corner_wall_p95_ms=percentile(corner,.95) if corner else None,
                corner_transit_sim_s=round(transit,3),corner_near_still_sim_s=round(near_still,3))


records=[inspect(case) for case in ('prior-manual-abort','manual-abort','prior-manual-other','manual-other')]
(ROOT/'controller-cadence-comparison.json').write_text(json.dumps(records,indent=2)+'\n')
for record in records:
    print(record['label'],record['passed'],record['median_wall_ms'],record['p95_wall_ms'],
          record['missed_20hz_warnings'],record['corner_near_still_sim_s'])
