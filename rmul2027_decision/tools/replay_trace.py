#!/usr/bin/env python3
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser(description='Offline ordered ownership audit of recorded decision events')
p.add_argument('events',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
rows=[json.loads(line) for line in a.events.read_text().splitlines() if line.strip()]
owned=None;accepted={};sends=[];terminals=[];cancels=[];faults=[];seen=set()
for row in rows:
 kind=row['kind'];token=row.get('token')
 if kind=='send':
  assert owned is None,('Competing send before previous terminal',owned,row)
  owned=token;sends.append(row)
 elif kind=='goal_response':
  assert token==owned,('Response for unowned token',row)
  if row['accepted']:
   assert row['uuid'] and row['uuid'] not in seen
   accepted[token]=row['uuid'];seen.add(row['uuid'])
  else:owned=None
 elif kind=='action_result':
  assert token==owned and accepted[token]==row['uuid'],('Unmatched result',row)
  if row['code'] in [1,2,3]:owned=None;terminals.append(row)
 elif kind=='cancel':
  assert token==owned,('Cancel without ownership',row)
  cancels.append(row)
 elif kind=='fault':faults.append(row)
summary=dict(passed=True,scope='ordered event replay audit; does not rerun ROS or physics',
             sends=len(sends),accepted=len(accepted),terminals=len(terminals),cancel_requests=len(cancels),
             faults=[x['detail'] for x in faults],unresolved_token=owned,
             successful_goals=[next(s['goal'] for s in sends if s['token']==t['token']) for t in terminals if t['code']==1])
if a.output:a.output.write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
