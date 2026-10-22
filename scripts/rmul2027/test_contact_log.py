if __name__ == '__main__':
 from storage_entry import ensure_storage_guard
 ensure_storage_guard(__file__)

import argparse,errno,json,os
from pathlib import Path
from unittest.mock import patch
from contact_log import ContactLog
p=argparse.ArgumentParser()
p.add_argument('--record-dir',type=Path,required=True)
p.add_argument('--replay',type=Path,required=True)
args=p.parse_args();root=args.record_dir.resolve();root.mkdir(parents=True,exist_ok=False)
checks=[]
def passed(name):checks.append({'name':name,'passed':True})
def pair(first,second):return {'collision1':{'name':first},'collision2':{'name':second}}
probe=pair('c020_opponent_cart::cart::cart_collision','c020_contact_probe::probe::probe_collision')
actual=pair('c020_opponent_cart::cart::cart_collision','red_standard_robot1::armor::collision')
def row(pairs,flag=False):return json.dumps({'sim':100,'probe':flag,'message':{'contact':pairs}}).encode()+b'\n'
def append(path,data):
 with path.open('ab') as stream:stream.write(data)
target=root/'contacts.jsonl';target.write_bytes(args.replay.read_bytes())
reader=ContactLog(target)
assert reader.poll()==[] and reader.rows>0 and reader.real_pairs==0
passed('actual failed-round replay contains only explicit probe collision pairs')
append(target,row([actual]));assert len(reader.poll())==1
passed('new real collision detected after probe replay')
data=row([actual]);half=len(data)//2
append(target,data[:half]);assert reader.poll()==[] and reader.pending
append(target,data[half:]);assert len(reader.poll())==1 and not reader.pending
passed('incomplete appended line retained until complete')
append(target,row([probe,actual],True));assert len(reader.poll())==1
passed('mixed real and probe contacts detected even with aggregate probe true')
append(target,row([probe],True));assert reader.poll()==[]
passed('delayed explicit positive-control pair excluded')
with patch.object(Path,'open',side_effect=OSError(errno.ENODATA,'fixture')):
 assert reader.poll()==[] and reader.retries==1
append(target,row([actual]));assert len(reader.poll())==1
passed('transient ENODATA retries without losing subsequent contact')
target.write_bytes(b'')
try:reader.poll();raise AssertionError('truncation accepted')
except RuntimeError:passed('truncation fails closed')
target.unlink();target.write_bytes(row([actual]))
try:reader.poll();raise AssertionError('replacement accepted')
except RuntimeError:passed('inode replacement fails closed')
bad=root/'malformed.jsonl';bad.write_bytes(b'{broken}\n')
try:ContactLog(bad).poll();raise AssertionError('malformed input accepted')
except ValueError:passed('malformed complete record fails closed')
missing=ContactLog(root/'not-created.jsonl')
try:missing.poll();raise AssertionError('missing log accepted')
except FileNotFoundError:passed('missing contact source fails closed')
result={'passed':True,'checks':checks,'replayed_source':str(args.replay),'source_sha256':__import__('hashlib').sha256(args.replay.read_bytes()).hexdigest(),'monitor':reader.summary()}
(root/'result.json').write_text(json.dumps(result,indent=2)+'\n')
for path in [target,bad]:path.unlink()
print(json.dumps(result))
