import errno
import json
import os
from pathlib import Path
import time

class ContactLog:
 def __init__(self,path):
  self.path=Path(path);self.offset=0;self.pending=b'';self.inode=None
  self.rows=0;self.probe_pairs=0;self.real_pairs=0;self.retries=0;self.error_since=None
 def poll(self):
  try:
   with self.path.open('rb') as stream:
    stat=os.fstat(stream.fileno());inode=(stat.st_dev,stat.st_ino)
    if self.inode is not None and self.inode!=inode:raise RuntimeError('contact log identity changed')
    if stat.st_size<self.offset:raise RuntimeError('contact log shrank')
    count=stat.st_size-self.offset
    if count>8*1024*1024:raise RuntimeError('contact log backlog exceeded 8 MiB')
    stream.seek(self.offset);data=stream.read(count)
    if len(data)!=count:raise RuntimeError('contact log snapshot short read')
  except OSError as error:
   if error.errno not in [errno.EAGAIN,errno.ENODATA]:raise
   self.retries+=1
   if self.error_since is None:self.error_since=time.monotonic()
   if time.monotonic()-self.error_since>.5:raise RuntimeError('contact log unavailable for 0.5s') from error
   return []
  self.error_since=None;self.inode=inode;self.offset+=len(data)
  complete=self.pending+data;parts=complete.split(b'\n');self.pending=parts.pop()
  if len(self.pending)>1024*1024:raise RuntimeError('contact log incomplete record exceeded 1 MiB')
  real=[]
  for line in parts:
   if not line.strip():continue
   row=json.loads(line);pairs=row['message']['contact']
   if not isinstance(pairs,list):raise ValueError('invalid contact list')
   self.rows+=1;has_real=False
   for pair in pairs:
    names=[pair[key]['name'] for key in ['collision1','collision2']]
    if not all(isinstance(name,str) and name for name in names):raise ValueError('missing collision names')
    if any('c020_contact_probe::' in name for name in names):self.probe_pairs+=1
    else:self.real_pairs+=1;has_real=True
   if has_real:real.append(row)
  return real
 def summary(self):
  return {'rows':self.rows,'explicit_probe_pairs':self.probe_pairs,'real_pairs':self.real_pairs,'read_retries':self.retries,'pending_bytes':len(self.pending),'policy':'inspect every collision pair; only explicit c020_contact_probe pairs excluded, regardless of aggregate probe flag'}
