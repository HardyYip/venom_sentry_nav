import errno
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import stat
import time

SHM=Path('/dev/shm')
SEGMENT=re.compile(r'fastrtps_[0-9a-f]{16}')

def identity(path):
 s=path.lstat()
 return {'device':s.st_dev,'inode':s.st_ino,'uid':s.st_uid,'bytes':s.st_size}

def unlink_unused(path,expected):
 path=Path(path)
 if path.parent!=SHM or not SEGMENT.fullmatch(path.name):return {'deleted':False,'reason':'outside_segment_scope'}
 broken=[False]
 previous=signal.getsignal(signal.SIGIO)
 signal.signal(signal.SIGIO,lambda *_:broken.__setitem__(0,True))
 fd=None;leased=False
 try:
  fd=os.open(path,os.O_RDWR|os.O_NOFOLLOW|os.O_CLOEXEC|os.O_NONBLOCK)
  s=os.fstat(fd)
  if not stat.S_ISREG(s.st_mode) or s.st_uid!=os.getuid():return {'deleted':False,'reason':'owner_or_type_changed'}
  if (s.st_dev,s.st_ino)!=(expected['device'],expected['inode']):return {'deleted':False,'reason':'identity_changed'}
  fcntl.fcntl(fd,fcntl.F_SETOWN,os.getpid())
  try:fcntl.fcntl(fd,fcntl.F_SETLEASE,fcntl.F_WRLCK);leased=True
  except OSError as e:return {'deleted':False,'reason':'lease_refused','errno':e.errno}
  current=identity(path)
  if current['device']!=s.st_dev or current['inode']!=s.st_ino:return {'deleted':False,'reason':'identity_changed'}
  if broken[0] or fcntl.fcntl(fd,fcntl.F_GETLEASE)!=fcntl.F_WRLCK:return {'deleted':False,'reason':'lease_break_requested'}
  path.unlink()
  return {'deleted':True,'bytes':s.st_size,'proof':'owned segment, unchanged inode, exclusive kernel write lease'}
 except FileNotFoundError:return {'deleted':False,'reason':'already_removed'}
 except OSError as e:return {'deleted':False,'reason':'os_error','errno':e.errno}
 finally:
  if fd is not None:
   if leased:
    try:fcntl.fcntl(fd,fcntl.F_SETLEASE,fcntl.F_UNLCK)
    except OSError:pass
   os.close(fd)
  signal.signal(signal.SIGIO,previous)

class SharedMemoryScope:
 def __init__(self,token,report):
  self.token=token;self.report=Path(report);self.records={};self.errors=0
  self.baseline=set()
  if SHM.is_dir():
   for p in SHM.iterdir():
    if not SEGMENT.fullmatch(p.name):continue
    try:s=identity(p);self.baseline.add((s['device'],s['inode']))
    except OSError:pass
 def _add(self,name,pid,expected_inode=None):
  path=SHM/name
  if not SEGMENT.fullmatch(name):return
  try:
   if path.is_symlink():return
   data=identity(path)
   if data['uid']!=os.getuid() or (data['device'],data['inode']) in self.baseline:return
   if expected_inode is not None and data['inode']!=expected_inode:return
   key=(name,data['device'],data['inode'])
   if key not in self.records:self.records[key]={'name':name,**data,'observed_pids':[]}
   if pid not in self.records[key]['observed_pids']:self.records[key]['observed_pids'].append(pid)
  except OSError:pass
 def capture(self):
  if not self.token:return
  marker=('RMUL_STORAGE_RUN_ID='+self.token).encode()+b'\0'
  for proc in Path('/proc').iterdir():
   if not proc.name.isdigit():continue
   try:
    if proc.stat().st_uid!=os.getuid() or marker not in (proc/'environ').read_bytes():continue
    for line in (proc/'maps').read_text().splitlines():
     fields=line.split(maxsplit=5)
     if len(fields)==6 and fields[5].startswith('/dev/shm/'):
      name=fields[5].removesuffix(' (deleted)').split('/dev/shm/',1)[1]
      self._add(name,int(proc.name),int(fields[4]))
    for fd in (proc/'fd').iterdir():
     try:
      target=os.readlink(fd)
      if target.startswith('/dev/shm/'):
       self._add(target.removesuffix(' (deleted)').split('/dev/shm/',1)[1],int(proc.name),fd.stat().st_ino)
     except OSError:continue
   except (FileNotFoundError,ProcessLookupError):continue
   except (OSError,ValueError):self.errors+=1
 def cleanup(self):
  records=[]
  for record in self.records.values():
   records.append({**record,**unlink_unused(SHM/record['name'],record)})
  result={'created_at_epoch':time.time(),'enabled':bool(self.token),'ownership':'new since launch; mapped or opened by a process carrying this storage run token','deletion_proof':'own UID + unchanged inode + exclusive kernel write lease; busy or uncertain segments retained','capture_errors':self.errors,'deleted_files':sum(r['deleted'] for r in records),'deleted_bytes':sum(r.get('bytes',0) for r in records if r['deleted']),'records':records,'reference':'https://man7.org/linux/man-pages/man2/F_SETLEASE.2const.html'}
  self.report.write_text(json.dumps(result,indent=2)+'\n')
  return result
