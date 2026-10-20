import argparse
import json
import os
from pathlib import Path
import re
import subprocess

parser=argparse.ArgumentParser();parser.add_argument('--record-file',type=Path);args=parser.parse_args()
def command(arguments):
 try:
  result=subprocess.run(arguments,capture_output=True,text=True,timeout=12)
  return {'returncode':result.returncode,'output':(result.stdout+result.stderr)[-12000:]}
 except (OSError,subprocess.TimeoutExpired) as error:return {'error':str(error)}
result={'environment':{key:os.environ.get(key) for key in ['DISPLAY','WAYLAND_DISPLAY','XDG_RUNTIME_DIR','QT_QPA_PLATFORM','GALLIUM_DRIVER','LIBGL_ALWAYS_SOFTWARE']},
        'gpu_device_present':Path('/dev/dxg').exists(),'glxinfo':command(['glxinfo','-B'])}
weston=Path('/mnt/wslg/weston.log')
if weston.exists():
 text=weston.read_text(errors='replace')
 modes=re.findall(r'RDP backend: use_gfxredir = (\d)',text)
 result['wslg']={'latest_use_gfxredir':int(modes[-1]) if modes else None,
                 'historical_shared_memory_eio':'rdp_allocate_shared_memory: Failed to open' in text and 'Input/output error' in text,
                 'interpretation':'Historical errors do not prove the current window is broken. GPU renderer success does not prove Windows received its surface.'}
result['rviz_x11_windows']=[line for line in command(['xwininfo','-root','-tree']).get('output','').splitlines() if 'RViz' in line]
output=json.dumps(result,ensure_ascii=False,indent=2)
if args.record_file:
 args.record_file.parent.mkdir(parents=True,exist_ok=True);args.record_file.write_text(output+'\n')
print(output)
