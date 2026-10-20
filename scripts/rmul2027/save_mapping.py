import datetime
import hashlib
import yaml
import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

workspace = Path(__file__).resolve().parents[4]
artifact = workspace / 'artifacts/rmul_2027_3v3'
runs = sorted((workspace / 'logs').glob('*-rmul2027-mapping'))
assert runs, 'No mapping run found'
run = runs[-1]
identity = json.loads((run / 'launches.json').read_text())
pid = identity['wrapper_pid']
process = Path(f'/proc/{pid}')
command = (process / 'cmdline').read_bytes()
assert b'run_simulation.py' in command and b'mapping' in command, 'Mapping wrapper identity changed'
assert process.stat().st_uid == os.getuid(), 'Mapping wrapper belongs to another user'
stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
staging = artifact / ('maps-' + stamp + '.staging')
staging.mkdir()
subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', str(staging / 'rmul_2027_3v3'), '--free', '0.196', '--occ', '0.65', '--fmt', 'pgm', '--ros-args',
                '-r', '__ns:=/red_standard_robot1',
                '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=30.0'], check=True, timeout=45)
os.kill(pid, signal.SIGINT)
deadline = time.monotonic() + 40
while process.exists() and time.monotonic() < deadline:
    fields = (process / 'stat').read_text().rsplit(') ',1)[1].split()
    if fields[0] == 'Z':
        break
    time.sleep(.2)
pcd = workspace / 'src/pb2025_sentry_nav/point_lio/PCD/scans.pcd'
assert pcd.is_file() and pcd.stat().st_mtime > (run/'launches.json').stat().st_mtime, 'Fresh Point-LIO PCD was not saved'
shutil.copy2(pcd, staging/'rmul_2027_3v3.pcd')
grid_metadata=yaml.safe_load((staging/'rmul_2027_3v3.yaml').read_text())
grid_metadata['image']='rmul_2027_3v3.pgm'
grid_metadata['free_thresh']=.196
(staging/'rmul_2027_3v3.yaml').write_text(yaml.safe_dump(grid_metadata,sort_keys=False))
metadata = {'mapping_run': str(run), 'spawn_world_xyz_yaw': [-4.8,3.0,.15,0],
            'pcd_frame': 'Point-LIO camera_init / lidar odometry, consumed by existing small_gicp loader',
            'map_frame': 'map, identity map-to-odom during mapping',
            'world_to_map_xy': {'translation': [4.8,-3.0], 'yaw': 0.0},
            'source_sha256': json.loads((artifact/'source/manifest.json').read_text())['sha256'],
            'coverage': 'Sensor-observed region; unknown cells are preserved',
            'slam_scan_topic': yaml.safe_load((Path(__file__).resolve().parent/'config/nav2_params.yaml').read_text())['slam_toolbox']['ros__parameters']['scan_topic'],
            'file_sha256': {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in staging.iterdir() if p.suffix in ['.pcd','.pgm','.yaml']}}
(staging/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
destination = artifact/'maps'
if destination.exists():
    destination.rename(artifact/('maps-backup-'+stamp))
staging.rename(destination)
raw_archive=artifact/'mapping-raw'
raw_archive.mkdir(exist_ok=True)
pcd.rename(raw_archive/(run.name+'-scans.pcd'))
print(json.dumps({'maps':str(destination),'pcd_bytes':(destination/'rmul_2027_3v3.pcd').stat().st_size}),flush=True)
