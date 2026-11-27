#!/bin/bash
set -euo pipefail
d=/trial/contact-positive
mkdir -p "$d"
python3 /trial/scene.py "$d" /baseline/fixed-left/cases/left-near-01
python3 - "$d/world.sdf" <<'PY'
from pathlib import Path
import sys,xml.etree.ElementTree as E
p=Path(sys.argv[1]);r=E.parse(p);r.find(".//model[@name='box0']/pose").text='0.30 0 0 0 0 0';r.write(p)
PY
ign gazebo -s --headless-rendering "$d/world.sdf" > "$d/gazebo.log" 2>&1 &
sim=$!
trap 'kill -TERM "$sim" 2>/dev/null || true' EXIT
for n in $(seq 1 50); do
    if ign service -l 2>/dev/null | grep -q /world/navigation/control;then break;fi
    sleep .2
done
topic=/world/navigation/model/cart/link/body/sensor/contacts/contact
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge "$topic@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts" > "$d/bridge.log" 2>&1 &
bridge=$!
trap 'kill -TERM "$bridge" "$sim" 2>/dev/null || true' EXIT
TOPIC="$topic" timeout 20 python3 - <<'PY'
import json,os,subprocess,time,rclpy
from pathlib import Path
from ros_gz_interfaces.msg import Contacts
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('contact_positive');hits=[]
def seen(msg):
    for c in msg.contacts:
        hits.append([c.collision1.name,c.collision1.id,c.collision2.name,c.collision2.id,len(c.positions)])
n.create_subscription(Contacts,os.environ['TOPIC'],seen,qos_profile_sensor_data)
time.sleep(1)
subprocess.run(['ign','service','-s','/world/navigation/control','--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean','--timeout','3000','--req','pause: false'],check=True)
end=time.monotonic()+9
while not hits and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
Path('/trial/contact-positive/check.json').write_text(json.dumps({'passed':bool(hits),'contacts':hits[:5]},indent=2))
n.destroy_node();rclpy.shutdown()
if not hits:raise RuntimeError('No positive contact message')
PY
