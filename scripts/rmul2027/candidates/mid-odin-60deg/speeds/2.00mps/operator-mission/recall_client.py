"""Inject one operator service request after movement on the second waypoint leg."""
import json,sys,time
from pathlib import Path
import rclpy
from std_srvs.srv import Trigger
d=Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('simulated_operator')
client=node.create_client(Trigger,'/red_standard_robot1/operator_recall')
assert client.wait_for_service(timeout_sec=120)
deadline=time.monotonic()+240
while time.monotonic()<deadline:
    path=d/'trajectory.jsonl'
    if path.exists():
        rows=[json.loads(line) for line in path.read_text().splitlines() if line.endswith('}')]
        moving=[r for r in rows if r['leg']=='patrol_B' and r['speed']>.10]
        if len(moving)>=3:
            future=client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            assert future.done() and future.result().success
            print('Operator recall accepted',flush=True)
            break
    time.sleep(.08)
else:raise TimeoutError('No moving B leg')
node.destroy_node();rclpy.shutdown()
