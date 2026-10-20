import json, math, time
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node=rclpy.create_node('rmul2027_scan_inspection',namespace='/red_standard_robot1')
values={}
for topic in ('rplidar_a2/scan','obstacle_scan'):
    node.create_subscription(LaserScan,topic,lambda msg,t=topic:values.update({t:msg}),qos_profile_sensor_data)
deadline=time.monotonic()+20
while len(values)<2 and time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
for name,msg in values.items():
    finite=[x for x in msg.ranges if math.isfinite(x)]
    print(json.dumps({'topic':name,'frame':msg.header.frame_id,'finite':len(finite),
      'total':len(msg.ranges),'min':min(finite) if finite else None,
      'max':max(finite) if finite else None}),flush=True)
node.destroy_node()
rclpy.shutdown()
