import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

rclpy.init()
node=Node('official_field_reference',namespace='/red_standard_robot1')
node.set_parameters([Parameter('use_sim_time',value=True)])
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
publisher=node.create_publisher(MarkerArray,'official_field_reference',qos)
folder=Path(__file__).resolve().parent
meshes=sorted((folder/'scene/models/rmul_2027_3v3/meshes/visual').glob('*.stl'))
assert meshes,'Official field meshes missing'
message=MarkerArray()
for index,path in enumerate(meshes):
    marker=Marker()
    marker.header.frame_id='map'
    marker.ns='official_cad_display_only'
    marker.id=index
    marker.type=Marker.MESH_RESOURCE
    marker.action=Marker.ADD
    marker.mesh_resource=path.as_uri()
    marker.mesh_use_embedded_materials=False
    marker.pose.position.x=4.8
    marker.pose.position.y=-3.0
    marker.pose.position.z=-.03
    marker.pose.orientation.w=1.0
    marker.scale.x=marker.scale.y=marker.scale.z=1.0
    marker.color.r=.35
    marker.color.g=.75
    marker.color.b=.85
    marker.color.a=.35
    message.markers.append(marker)
label=Marker()
label.header.frame_id='map'
label.ns='official_cad_display_only'
label.id=len(meshes)
label.type=Marker.TEXT_VIEW_FACING
label.action=Marker.ADD
label.pose.position.x=4.8
label.pose.position.y=1.6
label.pose.position.z=.25
label.pose.orientation.w=1.0
label.scale.z=.22
label.color.r=.4
label.color.g=.9
label.color.b=1.0
label.color.a=1.0
label.text='Official RMUL 2027 CAD reference'
message.markers.append(label)
publisher.publish(message)
node.create_timer(2.0,lambda:publisher.publish(message))
print(json.dumps({'field_reference_meshes':len(meshes),'frame':'map','world_to_map_xy':[4.8,-3.0],'display_z_offset':-.03,'navigation_input':False}),flush=True)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
