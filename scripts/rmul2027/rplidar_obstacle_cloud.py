from array import array
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

class RplidarObstacleCloud(Node):
 def __init__(self):
  super().__init__('rplidar_obstacle_cloud')
  self.publisher=self.create_publisher(PointCloud2,'rplidar_obstacles',qos_profile_sensor_data)
  self.subscription=self.create_subscription(LaserScan,'rplidar_a2/scan',self.convert,qos_profile_sensor_data)
 def convert(self,scan):
  values=array('f')
  for index,distance in enumerate(scan.ranges):
   if not math.isfinite(distance) or distance<scan.range_min or distance>scan.range_max:continue
   angle=scan.angle_min+index*scan.angle_increment
   values.extend((distance*math.cos(angle),distance*math.sin(angle),0.0,1.0))
  cloud=PointCloud2();cloud.header=scan.header;cloud.height=1;cloud.width=len(values)//4
  cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(['x','y','z','intensity'])]
  cloud.is_bigendian=sys.byteorder=='big';cloud.point_step=16;cloud.row_step=cloud.width*16
  cloud.is_dense=True;cloud.data=values.tobytes();self.publisher.publish(cloud)

def main():
 rclpy.init();node=RplidarObstacleCloud()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
