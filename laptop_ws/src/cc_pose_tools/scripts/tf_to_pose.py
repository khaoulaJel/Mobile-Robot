#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped
class Pose(Node):
    def __init__(self):
        super().__init__('tf_to_pose');self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(PoseStamped,'/robot_pose',10);self.create_timer(.1,self.tick)
    def tick(self):
        try:t=self.buffer.lookup_transform('map','base_link',rclpy.time.Time())
        except TransformException:return
        m=PoseStamped();m.header=t.header;m.pose.position.x=t.transform.translation.x;m.pose.position.y=t.transform.translation.y;m.pose.position.z=t.transform.translation.z;m.pose.orientation=t.transform.rotation;self.pub.publish(m)
rclpy.init();n=Pose()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
