import serial
import time
import json
import rclpy
import re
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion, Point
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

class OdomTranslater(Node):
		
	def __init__(self):
		super().__init__('odom_translater')
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
		self.subscription = self.create_subscription(Odometry, 'odom', self.location_callback, 10)
		
	def location_callback(self, msg):
		transform_stamped = TransformStamped()
		transform_stamped.header.stamp = self.get_clock().now().to_msg()
		transform_stamped.header.frame_id = msg.header.frame_id
		transform_stamped.child_frame_id = msg.child_frame_id
		transform_stamped.transform.translation.x = msg.pose.pose.position.x
		transform_stamped.transform.translation.y = msg.pose.pose.position.y
		transform_stamped.transform.translation.z = 0.0

		transform_stamped.transform.rotation.x = msg.pose.pose.orientation.x
		transform_stamped.transform.rotation.y = msg.pose.pose.orientation.y
		transform_stamped.transform.rotation.z = msg.pose.pose.orientation.z
		transform_stamped.transform.rotation.w = msg.pose.pose.orientation.w
		

		self.tf_broadcaster.sendTransform(transform_stamped)
		
		

        	        
def main(args=None):
	rclpy.init(args=args)
	odom_translater = OdomTranslater()
	rclpy.spin(odom_translater)
	odom_translater.destroy_node()
	rclpy.shutdown()



if __name__ == '__main__':
	main()	
