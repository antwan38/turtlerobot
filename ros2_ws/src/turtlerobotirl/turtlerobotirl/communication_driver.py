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

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

class CommunicationrDiver(Node):

	
		
	def __init__(self):
		super().__init__('communication_driver')

		self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
		self.subscription = self.create_subscription(Twist, 'cmd_vel', cmd_vel_callback, 10)
		self.theta = 0.0
		self.x = 0.0
		self.y = 0.0
		time.sleep(3)
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
		ser.reset_input_buffer()
		timer_period = 0.1  # seconds
		self.timer = self.create_timer(timer_period, self.location_callback)
		
				
	def location_callback(self):
		time.sleep(0.1)
		self.euler_to_quaternion()
		if ser.in_waiting > 0:
			try:
				self.x, self.y, self.theta, self.lineair, self.angular   = ser.readline().decode('utf-8').split(",")
				self.x, self.y, self.theta, self.lineair, self.angular = map(float, [self.x, self.y, self.theta, self.lineair, self.angular])

			except Exception as e:
				print("Error reading serial data:", e)

			# odom_msg = Odometry()
			# odom_msg.header.stamp = self.get_clock().now().to_msg()
			# odom_msg.header.frame_id = 'odom'
			# odom_msg.child_frame_id = 'base_link'

			# odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
			# odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=self.theta, w=1.0)
			# odom_msg.twist.twist.linear = Vector3(x=self.linear, y=0.0, z=0.0)
			# odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.angular)

			transform_stamped = TransformStamped()
			transform_stamped.header.stamp = self.get_clock().now().to_msg()
			transform_stamped.header.frame_id = "odomtf"
			transform_stamped.child_frame_id = "base_linktf" 
			transform_stamped.transform.translation.x = self.x
			transform_stamped.transform.translation.y = self.y
			transform_stamped.transform.rotation = self.quaternion

			self.tf_broadcaster.sendTransform(transform_stamped)
			#self.publisher_.publish(odom_msg)
		
	def euler_to_quaternion(self):
		cy = np.cos(self.theta * 0.5)
		sy = np.sin(self.theta * 0.5)
		cp = np.cos(0)
		sp = np.sin(0)
		cr = np.cos(0)
		sr = np.sin(0)

		self.quaternion = Quaternion()
		self.quaternion.x = cy * cp * sr - sy * sp * cr
		self.quaternion.y = cy * sp * cr + sy * cp * sr
		self.quaternion.z = sy * cp * cr - cy * sp * sr
		self.quaternion.w = cy * cp * cr + sy * sp * sr

        	        
def main(args=None):
	rclpy.init(args=args)
	communication_driver = CommunicationrDiver()
	rclpy.spin(communication_driver)
	communication_driver.destroy_node()
	rclpy.shutdown()

def cmd_vel_callback(msg):
	output = str(msg.linear.x )+","+ str(msg.angular.z)
	ser.write(output.encode())


if __name__ == '__main__':
	main()
	
