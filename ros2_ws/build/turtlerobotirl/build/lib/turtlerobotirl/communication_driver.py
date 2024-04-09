import serial
import time
import rclpy
import re
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs

x = 0
y = 0
theta = 0
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

class CommunicationrDiver(Node):
		
	def __init__(self):
		super().__init__('communication_driver')
		
		time.sleep(3)
		ser.reset_input_buffer()
		timer_period = 0.1  # seconds
		self.timer = self.create_timer(timer_period, self.location_callback)
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
				
	def location_callback(self):
		time.sleep(0.1)
		if ser.in_waiting > 0:
			line = ser.readline().decode('utf-8')
			numbers = re.findall(r'-?\d+\.*\d*', line)
			x, y, theta = [float(num) for num in numbers]
		transform_stamped = TransformStamped()
		transform_stamped.header.stamp = self.get_clock().now().to_msg()
		transform_stamped.header.frame_id = "base_link"
		transform_stamped.child_frame_id = "odom" 
		transform_stamped.transform.translation.x = x
		transform_stamped.transform.translation.y = y
		transform_stamped.transform.rotation.z = theta

		self.tf_broadcaster.sendTransform(transform_stamped)
        	        
def main(args=None):
	rclpy.init(args=args)
	communication_driver = CommunicationrDiver()
	rclpy.spin(communication_driver)
	communication_driver.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	
