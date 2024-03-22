import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
	
	def __init__(self):
		super().__init__('lidar_subscriber')
		self.subscription = self.create_subscription(
		LaserScan,
		'/scan',
		self.lidar_callback,
		10)
		self.subscription
		
	def lidar_callback(self, msg):
		ranges = msg.ranges
		print("left", ranges[810])
		print("front", ranges[540])


def main(args=None):
	rclpy.init()
	lidar_subscriber = LidarSubscriber()
	rclpy.spin(lidar_subscriber)
	lidar_subscriber.destroy_node()
	rclpy.shutdown()
 	
if __name__ == '__main__':
	main()

		
