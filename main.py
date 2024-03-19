import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def laser_scan_callback(msg):
    angle_min = msg.angle_min  # Start angle of the scan
    angle_increment = msg.angle_increment  # Angular distance between measurements

    # Calculate the index corresponding to the angle of 270 degrees
    angle_desired = 270.0  # Angle in degrees
    index = int((angle_desired - angle_min) / angle_increment)

    # Check if the index is within valid range
    if 0 <= index < len(msg.ranges):
        # Access the range data for the object to the left of the lidar
        range_to_left = msg.ranges[index]
        print("Range to the left of the lidar: %.2f meters" % range_to_left)
    else:
        print("Invalid index. Angle may be outside the lidar's field of view.")


def main():
    rclpy.init()
    node = rclpy.create_node('lidar_object_to_left')
    subscription = node.create_subscription(LaserScan, '/scan', laser_scan_callback, 10)
    rclpy.spin(node)
