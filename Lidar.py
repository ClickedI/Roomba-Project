import rclpy
from rclpy.node import Node
from irobot_edu_sdk.backend.usb import USB
from irobot_edu_sdk.robots import Robot, Create3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

robot = Create3(USB())


async def ir_prox():
    num = 0
    sensors = (await robot.get_ir_proximity()).sensors
    sensor1 = sensors[0]
    sensor2 = sensors[1]
    sensor3 = sensors[2]
    sensor4 = sensors[3]
    sensor5 = sensors[4]
    sensor6 = sensors[5]
    sensor7 = sensors[6]
    sensor8 = sensors[7]
    if sensors[0:7] > num:
        exit(2)


class WallFollow(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.ranges = []
        self.start_time = 0
        self.duration = 3
        self.wall_found = False

    def lidar_callback(self, msg):
        ranges = msg.ranges
        front = ranges[475:625]
        left = ranges[810]

        if not self.wall_found:
            if (all(value > .4 for value in front) or left > .3) and time.time() - self.start_time <= self.duration:
                self.move_robot(0, 0.5)
            elif time.time() - self.start_time >= self.duration:
                self.move_robot(.5, 0)
            elif front < .4 or left < .3:
                self.wall_found = True

        if self.wall_found:
            if all(value < .4 for value in front):
                self.move_robot(0, .5)
                print("front", front)
                print("left", left)
            elif .3 < left < .6:
                self.move_robot(.1, -.1)
            elif left > .6 and front > .4:
                self.move_robot(.5, 0)
                time.sleep(1)
                self.move_robot(0, .5)
                time.sleep(3)
                self.move_robot(.5,0)
            else:
                self.move_robot(.5, 0)

    def move_robot(self, speed, angle):
        twist_msg = Twist()
        twist_msg.linear.x = float(speed)
        twist_msg.angular.z = float(angle)
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init()
    lidar_subscriber = WallFollow()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()