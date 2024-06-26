#Author Robert Pellegrino Copyright 2024
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time



async def ir_prox():
    num = 1
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
        self.ranges = []  # List to store LIDAR range data
        self.start_time = 0  # Time when wall-following behavior started
        self.duration = 3  # Duration for which the robot follows the wall before turning
        self.wall_found = False  # Flag indicating if a wall is detected
        self.kp = 0.5  # Proportional gain for the PID controller
        self.ki = 0.0  # Integral gain for the PID controller
        self.kd = 0.1  # Derivative gain for the PID controller
        self.error_integral = 0.0  # Integral of the error for the PID controller
        self.last_error = 0.0  # Previous error for the PID controller
        self.desired_distance = 0.3  # Desired distance from the wall
        self.min_speed = 0.1  # Minimum linear speed for the robot
        self.max_speed = 0.5  # Maximum linear speed for the robot
        self.speed_gain = 0.1  # Gain to adjust linear speed based on error
        self.obstacle_threshold = 0.3  # Threshold distance to detect obstacles
        self.obstacle_avoidance_speed = 0.3  # Speed to avoid obstacles
        self.corner_threshold = 0.2  # Threshold to detect corners
        self.corner_turn_speed = 0.1  # Speed for turning at corners
        self.corner_turn_angle = -0.5  # Angle for turning at corners (negative for left turn)
        self.wall_following_active = True  # Flag indicating if wall-following behavior is active

    def pid_control(self, error):
        # Calculate PID terms
        self.error_integral += error
        derivative = error - self.last_error
        self.last_error = error

        # Calculate control signal
        control_signal = self.kp * error + self.ki * self.error_integral + self.kd * derivative
        return control_signal

    def lidar_callback(self, msg):
        ranges = msg.ranges
        front = ranges[500:600]
        left = ranges[810]
        error = self.desired_distance - left

        # Check for obstacle
        if min(front) < self.obstacle_threshold:
            # If obstacle detected, adjust behavior for obstacle avoidance
            self.move_robot(self.obstacle_avoidance_speed, -0.5)  # Move away from the obstacle
            self.wall_following_active = False
        else:
            # Continue wall following if no obstacle
            self.wall_following_active = True
            # Check for corner
            if min(front) - left > self.corner_threshold:
                self.move_robot(self.corner_turn_speed, self.corner_turn_angle)
            else:
                # Continue wall-following behavior
                linear_speed = max(self.min_speed, self.max_speed - abs(error * self.speed_gain))
                angle = self.pid_control(error)

                # Check for approaching wall
                if min(front) < self.desired_distance *2:  # Adjust the multiplier as needed
                    # Turn at a 90-degree angle
                    self.move_robot(0, .5)  # Stop linear motion and turn at full speed
                    time.sleep(1)  # Adjust the sleep duration as needed to turn 90 degrees
                else:
                    self.move_robot(linear_speed, angle)


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
