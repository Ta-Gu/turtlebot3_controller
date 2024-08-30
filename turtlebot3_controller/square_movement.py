import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class SquareMovement(Node):
    def __init__(self):
        super().__init__('advanced_movement')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.obstacle_detected = False

    def laser_callback(self, msg):
        # Check if there's an obstacle within 0.5 meters in front of the robot
        self.obstacle_detected = any(distance < 0.5 for distance in msg.ranges[len(msg.ranges)//3:2*len(msg.ranges)//3])

    def move_in_square(self):
        for _ in range(4):  # Repeat 4 times to make a square
            self.move_forward(2.0)  # Move forward or until an obstacle is detected
            self.turn(90)  # Turn 90 degrees to the left

    def move_forward(self, duration):
        start_time = time.time()
        twist = Twist()
        twist.linear.x = 0.3

        while time.time() - start_time < duration:
            if self.obstacle_detected:
                self.bypass_obstacle()
                start_time = time.time()  # Reset the timer after bypassing
            else:
                self.publisher_.publish(twist)
            time.sleep(0.1)

    def turn(self, angle):
        twist = Twist()
        twist.angular.z = 0.5
        duration = angle / 45.0  # Assume it turns 45 degrees per second

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(twist)
            time.sleep(0.1)

    def bypass_obstacle(self):
        # Back up slightly
        twist = Twist()
        twist.linear.x = -0.1
        self.publisher_.publish(twist)
        time.sleep(1.0)

        # Turn to avoid the obstacle
        twist = Twist()
        twist.angular.z = 2.0
        self.publisher_.publish(twist)
        time.sleep(1.5)

        # Continue forward
        twist = Twist()
        twist.linear.x = 0.2
        self.publisher_.publish(twist)
        time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    movement_node = SquareMovement()
    
    print("Starting advanced movement sequence...")
    movement_node.move_in_square()

    movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
