import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class SmartMover(Node):
    def __init__(self):
        super().__init__('smart_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.obstacle_near = False

    def laser_callback(self, msg):
        # Check if there are obstacles within 1 meter in front
        if min(msg.ranges) < 1.0:
            self.obstacle_near = True
        else:
            self.obstacle_near = False

    def move_forward(self, duration):
        if not self.obstacle_near:
            twist = Twist()
            twist.linear.x = 0.2
            end_time = time.time() + duration
            while time.time() < end_time and not self.obstacle_near:
                self.publisher_.publish(twist)
                time.sleep(0.1)
            twist.linear.x = 0
            self.publisher_.publish(twist)

    def turn(self, duration, clockwise=True):
        twist = Twist()
        twist.angular.z = -0.5 if clockwise else 0.5
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0
        self.publisher_.publish(twist)

    def execute_pattern(self):
        for _ in range(4):
            self.move_forward(5)  # Move forward for 5 seconds
            self.turn(2)          # Turn for 2 seconds
            if self.obstacle_near:  # Check for obstacles
                break

def main(args=None):
    rclpy.init(args=args)
    smart_mover = SmartMover()
    smart_mover.execute_pattern()
    rclpy.spin(smart_mover)
    smart_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
