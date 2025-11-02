
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

class BumpGoNode(Node):
    def __init__(self):
        super().__init__('bump_go_kiwi')
        self.get_logger().info('Selfmade BumpGoNode initialized')

        self.FORWARD = 0
        self.BACKWARD = 1
        self.TURN = 2
        self.STOP = 3
        self.state = self.FORWARD
        self.state_ts = self.get_clock().now()

        self.SPEED_LINEAR = 0.5
        self.SPEED_ANGULAR = 0.5
        self.DISTANCE_THRESHOLD = 0.5 

        self.TIME_BACKING = 2.0
        self.TIME_TURNING = 2.0
        self.TIMEOUT_SCAN = 1.0

        self.last_scan = None
        self.scan_sub = self.create_subscription(
            LaserScan, 
            'input_scan', 
            self.scan_callback, 
            qos_profile_sensor_data)
        
        self.vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10)
        self.timer = self.create_timer(0.05, self.control_cycle)


    def scan_callback(self, msg):
        self.last_scan = msg

    def control_cycle(self):
        if self.last_scan is None:
            self.get_logger().warning('No laser scan is received')
            return
        
        out_vel = Twist()
        if self.state == self.FORWARD:
            out_vel.linear.x = self.SPEED_LINEAR
            if self.check_forward2back():
                self.go_state(self.BACKWARD)
            if self.check_forward2stop():
                self.go_state(self.STOP)

        elif self.state == self.BACKWARD:
            out_vel.linear.x = -self.SPEED_LINEAR
            if self.check_backward2turn():
                self.go_state(self.TURN)

        elif self.state == self.TURN:
            out_vel.angular.z = self.SPEED_ANGULAR
            if self.check_turn2forward():
                self.go_state(self.FORWARD)

        elif self.state == self.STOP:
            if self.check_forward2stop() is False:
                self.go_state(self.FORWARD)

        self.vel_pub.publish(out_vel)

    def go_state(self, new_state):
        self.get_logger().info(f'State is changed from {self.state} to {new_state}')
        self.state = new_state
        self.state_ts = self.get_clock().now()

    def check_forward2back(self):
        pos_to_check = round(len(self.last_scan.ranges) / 2)
        return self.last_scan.ranges[pos_to_check] < self.DISTANCE_THRESHOLD
    
    def check_forward2stop(self):
        last_scan_age = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return last_scan_age >= Duration(seconds=self.TIMEOUT_SCAN)
    
    def check_backward2turn(self):
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.TIME_BACKING)
    
    def check_turn2forward(self):
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.TIME_TURNING)

def main(args=None):
    rclpy.init(args=args)

    bump_go_node = BumpGoNode()

    rclpy.spin(bump_go_node)
    bump_go_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()