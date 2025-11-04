#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time
import random


class Walk(Node):
    def __init__(self):
        super().__init__('walk')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/base_scan', self.scan_callback, 10)

        # Timing
        self.start_time = time.time()
        self.last_update_time = time.time()
        
        self.distance_traveled = 0.0       
        self.reached_goal = False
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Laser sensor variables
        self.front_clear = True
        self.front_min = 10.0
        self.left_avg = 10.0
        self.right_avg = 10.0
        self.left_front_avg = 10.0
        self.right_front_avg = 10.0

        # State
        self.state = 'INIT_TURN'
        self.turn_dir = None
        self.turn_start_time = None
        self.state_start_time = time.time()
        
        # Stuck detection 
        self.stuck_counter = 0
        self.last_state_change = time.time()
        self.time_in_current_state = 0.0
        
        # Control parameters
        self.max_linear_speed = 0.5
        self.min_linear_speed = 0.15
        self.turn_speed = 0.5
        self.obstacle_threshold = 0.8
        
        self.twist = Twist()

        self.get_logger().info("Walker Starting...")

    def scan_callback(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        
        front_center = n // 2
        front_width = min(15, n // 10)
        diag_width = min(25, n // 8)
        #more precision with lidar
        def safe_slice(start, end):
            start = max(0, min(start, n))
            end = max(0, min(end, n))
            return msg.ranges[start:end]
        
        front = safe_slice(front_center - front_width, front_center + front_width)
        left_front = safe_slice(front_center + front_width, front_center + front_width + diag_width)
        right_front = safe_slice(front_center - front_width - diag_width, front_center - front_width)
        left = safe_slice(front_center + front_width + diag_width, n)
        right = safe_slice(0, front_center - front_width - diag_width)

        def avg_valid(data):
            valid = [d for d in data if not math.isinf(d) and d > 0.01]
            return sum(valid) / len(valid) if valid else 10.0
        
        def min_valid(data):
            valid = [d for d in data if not math.isinf(d) and d > 0.01]
            return min(valid) if valid else 10.0
        #Get averages of lidar lengths for objects

        self.left_avg = avg_valid(left)
        self.right_avg = avg_valid(right)
        self.left_front_avg = avg_valid(left_front)
        self.right_front_avg = avg_valid(right_front)
        self.front_min = min_valid(front)

        self.front_clear = self.front_min > self.obstacle_threshold

    def update_distance(self):
        current_time = time.time()
        dt = current_time - self.last_update_time
       
        if self.current_linear_vel > 0:
            self.distance_traveled += self.current_linear_vel * dt
        
        self.last_update_time = current_time
        

    def is_stuck_timebased(self):

        if self.state == 'TURN' and self.time_in_current_state > 8.0:
            return True
        # If we've been in any state too long
        if self.time_in_current_state > 15.0:
            return True
        return False

    def choose_turn_direction(self):

        left_score = self.left_front_avg * 2 + self.left_avg
        right_score = self.right_front_avg * 2 + self.right_avg
    
        if self.turn_dir == 'LEFT':
            left_score *= 1.15
        elif self.turn_dir == 'RIGHT':
            right_score *= 1.15
        
        return 'LEFT' if left_score > right_score else 'RIGHT'

    def get_adaptive_speed(self):

        if self.front_min < 1.0:
            return self.min_linear_speed
        elif self.front_min < 1.5:
            return (self.max_linear_speed + self.min_linear_speed) / 2
        else:
            return self.max_linear_speed

    def update_state(self):

        current_time = time.time()
        self.time_in_current_state = current_time - self.state_start_time
        
        # Check for stuck condition
        if self.is_stuck_timebased() and self.state != 'RECOVERY':

            self.state = 'RECOVERY'
            self.state_start_time = current_time
            self.stuck_counter += 1
            return

        # State machine
        if self.state == 'INIT_TURN':
            self.current_linear_vel = 0.0
            self.current_angular_vel = self.turn_speed
            
            if self.time_in_current_state > random.uniform(1.0, 2.5):
                self.state = 'FORWARD'
                self.state_start_time = current_time

        elif self.state == 'FORWARD':
            if not self.front_clear:
                self.turn_dir = self.choose_turn_direction()
                self.state = 'TURN'
                self.state_start_time = current_time
            else:
                self.current_linear_vel = self.get_adaptive_speed()
                
                # Wall following
                if self.left_front_avg < 0.8 and self.right_front_avg > self.left_front_avg + 0.3:
                    self.current_angular_vel = -0.1
                elif self.right_front_avg < 0.8 and self.left_front_avg > self.right_front_avg + 0.3:
                    self.current_angular_vel = 0.1
                else:
                    self.current_angular_vel = 0.0

        elif self.state == 'TURN':
            self.current_linear_vel = 0.0
            self.current_angular_vel = self.turn_speed if self.turn_dir == 'LEFT' else -self.turn_speed
            
            if self.front_clear and self.time_in_current_state > 1.0:
                self.state = 'FORWARD'
                self.state_start_time = current_time

        elif self.state == 'RECOVERY':
            if self.time_in_current_state < 1.5:
                # Back up
                self.current_linear_vel = -0.2
                self.current_angular_vel = 0.0
            elif self.time_in_current_state < 3.5:
                # Turn
                self.current_linear_vel = 0.0
                direction = random.choice([-1, 1])
                self.current_angular_vel = direction * self.turn_speed
            else:
                self.state = 'FORWARD'
                self.state_start_time = current_time

        # Update twist message
        self.twist.linear.x = self.current_linear_vel
        self.twist.angular.z = self.current_angular_vel

    def move(self):

        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except KeyboardInterrupt:
                self.get_logger().info("Keyboard interrupt")
                break

            self.update_distance()
            
            if time.time() - self.start_time > 320:
                self.get_logger().info("Time limit reached")
                break

            self.update_state()

            # Publish velocity
            self.pub.publish(self.twist)
            
            time.sleep(0.05)

        # Stop
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = Walk()
    node.move()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
