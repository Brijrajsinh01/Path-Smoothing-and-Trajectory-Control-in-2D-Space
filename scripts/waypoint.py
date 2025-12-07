#!/usr/bin/env python3
"""
Waypoint Generator Node with Progressive Deletion and Sequential Patterns
Generates waypoints in multiple patterns sequentially to demonstrate robustness.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import numpy as np
import math


class WaypointGenerator(Node):
    """Generate waypoints with progressive deletion and sequential patterns."""
    
    def __init__(self):
        super().__init__('waypoint_generator')
        
        # Publisher
        self.waypoint_pub = self.create_publisher(PoseArray, '/waypoints', 10)
        
        # Subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('pattern', 'sequential')  # Can be specific or 'sequential'
        self.declare_parameter('size', 3.0)
        self.declare_parameter('num_points', 8)
        self.declare_parameter('publish_rate', 3.0)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('enable_deletion', True)
        self.declare_parameter('sequential_mode', True)  # NEW!
        self.declare_parameter('pause_between_patterns', 2.0)  # Seconds between patterns
        
        self.pattern = self.get_parameter('pattern').value
        self.size = self.get_parameter('size').value
        self.num_points = self.get_parameter('num_points').value
        publish_rate = self.get_parameter('publish_rate').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.enable_deletion = self.get_parameter('enable_deletion').value
        self.sequential_mode = self.get_parameter('sequential_mode').value
        self.pause_between_patterns = self.get_parameter('pause_between_patterns').value
        
        # Sequential pattern list
        self.pattern_sequence = ['circle', 'square', 'zigzag', 'figure_eight', 'spiral']
        self.current_pattern_index = 0
        self.pattern_completed = False
        self.pause_timer = None
        
        # State variables
        self.waypoints = []
        self.robot_position = None
        self.initial_waypoint_count = 0
        self.total_patterns_completed = 0
        
        # Initialize first pattern
        if self.sequential_mode:
            self.load_next_pattern()
        else:
            self.waypoints = self.generate_waypoints(self.pattern)
            self.initial_waypoint_count = len(self.waypoints)
        
        # Timer to publish waypoints
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_waypoints)
        
        self.get_logger().info('='*60)
        self.get_logger().info('🚀 Waypoint Generator Started')
        self.get_logger().info('='*60)
        if self.sequential_mode:
            self.get_logger().info(f'Mode: SEQUENTIAL (all patterns)')
            self.get_logger().info(f'Pattern sequence: {" → ".join(self.pattern_sequence)}')
        else:
            self.get_logger().info(f'Mode: SINGLE PATTERN ({self.pattern})')
        self.get_logger().info(f'Waypoint tolerance: {self.waypoint_tolerance} m')
        self.get_logger().info(f'Progressive deletion: {self.enable_deletion}')
        self.get_logger().info('='*60)
    
    def load_next_pattern(self):
        """Load the next pattern in the sequence."""
        
        if self.current_pattern_index >= len(self.pattern_sequence):
            self.get_logger().info('='*60)
            self.get_logger().info('🎉 ALL PATTERNS COMPLETED!')
            self.get_logger().info(f'Total patterns executed: {self.total_patterns_completed}')
            self.get_logger().info('='*60)
            self.waypoints = []
            return
        
        pattern_name = self.pattern_sequence[self.current_pattern_index]
        self.waypoints = self.generate_waypoints(pattern_name)
        self.initial_waypoint_count = len(self.waypoints)
        self.pattern_completed = False
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'📍 PATTERN {self.current_pattern_index + 1}/{len(self.pattern_sequence)}: {pattern_name.upper()}')
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints')
        self.get_logger().info('='*60)
    
    def odom_callback(self, msg):
        """Update robot position and check for reached waypoints."""
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Check if robot reached first waypoint
        if self.enable_deletion and len(self.waypoints) > 0:
            self.check_and_delete_reached_waypoint()
    
    def check_and_delete_reached_waypoint(self):
        """Check if robot reached the first waypoint and delete it."""
        
        if self.robot_position is None or len(self.waypoints) == 0:
            return
        
        # Check distance to first waypoint
        first_wp = self.waypoints[0]
        distance = math.sqrt(
            (first_wp[0] - self.robot_position[0])**2 +
            (first_wp[1] - self.robot_position[1])**2
        )
        
        if distance < self.waypoint_tolerance:
            # Waypoint reached! Delete it
            deleted_wp = self.waypoints.pop(0)
            remaining = len(self.waypoints)
            progress = ((self.initial_waypoint_count - remaining) / 
                       self.initial_waypoint_count * 100)
            
            self.get_logger().info(
                f'✓ Waypoint reached: ({deleted_wp[0]:.2f}, {deleted_wp[1]:.2f}) | '
                f'Remaining: {remaining}/{self.initial_waypoint_count} ({progress:.0f}% complete)'
            )
            
            if remaining == 0:
                self.on_pattern_completed()
    
    def on_pattern_completed(self):
        """Handle pattern completion."""
        
        if self.pattern_completed:
            return  # Already handled
        
        self.pattern_completed = True
        self.total_patterns_completed += 1
        
        pattern_name = self.pattern_sequence[self.current_pattern_index]
        self.get_logger().info('')
        self.get_logger().info('─'*60)
        self.get_logger().info(f'✅ Pattern "{pattern_name.upper()}" COMPLETED!')
        self.get_logger().info('─'*60)
        
        if self.sequential_mode:
            self.current_pattern_index += 1
            
            if self.current_pattern_index < len(self.pattern_sequence):
                # Schedule next pattern after pause
                next_pattern = self.pattern_sequence[self.current_pattern_index]
                self.get_logger().info(f'⏳ Pausing {self.pause_between_patterns}s before next pattern: {next_pattern.upper()}')
                
                # Create one-shot timer for pause
                self.pause_timer = self.create_timer(
                    self.pause_between_patterns,
                    self.load_next_pattern_after_pause
                )
            else:
                self.load_next_pattern()  # Will show completion message
        else:
            self.get_logger().info('🎉 Mission complete!')
    
    def load_next_pattern_after_pause(self):
        """Load next pattern after pause timer expires."""
        if self.pause_timer:
            self.pause_timer.cancel()
            self.pause_timer = None
        self.load_next_pattern()
    
    def generate_waypoints(self, pattern):
        """Generate waypoints based on selected pattern."""
        
        if pattern == 'square':
            return self.generate_square()
        elif pattern == 'circle':
            return self.generate_circle()
        elif pattern == 'figure_eight':
            return self.generate_figure_eight()
        elif pattern == 'zigzag':
            return self.generate_zigzag()
        elif pattern == 'spiral':
            return self.generate_spiral()
        else:
            self.get_logger().warn(f'Unknown pattern: {pattern}, using square')
            return self.generate_square()
    
    def generate_square(self):
        """Generate square pattern waypoints."""
        half_size = self.size / 2
        return [
            (half_size, half_size),
            (half_size, -half_size),
            (-half_size, -half_size),
            (-half_size, half_size),
            (half_size, half_size)
        ]
    
    def generate_circle(self):
        """Generate circular pattern waypoints."""
        waypoints = []
        radius = self.size / 2
        for i in range(self.num_points):
            angle = 2 * np.pi * i / self.num_points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            waypoints.append((x, y))
        waypoints.append(waypoints[0])  # Close the loop
        return waypoints
    
    def generate_figure_eight(self):
        """Generate figure-eight pattern waypoints."""
        waypoints = []
        radius = self.size / 4
        for i in range(self.num_points * 2):  # More points for smoother figure-8
            t = 2 * np.pi * i / (self.num_points * 2)
            x = radius * np.sin(2 * t)
            y = radius * np.sin(t)
            waypoints.append((x, y))
        waypoints.append(waypoints[0])
        return waypoints
    
    def generate_zigzag(self):
        """Generate zigzag pattern waypoints."""
        waypoints = []
        num_zigs = 5
        for i in range(num_zigs + 1):
            x = i * (self.size / num_zigs) - self.size / 2
            y = (self.size / 3) if i % 2 == 0 else -(self.size / 3)
            waypoints.append((x, y))
        return waypoints
    
    def generate_spiral(self):
        """Generate spiral pattern waypoints."""
        waypoints = []
        num_turns = 3
        points_per_turn = self.num_points
        total_points = num_turns * points_per_turn
        
        for i in range(total_points):
            t = 2 * np.pi * i / points_per_turn
            r = (self.size / 2) * (i / total_points)
            x = r * np.cos(t)
            y = r * np.sin(t)
            waypoints.append((x, y))
        return waypoints
    
    def publish_waypoints(self):
        """Publish current waypoint list."""
        
        if len(self.waypoints) == 0:
            return
        
        waypoint_msg = PoseArray()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = 'odom'
        
        for wp in self.waypoints:
            pose = Pose()
            pose.position.x = wp[0]
            pose.position.y = wp[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            waypoint_msg.poses.append(pose)
        
        self.waypoint_pub.publish(waypoint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
