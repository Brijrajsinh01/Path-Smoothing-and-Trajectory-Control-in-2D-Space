#!/usr/bin/env python3
"""
Path Executor Node - OPTIMIZED
Executes smooth path with realistic velocity control:
- Smooth acceleration/deceleration
- Look-ahead for upcoming turns
- Velocity reduction based on turn sharpness
- Gradual velocity transitions
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
import csv
from datetime import datetime
import os


class PathExecutor(Node):
    """Execute smooth path using Pure Pursuit with optimized dynamic velocity."""
    
    def __init__(self):
        super().__init__('path_executor')
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/smooth_path',
            self.path_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('look_ahead_distance', 0.3)
        self.declare_parameter('max_linear_velocity', 0.7)
        self.declare_parameter('min_linear_velocity', 0.08)  # Lower minimum for sharp turns
        self.declare_parameter('max_angular_velocity', 2.84)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('control_frequency', 20.0)
        
        # NEW: Advanced velocity control parameters
        self.declare_parameter('max_acceleration', 1.0)  # m/s^2
        self.declare_parameter('max_deceleration', 1.0)  # m/s^2 (can brake faster)
        self.declare_parameter('turn_look_ahead_distance', 1.0)  # Look ahead for turns
        self.declare_parameter('sharp_turn_threshold', 0.5)  # rad (~30 degrees)
        self.declare_parameter('velocity_smoothing_factor', 0.5)  # 0-1, higher = smoother
        
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.min_linear_velocity = self.get_parameter('min_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        control_frequency = self.get_parameter('control_frequency').value
        
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_deceleration = self.get_parameter('max_deceleration').value
        self.turn_look_ahead_distance = self.get_parameter('turn_look_ahead_distance').value
        self.sharp_turn_threshold = self.get_parameter('sharp_turn_threshold').value
        self.velocity_smoothing_factor = self.get_parameter('velocity_smoothing_factor').value
        
        # State variables
        self.path = None
        self.current_pose = None
        self.current_index = 0
        self.goal_reached = False
        self.path_start_time = None
        
        # NEW: Velocity state for smooth transitions
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_control_time = None
        
        # CSV Logging setup
        # Save to task_ws/src/my_bot/config directory
        config_dir = os.path.expanduser('~/task_ws/src/my_bot/config')
        os.makedirs(config_dir, exist_ok=True)
        csv_path = os.path.join(config_dir, 'trajectory_execution.csv')
        
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'time_seconds', 'x', 'y', 'theta',
            'linear_velocity', 'angular_velocity', 'curvature',
            'current_curv', 'upcoming_curv'
        ])
        self.csv_file.flush()
        self.start_time = None
        self.csv_path = csv_path
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / control_frequency,
            self.control_loop
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('🚀 Path Executor Started - OPTIMIZED VERSION')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Algorithm: Pure Pursuit with Advanced Velocity Control')
        self.get_logger().info(f'Look-ahead distance: {self.look_ahead_distance} m')
        self.get_logger().info(f'Turn look-ahead: {self.turn_look_ahead_distance} m')
        self.get_logger().info(f'Control frequency: {control_frequency} Hz')
        self.get_logger().info(f'Velocity range: {self.min_linear_velocity:.2f}-{self.max_linear_velocity:.2f} m/s')
        self.get_logger().info(f'Max acceleration: {self.max_acceleration:.2f} m/s²')
        self.get_logger().info(f'Max deceleration: {self.max_deceleration:.2f} m/s²')
        self.get_logger().info(f'📊 Logging to: {self.csv_path}')
        self.get_logger().info('='*60)
    
    def path_callback(self, msg):
        """Receive smooth path and start execution with smooth transitions."""
        
        if len(msg.poses) < 2:
            self.get_logger().warn('Path too short')
            return
        
        # Check if this is the first path or a completely new pattern
        is_first_path = (self.path is None)
        
        # Update path
        old_path = self.path
        self.path = msg
        self.goal_reached = False
        
        if is_first_path:
            # First path - start from beginning with zero velocity
            self.current_index = 0
            self.current_velocity = 0.0
            self.target_velocity = 0.0
            self.path_start_time = self.get_clock().now()
            self.last_control_time = self.get_clock().now()
            self.get_logger().info('🚀 First path - starting from zero')
        else:
            # Path update - maintain velocity and find closest point on new path
            self.current_index = self.find_closest_path_index()
            # DON'T reset velocity - maintain momentum!
            self.get_logger().info(
                f'🔄 Path updated - maintaining velocity {self.current_velocity:.2f} m/s, '
                f'continuing from index {self.current_index}',
                throttle_duration_sec=2.0
            )
        
        # Calculate total path length
        total_length = self.calculate_path_length(msg.poses)
        
        self.get_logger().info(
            f'✓ Path: {len(msg.poses)} poses, {total_length:.2f} m',
            throttle_duration_sec=2.0
        )
    
    def odom_callback(self, msg):
        """Update current robot pose from odometry."""
        self.current_pose = msg.pose.pose
    
    def calculate_path_length(self, poses):
        """Calculate total length of the path."""
        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            length += math.sqrt(dx**2 + dy**2)
        return length
    
    def find_closest_path_index(self):
        """Find the closest point on the new path to current robot position."""
        
        if self.current_pose is None or self.path is None:
            return 0
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        min_distance = float('inf')
        closest_index = 0
        
        # Find closest point on new path
        for i, pose in enumerate(self.path.poses):
            dx = pose.pose.position.x - current_x
            dy = pose.pose.position.y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # Start slightly ahead to avoid going backwards
        # Look ahead by a few points to maintain forward progress
        look_ahead_offset = min(5, len(self.path.poses) - closest_index - 1)
        return min(closest_index + look_ahead_offset, len(self.path.poses) - 1)
    
    def control_loop(self):
        """Main control loop - called at control frequency."""
        
        if self.path is None or self.current_pose is None:
            return
        
        if self.goal_reached:
            self.publish_velocity(0.0, 0.0)
            return
        
        # Calculate time delta
        current_time = self.get_clock().now()
        if self.last_control_time is not None:
            dt = (current_time - self.last_control_time).nanoseconds / 1e9
        else:
            dt = 0.05  # Default 50ms
        self.last_control_time = current_time
        
        # Find look-ahead point
        look_ahead_point, look_ahead_index = self.find_look_ahead_point()
        
        if look_ahead_point is None:
            # Goal reached - smooth deceleration
            self.goal_reached = True
            self.smooth_stop(dt)
            
            elapsed_time = (current_time - self.path_start_time).nanoseconds / 1e9
            self.get_logger().info(f'🎯 Goal reached in {elapsed_time:.2f} seconds!')
            return
        
        # NEW: Calculate target velocity based on multiple factors
        self.target_velocity, current_curv, upcoming_curv = self.calculate_optimal_velocity(look_ahead_index)
        
        # NEW: Smooth velocity transition with acceleration limits
        linear_vel = self.smooth_velocity_transition(self.target_velocity, dt)
        
        # Pure pursuit control
        angular_vel = self.pure_pursuit_control(look_ahead_point, linear_vel)
        
        # NEW: Further reduce velocity if angular velocity is high (sharp turn)
        linear_vel = self.adjust_velocity_for_angular(linear_vel, angular_vel)
        
        # Publish velocity commands
        self.publish_velocity(linear_vel, angular_vel)
        
        # CSV Logging: Log trajectory data
        self.log_trajectory_data(linear_vel, angular_vel, current_curv, upcoming_curv)
        
        # Log progress with curvature information
        progress = (self.current_index / len(self.path.poses)) * 100
        self.get_logger().info(
            f'Progress: {progress:.0f}% | Vel: {linear_vel:.2f} m/s (target: {self.target_velocity:.2f}) | '
            f'Ang: {angular_vel:.2f} rad/s | Curv: {current_curv:.3f} | Upcoming: {upcoming_curv:.3f}',
            throttle_duration_sec=1.0
        )
    
    def calculate_optimal_velocity(self, look_ahead_index):
        """
        Calculate optimal velocity based on:
        1. Current path curvature
        2. Upcoming turn severity (look-ahead)
        3. Distance to goal
        
        Returns: (optimal_velocity, current_curvature, upcoming_curvature)
        """
        
        # Factor 1: Current curvature
        current_curvature = self.calculate_curvature(look_ahead_index)
        velocity_from_curvature = self.velocity_from_curvature(current_curvature)
        
        # Factor 2: Upcoming turns (look ahead further)
        upcoming_curvature = self.calculate_upcoming_curvature(look_ahead_index)
        velocity_from_upcoming = self.velocity_from_curvature(upcoming_curvature)
        
        # Factor 3: Distance to goal (slow down near end)
        distance_to_goal = self.calculate_distance_to_goal()
        velocity_from_goal = self.velocity_from_goal_distance(distance_to_goal)
        
        # Take the minimum (most conservative)
        optimal_velocity = min(
            velocity_from_curvature,
            velocity_from_upcoming,
            velocity_from_goal,
            self.max_linear_velocity
        )
        
        return max(optimal_velocity, self.min_linear_velocity), current_curvature, upcoming_curvature
    
    def velocity_from_curvature(self, curvature):
        """
        Calculate velocity based on curvature.
        More aggressive slowdown for sharp turns.
        """
        
        if curvature < 0.1:  # Nearly straight
            return self.max_linear_velocity
        elif curvature < 0.3:  # Gentle curve
            return self.max_linear_velocity * 0.7
        elif curvature < 0.6:  # Moderate turn
            return self.max_linear_velocity * 0.6
        elif curvature < 1.0:  # Sharp turn
            return self.max_linear_velocity * 0.3
        else:  # Very sharp turn
            return self.min_linear_velocity
    
    def calculate_upcoming_curvature(self, current_index):
        """
        Look ahead and find maximum curvature in upcoming path segment.
        This allows robot to slow down BEFORE entering a turn.
        """
        
        if self.path is None or current_index >= len(self.path.poses) - 1:
            return 0.0
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        max_curvature = 0.0
        accumulated_distance = 0.0
        
        # Look ahead along the path
        for i in range(current_index, min(current_index + 50, len(self.path.poses))):
            pose = self.path.poses[i]
            dx = pose.pose.position.x - current_x
            dy = pose.pose.position.y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            accumulated_distance += distance
            
            if accumulated_distance > self.turn_look_ahead_distance:
                break
            
            curvature = self.calculate_curvature(i)
            max_curvature = max(max_curvature, curvature)
        
        return max_curvature
    
    def calculate_distance_to_goal(self):
        """Calculate remaining distance to goal along the path."""
        
        if self.path is None or self.current_index >= len(self.path.poses):
            return 0.0
        
        distance = 0.0
        for i in range(self.current_index, len(self.path.poses) - 1):
            p1 = self.path.poses[i].pose.position
            p2 = self.path.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            distance += math.sqrt(dx**2 + dy**2)
        
        return distance
    
    def velocity_from_goal_distance(self, distance):
        """
        Slow down as approaching goal.
        Allows smooth stop instead of abrupt halt.
        """
        
        if distance > 1.0:  # Far from goal
            return self.max_linear_velocity
        elif distance > 0.5:  # Approaching goal
            return self.max_linear_velocity * 0.5
        elif distance > 0.3:  # Near goal
            return self.max_linear_velocity * 0.7
        else:  # Very close to goal
            return self.min_linear_velocity
    
    def smooth_velocity_transition(self, target_velocity, dt):
        """
        Smoothly transition from current velocity to target velocity.
        Respects acceleration and deceleration limits.
        """
        
        velocity_diff = target_velocity - self.current_velocity
        
        if velocity_diff > 0:  # Accelerating
            max_change = self.max_acceleration * dt
        else:  # Decelerating
            max_change = self.max_deceleration * dt
        
        # Limit velocity change
        velocity_change = np.clip(velocity_diff, -max_change, max_change)
        
        # Apply smoothing factor (exponential moving average)
        smoothed_change = velocity_change * (1.0 - self.velocity_smoothing_factor)
        
        # Update current velocity
        self.current_velocity += smoothed_change
        self.current_velocity = np.clip(
            self.current_velocity,
            self.min_linear_velocity,
            self.max_linear_velocity
        )
        
        return self.current_velocity
    
    def adjust_velocity_for_angular(self, linear_vel, angular_vel):
        """
        Further reduce linear velocity if angular velocity is high.
        Prevents robot from going too fast while turning sharply.
        """
        
        angular_magnitude = abs(angular_vel)
        
        if angular_magnitude > 2.0:  # Very sharp turn
            return linear_vel * 0.5
        elif angular_magnitude > 1.0:  # Sharp turn
            return linear_vel * 0.7
        elif angular_magnitude > 0.5:  # Moderate turn
            return linear_vel * 0.85
        else:  # Gentle turn or straight
            return linear_vel
    
    def smooth_stop(self, dt):
        """Smoothly decelerate to stop."""
        
        if self.current_velocity > 0.01:
            decel = self.max_deceleration * dt
            self.current_velocity = max(0.0, self.current_velocity - decel)
            self.publish_velocity(self.current_velocity, 0.0)
        else:
            self.current_velocity = 0.0
            self.publish_velocity(0.0, 0.0)
    
    def find_look_ahead_point(self):
        """Find the look-ahead point on the path."""
        
        if len(self.path.poses) == 0:
            return None, None
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Start from current index
        for i in range(self.current_index, len(self.path.poses)):
            pose = self.path.poses[i]
            target_x = pose.pose.position.x
            target_y = pose.pose.position.y
            
            distance = math.sqrt(
                (target_x - current_x)**2 + (target_y - current_y)**2
            )
            
            # Check if we're at the goal
            if i == len(self.path.poses) - 1 and distance < self.goal_tolerance:
                return None, None  # Goal reached
            
            # Find point at look-ahead distance
            if distance >= self.look_ahead_distance:
                self.current_index = i
                return (target_x, target_y), i
        
        # If no point found, use the last point
        last_pose = self.path.poses[-1]
        return (last_pose.pose.position.x, last_pose.pose.position.y), len(self.path.poses) - 1
    
    def calculate_curvature(self, index):
        """
        Calculate path curvature at given index.
        Returns value in radians (0 = straight, π = sharp turn)
        """
        
        if index < 1 or index >= len(self.path.poses) - 1:
            return 0.0
        
        # Get three consecutive points
        p1 = self.path.poses[index - 1].pose.position
        p2 = self.path.poses[index].pose.position
        p3 = self.path.poses[index + 1].pose.position
        
        # Calculate angles
        angle1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
        angle2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
        
        # Angle difference = curvature approximation
        angle_diff = abs(angle2 - angle1)
        
        # Normalize to [0, π]
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        
        return angle_diff
    
    def pure_pursuit_control(self, look_ahead_point, linear_vel):
        """Calculate angular velocity using Pure Pursuit algorithm."""
        
        # Get current robot pose
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Get current robot orientation (yaw)
        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate angle to look-ahead point
        target_x, target_y = look_ahead_point
        dx = target_x - current_x
        dy = target_y - current_y
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle error
        angle_error = target_angle - current_yaw
        
        # Normalize angle to [-π, π]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Calculate distance to look-ahead point
        distance = math.sqrt(dx**2 + dy**2)
        
        # Pure Pursuit formula: κ = 2*sin(α) / L
        if distance > 0:
            curvature = 2 * math.sin(angle_error) / distance
        else:
            curvature = 0
        
        # Calculate angular velocity
        angular_vel = curvature * linear_vel
        
        # Limit angular velocity
        angular_vel = np.clip(
            angular_vel,
            -self.max_angular_velocity,
            self.max_angular_velocity
        )
        
        return angular_vel
    
    def publish_velocity(self, linear, angular):
        """Publish velocity command to robot."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
    
    def log_trajectory_data(self, linear_vel, angular_vel, current_curv, upcoming_curv):
        """Log trajectory data to CSV file."""
        
        if self.current_pose is None:
            return
        
        # Initialize start time on first log
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # Get current time
        current_time = self.get_clock().now()
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        time_seconds = (current_time - self.start_time).nanoseconds / 1e9
        
        # Extract position
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Extract orientation (convert quaternion to yaw)
        orientation = self.current_pose.orientation
        theta = self.quaternion_to_yaw(orientation)
        
        # Calculate curvature (kappa = omega / v)
        if abs(linear_vel) > 0.01:
            curvature = abs(angular_vel) / abs(linear_vel)
        else:
            curvature = 0.0
        
        # Write to CSV
        self.csv_writer.writerow([
            timestamp,
            f'{time_seconds:.3f}',
            f'{x:.4f}',
            f'{y:.4f}',
            f'{theta:.4f}',
            f'{linear_vel:.4f}',
            f'{angular_vel:.4f}',
            f'{curvature:.4f}',
            f'{current_curv:.4f}',
            f'{upcoming_curv:.4f}'
        ])
        self.csv_file.flush()
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (theta)."""
        
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        if self.csv_file is not None:
            self.csv_file.close()
            self.get_logger().info(f'✓ CSV file closed: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
