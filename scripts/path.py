#!/usr/bin/env python3
"""
Path Smoother Node
Smooths discrete waypoints into a continuous path using cubic spline interpolation.
Starts from robot's current position and visits all waypoints in order.
Includes tangent-based orientation at each path point.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
from scipy.interpolate import CubicSpline
import math
import csv
from datetime import datetime
import os


class PathSmoother(Node):
    """Smooth waypoints using cubic spline interpolation."""
    
    def __init__(self):
        super().__init__('path_smoother')
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseArray,
            '/waypoints',
            self.waypoint_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/smooth_path', 10)
        
        # Parameters
        self.declare_parameter('num_samples', 500)
        self.declare_parameter('include_robot_position', True)
        self.declare_parameter('auto_publish', True)
        self.declare_parameter('calculate_orientations', True)  # NEW!
        
        self.num_samples = self.get_parameter('num_samples').value
        self.include_robot_position = self.get_parameter('include_robot_position').value
        self.auto_publish = self.get_parameter('auto_publish').value
        self.calculate_orientations = self.get_parameter('calculate_orientations').value
        
        # State variables
        self.robot_position = None
        self.latest_waypoints = None
        
        # CSV Logging setup
        # Save to task_ws/src/my_bot/config directory
        config_dir = os.path.expanduser('~/task_ws/src/my_bot/config')
        os.makedirs(config_dir, exist_ok=True)
        csv_path = os.path.join(config_dir, 'waypoints_and_paths.csv')
        
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'type', 'pattern', 'sequence_id', 'point_index',
            'x', 'y', 'theta', 'total_points'
        ])
        self.csv_file.flush()
        self.waypoint_count = 0
        self.path_count = 0
        self.current_pattern = 'unknown'
        self.csv_path = csv_path
        
        self.get_logger().info('Path Smoother started')
        self.get_logger().info(f'Smoothing with {self.num_samples} samples')
        self.get_logger().info(f'Include robot position: {self.include_robot_position}')
        self.get_logger().info(f'Calculate orientations: {self.calculate_orientations}')
        self.get_logger().info(f'Algorithm: Cubic Spline Interpolation')
        self.get_logger().info(f'📊 Logging to: {self.csv_path}')
    
    def odom_callback(self, msg):
        """Update robot's current position."""
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def waypoint_callback(self, msg):
        """Process waypoints and generate smooth path."""
        
        if len(msg.poses) < 2:
            self.get_logger().warn('Need at least 2 waypoints to smooth')
            return
        
        # Store waypoints
        self.latest_waypoints = msg
        
        # Log waypoints to CSV
        self.log_waypoints(msg)
        
        # Auto publish if enabled
        if self.auto_publish:
            self.generate_and_publish_path(msg)
    
    def generate_and_publish_path(self, waypoint_msg):
        """Generate smooth path from robot position through all waypoints."""
        
        # Extract waypoint coordinates
        waypoints_x = [pose.position.x for pose in waypoint_msg.poses]
        waypoints_y = [pose.position.y for pose in waypoint_msg.poses]
        
        # Prepend robot's current position if available
        if self.include_robot_position and self.robot_position is not None:
            robot_x, robot_y = self.robot_position
            
            # Check if robot is already at first waypoint
            first_wp_x, first_wp_y = waypoints_x[0], waypoints_y[0]
            distance_to_first = np.sqrt(
                (robot_x - first_wp_x)**2 + (robot_y - first_wp_y)**2
            )
            
            # Only add robot position if it's not too close to first waypoint
            if distance_to_first > 0.1:  # 10cm threshold
                waypoints_x.insert(0, robot_x)
                waypoints_y.insert(0, robot_y)
                self.get_logger().info(
                    f'✓ Added robot position ({robot_x:.2f}, {robot_y:.2f}) as starting point'
                )
            else:
                self.get_logger().info('Robot already at first waypoint, skipping prepend')
        elif self.include_robot_position and self.robot_position is None:
            self.get_logger().warn('Robot position not available yet, using waypoints only')
        
        self.get_logger().info(
            f'Received {len(waypoint_msg.poses)} waypoints, '
            f'total path points: {len(waypoints_x)}'
        )
        
        # Generate smooth path
        smooth_x, smooth_y = self.smooth_path(waypoints_x, waypoints_y)
        
        # Publish smooth path
        path_msg = self.publish_smooth_path(smooth_x, smooth_y, waypoint_msg.header.frame_id)
        
        # Log smooth path to CSV
        if path_msg is not None:
            self.log_smooth_path(path_msg)
        
        self.get_logger().info(
            f'✓ Smoothed {len(waypoints_x)} points into {len(smooth_x)} points'
        )
        self.get_logger().info(
            f'Path: Robot → WP1 → WP2 → ... → WP{len(waypoint_msg.poses)}'
        )
    
    def smooth_path(self, waypoints_x, waypoints_y):
        """
        Apply cubic spline interpolation to waypoints.
        
        Algorithm:
        1. Calculate arc-length parameterization (distance along path)
        2. Create cubic splines for x(s) and y(s)
        3. Sample splines uniformly to get smooth path
        
        This ensures smooth velocity and acceleration profiles.
        """
        
        # Step 1: Calculate arc-length parameter (distance along path)
        distances = [0.0]
        for i in range(1, len(waypoints_x)):
            dx = waypoints_x[i] - waypoints_x[i-1]
            dy = waypoints_y[i] - waypoints_y[i-1]
            dist = np.sqrt(dx**2 + dy**2)
            distances.append(distances[-1] + dist)
        
        # Handle case where all waypoints are the same
        if distances[-1] == 0:
            self.get_logger().warn('All waypoints are at the same location')
            return waypoints_x, waypoints_y
        
        # Step 2: Create cubic splines
        # bc_type='natural' means second derivative is zero at endpoints
        # This gives the smoothest possible curve
        cs_x = CubicSpline(distances, waypoints_x, bc_type='natural')
        cs_y = CubicSpline(distances, waypoints_y, bc_type='natural')
        
        # Step 3: Sample the splines uniformly
        s = np.linspace(0, distances[-1], self.num_samples)
        smooth_x = cs_x(s)
        smooth_y = cs_y(s)
        
        return smooth_x.tolist(), smooth_y.tolist()
    
    def publish_smooth_path(self, smooth_x, smooth_y, frame_id):
        """
        Publish smooth path as nav_msgs/Path with tangent-based orientations.
        
        Orientation Calculation:
        - For each point, calculate the tangent direction (heading)
        - Tangent = direction from current point to next point
        - Convert to quaternion for ROS message
        """
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        
        for i in range(len(smooth_x)):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = frame_id
            
            # Set position
            pose.pose.position.x = smooth_x[i]
            pose.pose.position.y = smooth_y[i]
            pose.pose.position.z = 0.0
            
            # Calculate orientation from path tangent
            if self.calculate_orientations:
                if i < len(smooth_x) - 1:
                    # Calculate tangent direction to next point
                    dx = smooth_x[i+1] - smooth_x[i]
                    dy = smooth_y[i+1] - smooth_y[i]
                    yaw = math.atan2(dy, dx)
                else:
                    # Last point: use direction from previous point
                    if i > 0:
                        dx = smooth_x[i] - smooth_x[i-1]
                        dy = smooth_y[i] - smooth_y[i-1]
                        yaw = math.atan2(dy, dx)
                    else:
                        yaw = 0.0
                
                # Convert yaw to quaternion (rotation around Z-axis)
                # Quaternion formula: q = [0, 0, sin(yaw/2), cos(yaw/2)]
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Default orientation (facing forward)
                pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        
        if self.calculate_orientations:
            self.get_logger().info('✓ Published smooth path with tangent orientations to /smooth_path')
        else:
            self.get_logger().info('✓ Published smooth path to /smooth_path')
        
        return path_msg
    
    def log_waypoints(self, waypoint_msg):
        """Log waypoints to CSV file."""
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # Detect pattern from waypoint count
        pattern = self.detect_pattern(len(waypoint_msg.poses))
        
        # Log each waypoint
        for i, pose in enumerate(waypoint_msg.poses):
            x = pose.position.x
            y = pose.position.y
            theta = self.quaternion_to_yaw(pose.orientation)
            
            self.csv_writer.writerow([
                timestamp,
                'waypoint',
                pattern,
                self.waypoint_count,
                i,
                f'{x:.4f}',
                f'{y:.4f}',
                f'{theta:.4f}',
                len(waypoint_msg.poses)
            ])
        
        self.csv_file.flush()
        self.waypoint_count += 1
        
        self.get_logger().info(
            f'✓ Logged {len(waypoint_msg.poses)} waypoints (pattern: {pattern})',
            throttle_duration_sec=2.0
        )
    
    def log_smooth_path(self, path_msg):
        """Log smooth path to CSV file."""
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # Log each path point
        for i, pose_stamped in enumerate(path_msg.poses):
            pose = pose_stamped.pose
            x = pose.position.x
            y = pose.position.y
            theta = self.quaternion_to_yaw(pose.orientation)
            
            self.csv_writer.writerow([
                timestamp,
                'smooth_path',
                self.current_pattern,
                self.path_count,
                i,
                f'{x:.4f}',
                f'{y:.4f}',
                f'{theta:.4f}',
                len(path_msg.poses)
            ])
        
        self.csv_file.flush()
        self.path_count += 1
        
        self.get_logger().info(
            f'✓ Logged {len(path_msg.poses)} smooth path points',
            throttle_duration_sec=2.0
        )
    
    def detect_pattern(self, num_waypoints):
        """Detect pattern type from waypoint count."""
        
        # Simple heuristic based on waypoint count
        if num_waypoints > 30 and num_waypoints < 50:
            self.current_pattern = 'circle'
            return 'circle'
        elif num_waypoints >= 4 and num_waypoints <= 12:
            self.current_pattern = 'square'
            return 'square'
        elif num_waypoints > 10 and num_waypoints < 25:
            self.current_pattern = 'zigzag'
            return 'zigzag'
        elif num_waypoints > 50 and num_waypoints < 80:
            self.current_pattern = 'figure_eight'
            return 'figure_eight'
        elif num_waypoints > 80:
            self.current_pattern = 'spiral'
            return 'spiral'
        
        return self.current_pattern
    
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
    node = PathSmoother()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
