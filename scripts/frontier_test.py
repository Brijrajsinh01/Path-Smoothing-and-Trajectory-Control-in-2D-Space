#!/usr/bin/env python3
"""
Optimized frontier explorer implementation
This is an improved version of the original contour_frontier_explorer.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
import tf2_ros
import math
import threading
import queue
from enum import Enum
import time

class ExplorationState(Enum):
    IDLE = 1
    EXPLORING = 2
    GOAL_REACHED = 3
    STUCK = 4
    COMPLETED = 5

class OptimizedFrontierExplorer(Node):
    def __init__(self):
        super().__init__('optimized_frontier_explorer')
        
        # State management
        self.state = ExplorationState.IDLE
        self.current_goal = None
        self.goal_start_time = None
        self.exploration_history = []
        self.stuck_count = 0
        
        # Parameters (should be configurable via ROS parameters)
        self.min_frontier_size = 5
        self.goal_similarity_threshold = 0.5
        self.max_stuck_attempts = 3
        self.exploration_coverage_threshold = 0.95
        
        # Optimized processing
        self.map_queue = queue.Queue(maxsize=1)
        self.processing_thread = threading.Thread(target=self.background_processor, daemon=True)
        self.processing_thread.start()
        
        # Communication
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/exploration_goal', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/exploration_markers', 10)
        self.done_subscription = self.create_subscription(
            Bool, '/goal_done', self.done_callback, 10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Optimized Frontier Explorer initialized')
    
    def map_callback(self, msg):
        """Non-blocking map callback"""
        # Only keep latest map to avoid processing backlog
        try:
            self.map_queue.put_nowait(msg)
        except queue.Full:
            # Remove old map and add new one
            try:
                self.map_queue.get_nowait()
                self.map_queue.put_nowait(msg)
            except queue.Empty:
                pass
    
    def background_processor(self):
        """Background thread for heavy computation"""
        while rclpy.ok():
            try:
                msg = self.map_queue.get(timeout=1.0)
                self.process_map_async(msg)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in background processor: {e}")
    
    def process_map_async(self, map_msg):
        """Asynchronous map processing with optimized algorithms"""
        if self.state not in [ExplorationState.IDLE, ExplorationState.GOAL_REACHED]:
            return
        
        # Check if exploration is complete
        if self.is_exploration_complete(map_msg):
            self.state = ExplorationState.COMPLETED
            self.get_logger().info("Exploration completed!")
            return
        
        # Optimized frontier detection
        frontiers = self.detect_frontiers_optimized(map_msg)
        
        if not frontiers:
            self.get_logger().info("No frontiers found")
            return
        
        # Enhanced goal selection
        best_goal = self.select_best_goal_enhanced(frontiers, map_msg)
        
        if best_goal and self.should_publish_goal(best_goal):
            self.publish_goal_safe(best_goal)
    
    def detect_frontiers_optimized(self, map_msg):
        """Optimized frontier detection using morphological operations"""
        width = map_msg.info.width
        height = map_msg.info.height
        grid = np.array(map_msg.data).reshape((height, width))
        
        # Create binary masks
        free_space = (grid == 0).astype(np.uint8)
        unknown_space = (grid == -1).astype(np.uint8)
        
        # Use morphological operations (much faster than pixel iteration)
        kernel = np.ones((3, 3), np.uint8)
        dilated_free = cv2.dilate(free_space, kernel, iterations=1)
        
        # Frontier is where dilated free space meets unknown space
        frontier_mask = cv2.bitwise_and(dilated_free, unknown_space) * 255
        
        # Remove small noise
        frontier_mask = cv2.morphologyEx(frontier_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(
            frontier_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter by minimum size
        valid_contours = [c for c in contours if len(c) >= self.min_frontier_size]
        
        return valid_contours
    
    def select_best_goal_enhanced(self, contours, map_msg):
        """Enhanced goal selection with multi-criteria scoring"""
        robot_x, robot_y = self.get_robot_position()
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        
        grid = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        
        best_score = -float('inf')
        best_goal = None
        
        for contour in contours:
            # Use centroid instead of midpoint for stability
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Convert to map coordinates
            map_x = origin.x + cx * resolution
            map_y = origin.y + cy * resolution
            
            # Multi-criteria scoring
            score = self.calculate_goal_score(
                (map_x, map_y), (robot_x, robot_y), contour, grid, resolution)
            
            if score > best_score:
                best_score = score
                best_goal = self.create_goal_message(map_x, map_y)
        
        return best_goal
    
    def calculate_goal_score(self, goal_pos, robot_pos, contour, grid, resolution):
        """Multi-criteria goal scoring"""
        map_x, map_y = goal_pos
        robot_x, robot_y = robot_pos
        
        # 1. Distance factor (closer is generally better)
        distance = math.hypot(map_x - robot_x, map_y - robot_y)
        distance_score = 1.0 / (distance + 0.1)
        
        # 2. Information gain estimation
        pixel_x = int((map_x - grid.shape[1] * resolution / 2) / resolution)
        pixel_y = int((map_y - grid.shape[0] * resolution / 2) / resolution)
        info_gain = self.estimate_information_gain(grid, pixel_x, pixel_y)
        
        # 3. Path feasibility (simple obstacle check)
        path_score = self.estimate_path_feasibility(robot_pos, goal_pos, grid, resolution)
        
        # 4. Exploration novelty
        novelty_score = self.calculate_novelty(goal_pos)
        
        # 5. Frontier size
        size_score = len(contour) / 100.0  # Normalize
        
        # Weighted combination
        total_score = (distance_score * 0.2 + 
                       info_gain * 0.3 + 
                       path_score * 0.25 + 
                       novelty_score * 0.15 + 
                       size_score * 0.1)
        
        return total_score
    
    def estimate_information_gain(self, grid, x, y, radius=20):
        """Estimate information gain around a point"""
        h, w = grid.shape
        x_min, x_max = max(0, x - radius), min(w, x + radius)
        y_min, y_max = max(0, y - radius), min(h, y + radius)
        
        if x_min >= x_max or y_min >= y_max:
            return 0.0
        
        region = grid[y_min:y_max, x_min:x_max]
        unknown_count = np.sum(region == -1)
        total_count = region.size
        
        return unknown_count / total_count if total_count > 0 else 0.0
    
    def estimate_path_feasibility(self, start, goal, grid, resolution):
        """Simple path feasibility check"""
        # This is a simplified version - in practice, use actual path planning
        distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
        
        # Check for obvious obstacles (simplified line check)
        # In a real implementation, you'd use A* or RRT
        return 1.0 / (distance + 0.1)  # Simplified scoring
    
    def calculate_novelty(self, goal_pos):
        """Calculate novelty score based on exploration history"""
        if not self.exploration_history:
            return 1.0
        
        min_distance = float('inf')
        for hist_pos in self.exploration_history:
            dist = math.hypot(goal_pos[0] - hist_pos[0], goal_pos[1] - hist_pos[1])
            min_distance = min(min_distance, dist)
        
        # Higher score for positions farther from previously explored areas
        return min(1.0, min_distance / 2.0)  # Normalize to [0, 1]
    
    def should_publish_goal(self, new_goal):
        """Intelligent goal publishing decision"""
        if self.current_goal is None:
            return True
        
        # Check if new goal is significantly different
        distance = math.hypot(
            new_goal.pose.position.x - self.current_goal.pose.position.x,
            new_goal.pose.position.y - self.current_goal.pose.position.y
        )
        
        return distance > self.goal_similarity_threshold
    
    def publish_goal_safe(self, goal):
        """Safely publish goal with state management"""
        self.current_goal = goal
        self.goal_start_time = time.time()
        self.state = ExplorationState.EXPLORING
        
        self.goal_publisher.publish(goal)
        self.get_logger().info(
            f"Published goal at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        
        # Publish visualization marker
        self.publish_goal_marker(goal)
    
    def create_goal_message(self, x, y):
        """Create a properly formatted goal message"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal
    
    def done_callback(self, msg):
        """Handle goal completion"""
        if msg.data and self.state == ExplorationState.EXPLORING:
            self.handle_goal_completion()
    
    def handle_goal_completion(self):
        """Handle successful goal completion"""
        self.state = ExplorationState.GOAL_REACHED
        if self.current_goal:
            goal_pos = (self.current_goal.pose.position.x, self.current_goal.pose.position.y)
            self.exploration_history.append(goal_pos)
            
            # Limit history size
            if len(self.exploration_history) > 50:
                self.exploration_history.pop(0)
        
        self.current_goal = None
        self.stuck_count = 0
        self.get_logger().info("Goal completed successfully")
    
    def is_exploration_complete(self, map_msg):
        """Detect when exploration is finished"""
        grid = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        
        total_cells = grid.size
        known_cells = np.sum((grid == 0) | (grid == 100))
        coverage_ratio = known_cells / total_cells
        
        # Check coverage and remaining significant frontiers
        frontiers = self.detect_frontiers_optimized(map_msg)
        significant_frontiers = len([f for f in frontiers if len(f) > 10])
        
        is_complete = (coverage_ratio > self.exploration_coverage_threshold and 
                      significant_frontiers < 3)
        
        if is_complete:
            self.get_logger().info(
                f"Exploration complete: {coverage_ratio:.1%} coverage, "
                f"{significant_frontiers} significant frontiers remaining")
        
        return is_complete
    
    def publish_goal_marker(self, goal):
        """Publish visualization marker for the goal"""
        marker_array = MarkerArray()
        
        # Goal marker
        marker = Marker()
        marker.header = goal.header
        marker.ns = "exploration_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime.sec = 10
        
        marker_array.markers.append(marker)
        
        # History markers
        for i, hist_pos in enumerate(self.exploration_history[-10:]):  # Last 10 positions
            hist_marker = Marker()
            hist_marker.header.frame_id = 'map'
            hist_marker.header.stamp = self.get_clock().now().to_msg()
            hist_marker.ns = "exploration_history"
            hist_marker.id = i + 1
            hist_marker.type = Marker.CYLINDER
            hist_marker.action = Marker.ADD
            hist_marker.pose.position.x = hist_pos[0]
            hist_marker.pose.position.y = hist_pos[1]
            hist_marker.pose.position.z = 0.0
            hist_marker.pose.orientation.w = 1.0
            hist_marker.scale.x = 0.2
            hist_marker.scale.y = 0.2
            hist_marker.scale.z = 0.1
            hist_marker.color.a = 0.5
            hist_marker.color.r = 1.0
            hist_marker.color.g = 0.0
            hist_marker.color.b = 0.0
            hist_marker.lifetime.sec = 30
            
            marker_array.markers.append(hist_marker)
        
        self.marker_publisher.publish(marker_array)
    
    def get_robot_position(self):
        """Get the robot's current position using TF"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}. Using default (0,0)")
            return 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedFrontierExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

