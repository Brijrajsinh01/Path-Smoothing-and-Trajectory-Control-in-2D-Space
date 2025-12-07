#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CoordinateTransformer(Node):

    def __init__(self):
        super().__init__('coordinate_transformer')

        # Create a tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the transformed coordinates
        self.coordinates_publisher = self.create_publisher(PoseStamped, 'base_link_coordinates', 10)

        # Set a timer to repeatedly call the transform function
        self.timer = self.create_timer(0.20, self.transform_coordinates)

    def transform_coordinates(self):
        # try:
            # Look up the transform from odom to base_link
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            # Create a PoseStamped message to hold the transformed position and orientation
            pose_in_base_link = PoseStamped()
            pose_in_base_link.header.stamp = self.get_clock().now().to_msg()
            pose_in_base_link.header.frame_id = 'base_link'  # This is the target frame

            # Extract position (x, y, z) from the transform
            pose_in_base_link.pose.position.x = transform.transform.translation.x
            pose_in_base_link.pose.position.y = transform.transform.translation.y
            pose_in_base_link.pose.position.z = transform.transform.translation.z

            # Extract orientation (quaternion) from the transform
            pose_in_base_link.pose.orientation.x = transform.transform.rotation.x
            pose_in_base_link.pose.orientation.y = transform.transform.rotation.y
            pose_in_base_link.pose.orientation.z = transform.transform.rotation.z
            pose_in_base_link.pose.orientation.w = transform.transform.rotation.w

            # Publish the transformed pose (position and orientation)
            self.coordinates_publisher.publish(pose_in_base_link)

        # except (LookupException, ConnectivityException, ExtrapolationException) as e:
        #     self.get_logger().error(f'Could not transform coordinates: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
