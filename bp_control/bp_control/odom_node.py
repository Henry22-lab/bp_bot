import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Initialize variables for odometry calculations
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Previous encoder tick values for left and right wheels
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.first_reading = True  # Flag to ignore first reading

        # Robot parameters
        self.wheel_radius = 0.035  # meters
        self.wheel_base = 0.15  # meters (distance between wheels)
        self.ticks_per_revolution = 20  # Update with your encoder's ticks per revolution

        # Subscriber for encoder ticks
        self.encoder_subscriber = self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)

        # Publisher for odometry data
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Initialize the tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odom Node initialized and ready to receive encoder ticks.")

    def encoder_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning("Not enough data received in encoder ticks")
            return

        left_ticks = int(msg.data[0])
        right_ticks = int(msg.data[1])

        # Skip the first reading to initialize previous tick values
        if self.first_reading:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.first_reading = False
            return

        # Calculate the change in ticks since the last callback
        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks

        # Update previous tick values
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Calculate the time delta in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # Convert ticks to distance
        distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution
        left_distance = delta_left_ticks * distance_per_tick
        right_distance = delta_right_ticks * distance_per_tick

        # Calculate the linear and angular displacement
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # Update position and orientation
        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # Logging for debugging
        self.get_logger().info(f"Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        self.get_logger().info(f"Delta left: {delta_left_ticks}, Delta right: {delta_right_ticks}")
        self.get_logger().info(f"Left distance: {left_distance:.4f}, Right distance: {right_distance:.4f}")
        self.get_logger().info(f"x: {self.x:.4f}, y: {self.y:.4f}, theta: {self.theta:.4f}")

        # Publish odometry data
        self.publish_odometry()

    def publish_odometry(self):
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # Update position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Update velocity (linear and angular)
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = self.x / self.last_time if self.last_time > 0 else 0.0
        odom_msg.twist.twist.angular.z = self.theta / self.last_time if self.last_time > 0 else 0.0

        self.odom_publisher.publish(odom_msg)

        # Broadcast the transform between odom and base_link
        self.broadcast_transform()

    def broadcast_transform(self):
        # Create a TransformStamped message
        transform = TransformStamped()

        # Set timestamp and frame IDs
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        # Set translation (x, y, z)
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0  # Assuming 2D

        # Set rotation (quaternion from Euler angles)
        quat = quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomNode()

    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        odom_node.get_logger().info("Shutting down Odom Node")
    finally:
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

