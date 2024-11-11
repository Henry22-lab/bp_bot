import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32MultiArray
import math

class WheelTfNode(Node):
    def __init__(self):
        super().__init__('wheel_tf_node')

        # Initialize variables for encoder ticks
        self.left_ticks = 0
        self.right_ticks = 0

        # Create a TransformBroadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to the encoder ticks topic
        self.encoder_subscriber = self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)

        # Timer to publish the transforms
        self.create_timer(0.1, self.publish_transforms)

        self.get_logger().info("Wheel TF Node initialized and ready to receive encoder ticks.")

    def encoder_callback(self, msg):
        # Update left and right ticks from the encoder data
        self.left_ticks = msg.data[0]  # Assuming left ticks are the first element
        self.right_ticks = msg.data[1]  # Assuming right ticks are the second element

    def publish_transforms(self):
        # Define constants
        ticks_per_revolution = 20
        angle_per_tick = (2 * math.pi) / ticks_per_revolution  # Angle in radians per tick

        # Calculate rotation angles based on encoder ticks
        left_angle = self.left_ticks * angle_per_tick
        right_angle = self.right_ticks * angle_per_tick

        # Publish the left wheel transform
        left_transform = TransformStamped()
        left_transform.header.stamp = self.get_clock().now().to_msg()
        left_transform.header.frame_id = "base_link"
        left_transform.child_frame_id = "drivewhl_l_link"
        left_transform.transform.translation.x = 0.0  # Fixed position in x
        left_transform.transform.translation.y = 0.175  # Fixed position in y from URDF
        left_transform.transform.translation.z = 0.0
        
        # Set the rotation to rotate around the local y-axis for the left wheel
        left_transform.transform.rotation.x = 0.0
        left_transform.transform.rotation.y = math.sin(left_angle / 2.0)
        left_transform.transform.rotation.z = 0.0
        left_transform.transform.rotation.w = math.cos(left_angle / 2.0)

        # Publish the left wheel transform
        self.br.sendTransform(left_transform)

        # Publish the right wheel transform
        right_transform = TransformStamped()
        right_transform.header.stamp = self.get_clock().now().to_msg()
        right_transform.header.frame_id = "base_link"
        right_transform.child_frame_id = "drivewhl_r_link"
        right_transform.transform.translation.x = 0.0  # Fixed position in x
        right_transform.transform.translation.y = -0.175  # Fixed position in y from URDF
        right_transform.transform.translation.z = 0.0
        
        # Set the rotation to rotate around the local y-axis for the right wheel
        right_transform.transform.rotation.x = 0.0
        right_transform.transform.rotation.y = math.sin(right_angle / 2.0)
        right_transform.transform.rotation.z = 0.0
        right_transform.transform.rotation.w = math.cos(right_angle / 2.0)

        # Publish the right wheel transform
        self.br.sendTransform(right_transform)

        self.get_logger().info(f"Published left and right wheel rotations with angles: L={left_angle}, R={right_angle}")

def main(args=None):
    rclpy.init(args=args)
    wheel_tf_node = WheelTfNode()

    try:
        rclpy.spin(wheel_tf_node)
    except KeyboardInterrupt:
        wheel_tf_node.get_logger().info("Shutting down Wheel TF Node")
    finally:
        wheel_tf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

