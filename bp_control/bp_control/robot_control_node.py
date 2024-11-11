import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
import serial
import serial.tools.list_ports
import time

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Detect the serial port automatically
        self.serial_port = self.detect_arduino_port()
        if not self.serial_port:
            self.get_logger().error("Arduino not detected. Please check the connection.")
            return
        
        self.get_logger().info(f"Arduino detected on port: {self.serial_port.port}")
        
        # Initialize subscription to /cmd_vel and publisher for /encoder_ticks and /imu
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.encoder_publisher = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        self.get_logger().info("Robot Control Node initialized with Encoder Publisher and IMU Publisher")

        # Timer to read encoder and IMU data and publish
        self.create_timer(0.1, self.timer_callback)  # Set timer to call the callback every 0.1 seconds

    def detect_arduino_port(self):
        """Detects the serial port to which the Arduino is connected."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'Arduino' in port.description or 'wchusbserial' in port.description:
                return serial.Serial(port.device, 9600, timeout=1)
        return None

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        self.get_logger().info(f"Received cmd_vel: linear={linear_velocity}, angular={angular_velocity}")
        
        # Send velocities to Arduino over serial
        command = f"{linear_velocity},{angular_velocity}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent to Arduino: {command}")

    def read_sensor_data(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            try:
                # Parse the encoder and IMU data from Arduino
                data = line.split(',')
                if len(data) == 8:  # Ensure there are 8 values: left_ticks, right_ticks, ax, ay, az, gx, gy, gz
                    left_ticks = int(data[0])
                    right_ticks = int(data[1])
                    ax = float(data[2])
                    ay = float(data[3])
                    az = float(data[4])
                    gx = float(data[5])
                    gy = float(data[6])
                    gz = float(data[7])
                    
                    # Publish encoder ticks
                    encoder_msg = Int32MultiArray()
                    encoder_msg.data = [left_ticks, right_ticks]
                    self.encoder_publisher.publish(encoder_msg)
                    
                    # Publish IMU data
                    imu_msg = Imu()
                    imu_msg.linear_acceleration.x = ax
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az
                    imu_msg.angular_velocity.x = gx
                    imu_msg.angular_velocity.y = gy
                    imu_msg.angular_velocity.z = gz
                    self.imu_publisher.publish(imu_msg)

            except ValueError:
                self.get_logger().warn(f"Invalid data received: {line}")
    
    def timer_callback(self):
        self.read_sensor_data()

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    rclpy.spin(robot_control_node)
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

