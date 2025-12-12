import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import numpy as np


class MagCalibrator(Node):
    def __init__(self):
        super().__init__("mag_corrector")  # Node name matches YAML key

        # 1. Declare Parameters (with defaults just in case)
        self.declare_parameter("hard_iron_offset", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "soft_iron_matrix", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )

        # 2. Load Parameters
        offset_list = (
            self.get_parameter("hard_iron_offset")
            .get_parameter_value()
            .double_array_value
        )
        matrix_list = (
            self.get_parameter("soft_iron_matrix")
            .get_parameter_value()
            .double_array_value
        )

        # 3. Convert to NumPy
        self.offset = np.array(offset_list) * 1e-6

        # Reshape the list of 9 floats back into a 3x3 matrix
        if len(matrix_list) == 9:
            self.matrix = np.array(matrix_list).reshape(3, 3)
        else:
            self.get_logger().error(
                "Soft Iron Matrix must have 9 elements! Using Identity."
            )
            self.matrix = np.eye(3)

        self.get_logger().info(f"Loaded Offset: {self.offset}")
        self.get_logger().info(f"Loaded Matrix:\n{self.matrix}")

        # Sub/Pub
        self.sub = self.create_subscription(
            MagneticField, "/imu/mag_raw", self.callback, 10
        )
        self.pub = self.create_publisher(MagneticField, "/imu/mag", 10)

    def callback(self, msg):
        raw = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ])

        # Apply Calibration
        corrected = self.matrix.dot(raw - self.offset)

        out_msg = MagneticField()
        out_msg.header = msg.header
        out_msg.magnetic_field.x = corrected[0]
        out_msg.magnetic_field.y = corrected[1]
        out_msg.magnetic_field.z = corrected[2]
        out_msg.magnetic_field_covariance = msg.magnetic_field_covariance

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MagCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
