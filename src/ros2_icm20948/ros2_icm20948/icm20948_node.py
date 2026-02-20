import math

import qwiic_icm20948
import rclpy
import sensor_msgs.msg
from rclpy.node import Node


class ICM20948Node(Node):
    def __init__(self):
        super().__init__("icm20948_node")

        # Logger
        self.logger = self.get_logger()

        # Parameters
        self.declare_parameter("i2c_address", 0x68)
        self.declare_parameter("frame_id", "imu_icm20948")
        self.declare_parameter("pub_rate", 50)

        i2c_addr = self.get_parameter("i2c_address").get_parameter_value().integer_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value

        self.i2c_addr = i2c_addr
        self.frame_id = frame_id
        self.pub_rate = pub_rate

        self.gyro_bias_x = -93.51
        self.gyro_bias_y = -27.23
        self.gyro_bias_z = -53.68

        self.ACCEL_SCALE = 16384.0
        self.GYRO_SCALE = 131.0
        self.DEG_TO_RAD = math.pi / 180.0

        # IMU instance
        self.imu = qwiic_icm20948.QwiicIcm20948(address=self.i2c_addr)
        if not self.imu.connected:
            self.logger.info(
                "The Qwiic ICM20948 device isn't connected to the system. Please check your connection."
            )
        self.imu.begin()
        self.imu.enableDlpfAccel(True)
        self.imu.enableDlpfGyro(True)

        # Publishers
        self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(
            sensor_msgs.msg.MagneticField, "/imu/mag_raw", 10
        )
        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def publish_cback(self):
        imu_msg = sensor_msgs.msg.Imu()
        mag_msg = sensor_msgs.msg.MagneticField()

        try:
            if self.imu.dataReady():
                self.imu.getAgmt()

                gx = self.imu.gxRaw - self.gyro_bias_x
                gy = self.imu.gyRaw - self.gyro_bias_y
                gz = self.imu.gzRaw - self.gyro_bias_z
                ax = self.imu.axRaw
                ay = self.imu.ayRaw
                az = self.imu.azRaw

                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_id

                imu_msg.linear_acceleration.x = ax * 9.81 / self.ACCEL_SCALE
                imu_msg.linear_acceleration.y = ay * 9.81 / self.ACCEL_SCALE
                imu_msg.linear_acceleration.z = az * 9.81 / self.ACCEL_SCALE

                imu_msg.angular_velocity.x = gx * self.DEG_TO_RAD / self.GYRO_SCALE
                imu_msg.angular_velocity.y = gy * self.DEG_TO_RAD / self.GYRO_SCALE
                imu_msg.angular_velocity.z = gz * self.DEG_TO_RAD / self.GYRO_SCALE
                imu_msg.orientation_covariance[0] = -1

                mag_msg.header.stamp = imu_msg.header.stamp
                mag_msg.header.frame_id = self.frame_id
                mag_msg.magnetic_field.x = self.imu.mxRaw * 1e-6 * 0.15
                mag_msg.magnetic_field.y = -self.imu.myRaw * 1e-6 * 0.15
                mag_msg.magnetic_field.z = -self.imu.mzRaw * 1e-6 * 0.15
        except OSError:
            self.logger.warn("Lost connection to ICM20948")

        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    icm20948_node = ICM20948Node()
    rclpy.spin(icm20948_node)

    icm20948_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
