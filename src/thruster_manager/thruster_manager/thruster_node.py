#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio


class EscDriver:
    def __init__(self, pins):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon! Run 'sudo pigpiod'")
        self.pins = pins
        self.num_motors = len(pins)
        for pin in self.pins:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(pin, 0)

    def arm(self):
        print("Arming ESCs...")
        self.set_all_pwm(1500)
        time.sleep(2.0)
        print("ESCs Armed.")

    def set_pwm(self, index, pwm_value):
        if 0 <= index < self.num_motors:
            self.pi.set_servo_pulsewidth(self.pins[index], pwm_value)

    def set_all_pwm(self, pwm_value):
        for i in range(self.num_motors):
            self.set_pwm(i, pwm_value)

    def stop(self):
        print("Stopping!")
        self.set_all_pwm(0)
        self.pi.stop()


class ThrusterManager(Node):
    def __init__(self):
        super().__init__("thruster_manager")

        # --- CONFIGURATION ---
        self.motor_pins = [26, 19, 13, 6, 5]

        # DIRECTION: 1.0 or -1.0
        self.motor_dirs = [1.0, 1.0, -1.0, 1.0, -1.0]

        # SCALING: Tune these to balance your motors!
        # Example: If motor 0 is weak, set to 1.05. If motor 1 is too strong, set to 0.9.
        self.motor_scales = [1.0, 1.0, 1.0, 1.0, 1.0]

        # --- SETUP HARDWARE ---
        try:
            self.driver = EscDriver(self.motor_pins)
            self.driver.arm()
        except Exception as e:
            self.get_logger().error(f"Hardware Error: {e}")
            exit(1)

        self.subscription = self.create_subscription(
            Twist, "cmd_vel_out", self.listener_callback, 10
        )

        # Safety Watchdog
        self.last_msg_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_callback)
        self.is_timed_out = False

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 1.0 and not self.is_timed_out:
            self.get_logger().warn("Watchdog Timeout! Stopping Motors.")
            self.driver.set_all_pwm(1500)
            self.is_timed_out = True

    def calculate_pwm(self, force, scale):
        """
        1. Applies Scale (Hardware Trim)
        2. Linearizes (Sqrt curve)
        3. Maps to Full Range (1000-2000)
        """
        # A. Apply individual motor scaling (Trim)
        scaled_force = force * scale

        # B. Deadzone (small ignore window)
        if abs(scaled_force) < 0.02:
            return 1500

        # C. Linearization (Square Root Curve)
        sign = 1 if scaled_force > 0 else -1
        # We take abs(), sqrt it, then re-apply sign.
        # This gives higher PWM resolution at low speeds.
        linear_force = math.sqrt(abs(scaled_force)) * sign

        # D. Map to Full Range (1500 +/- 500)
        # Result is 1000 to 2000
        pwm = 1500 + (linear_force * 500)

        # E. Hard Clamp (Safety for ESCs)
        # Even if scale makes it 2050, clamp it to 2000
        return int(max(min(pwm, 2000), 1000))

    def listener_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.is_timed_out = False

        surge = msg.linear.x
        heave = msg.linear.z
        yaw = msg.angular.z
        pitch = msg.angular.y
        roll = msg.angular.x

        # --- MIXER ---
        vb = heave - pitch  # Vert Back
        vfl = heave + pitch + roll  # Vert Front Left
        vfr = heave + pitch - roll  # Vert Front Right
        hl = surge + yaw  # Horiz Left
        hr = surge - yaw  # Horiz Right

        raw_forces = [hl, vfl, vb, hr, vfr]

        # Process and Write
        for i, force in enumerate(raw_forces):
            # Apply Direction
            force = force * self.motor_dirs[i]

            # Calculate PWM with Scaling
            pwm = self.calculate_pwm(force, self.motor_scales[i])

            self.driver.set_pwm(i, pwm)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterManager()
    rclpy.spin(node)
    node.driver.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
