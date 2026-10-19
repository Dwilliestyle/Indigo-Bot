#!/usr/bin/env python3
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68


class MPU_Driver(Node):

    def __init__(self):
        super().__init__("mpu_driver")

        # Parameters
        self.frame_id_ = self.declare_parameter("frame_id", "imu_link").value
        self.calib_samples_target_ = self.declare_parameter("calibration_samples", 200).value
        self.warmup_samples_ = self.declare_parameter("warmup_samples", 50).value
        # DLPF_CFG: 3 = ~44 Hz, 5 = ~10 Hz, 6 = ~5 Hz
        self.dlpf_cfg_ = self.declare_parameter("dlpf_cfg", 5).value

        # Calibration state
        self.calibrated_ = False
        self.warmup_count_ = 0
        self.calib_count_ = 0
        self.sum_ = [0.0, 0.0, 0.0]
        self.sumsq_ = [0.0, 0.0, 0.0]
        self.bias_ = [0.0, 0.0, 0.0]

        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = self.frame_id_
        # This driver doesn't compute orientation; -1 tells consumers to ignore it
        self.imu_msg_.orientation_covariance[0] = -1.0
        # Accelerometer isn't calibrated; -1 tells consumers not to trust it for fusion
        self.imu_msg_.linear_acceleration_covariance[0] = -1.0

        self.get_logger().info(
            f"Calibrating gyro bias over {self.calib_samples_target_} samples, "
            "keep the robot still...")

        self.timer_ = self.create_timer(0.01, self.timerCallback)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
                if not self.is_connected_:
                    return

            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
        except OSError:
            self.is_connected_ = False
            return

        # Full scale range +/- 250 deg/s, sensitivity 131 LSB/(deg/s)
        g = [gyro_x / 7509.55, gyro_y / 7509.55, gyro_z / 7509.55]

        if not self.calibrated_:
            # Let the sensor settle after wake-up before sampling
            if self.warmup_count_ < self.warmup_samples_:
                self.warmup_count_ += 1
                return

            for i in range(3):
                self.sum_[i] += g[i]
                self.sumsq_[i] += g[i] * g[i]
            self.calib_count_ += 1

            if self.calib_count_ >= self.calib_samples_target_:
                n = float(self.calib_count_)
                var = [0.0, 0.0, 0.0]
                for i in range(3):
                    self.bias_[i] = self.sum_[i] / n
                    # Variance = E[x^2] - (E[x])^2, floored so it's never zero
                    var[i] = max(self.sumsq_[i] / n - self.bias_[i] ** 2, 1e-6)

                # Diagonal of the 3x3 row-major covariance matrix: indices 0, 4, 8
                self.imu_msg_.angular_velocity_covariance[0] = var[0]
                self.imu_msg_.angular_velocity_covariance[4] = var[1]
                self.imu_msg_.angular_velocity_covariance[8] = var[2]

                self.calibrated_ = True
                self.get_logger().info(
                    f"Gyro calibrated. Bias [rad/s]: x={self.bias_[0]:.5f} "
                    f"y={self.bias_[1]:.5f} z={self.bias_[2]:.5f} | "
                    f"variance: {var[0]:.2e} {var[1]:.2e} {var[2]:.2e}")
            return  # don't publish until calibrated

        self.imu_msg_.linear_acceleration.x = acc_x / 1670.13
        self.imu_msg_.linear_acceleration.y = acc_y / 1670.13
        self.imu_msg_.linear_acceleration.z = acc_z / 1670.13
        self.imu_msg_.angular_velocity.x = g[0] - self.bias_[0]
        self.imu_msg_.angular_velocity.y = g[1] - self.bias_[1]
        self.imu_msg_.angular_velocity.z = g[2] - self.bias_[2]

        self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub_.publish(self.imu_msg_)

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, self.dlpf_cfg_)
            # GYRO_CONFIG = 0 -> FS_SEL=0 -> +/-250 deg/s (matches the 7509.55 divisor above)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False

    def read_raw_data(self, addr):
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr + 1)
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value


def main():
    rclpy.init()
    mpu_driver = MPU_Driver()
    rclpy.spin(mpu_driver)
    mpu_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
