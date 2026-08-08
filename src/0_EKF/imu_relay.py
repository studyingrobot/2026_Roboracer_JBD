#!/usr/bin/env python3
"""VESC raw IMU -> robot_localization 이 먹을 수 있는 sensor_msgs/Imu 로 변환.

local_ekf.yaml 은 RealSense IMU(/camera/camera/imu) 기준으로 쓰여 있어서
EKF 는 rad/s, m/s^2, 유효한 frame_id, 0 이 아닌 covariance 를 기대한다.
반면 /sensors/imu/raw 는 deg/s, g, frame_id='', covariance 전부 0 이다.
그 간극을 여기서 메운다. EKF config 는 손대지 않는다.

  ros2 run/python3 imu_relay.py --ros-args \
      -p in_topic:=/sensors/imu/raw -p out_topic:=/camera/camera/imu
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

DEG2RAD = math.pi / 180.0
G = 9.80665


class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')

        self.declare_parameter('in_topic', '/sensors/imu/raw')
        self.declare_parameter('out_topic', '/camera/camera/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'base_link')
        # VESC IMU 는 deg/s, g 로 나온다. 이미 SI 라면 1.0, 1.0 으로 둘 것.
        self.declare_parameter('gyro_scale', DEG2RAD)
        self.declare_parameter('accel_scale', G)
        # covariance 0 은 robot_localization 이 1e-9 로 치환해서 센서를 맹신하게 만든다.
        self.declare_parameter('gyro_variance', 0.02)
        self.declare_parameter('accel_variance', 0.5)
        self.declare_parameter('orientation_variance', 0.05)
        # 정지 구간에서 자이로 bias 를 추정해 뺀다. yaw 가 자이로 적분뿐이라 필수에 가깝다.
        self.declare_parameter('estimate_bias', True)
        self.declare_parameter('bias_speed_threshold', 0.02)
        self.declare_parameter('bias_min_samples', 40)

        p = self.get_parameter
        self.frame_id = p('frame_id').value
        self.gyro_scale = p('gyro_scale').value
        self.accel_scale = p('accel_scale').value
        self.gyro_var = p('gyro_variance').value
        self.accel_var = p('accel_variance').value
        self.orient_var = p('orientation_variance').value
        self.estimate_bias = p('estimate_bias').value
        self.speed_thr = p('bias_speed_threshold').value
        self.min_samples = p('bias_min_samples').value

        self.stopped = True
        self.bias = [0.0, 0.0, 0.0]
        self.bias_sum = [0.0, 0.0, 0.0]
        self.bias_n = 0
        self.bias_ready = False

        self.pub = self.create_publisher(Imu, p('out_topic').value, 10)
        self.create_subscription(Imu, p('in_topic').value, self.on_imu, 50)
        if self.estimate_bias:
            self.create_subscription(Odometry, p('odom_topic').value, self.on_odom, 20)

        self.get_logger().info(
            f"{p('in_topic').value} -> {p('out_topic').value} "
            f"(gyro x{self.gyro_scale:.5f}, accel x{self.accel_scale:.3f}, "
            f"frame_id='{self.frame_id}', bias={'on' if self.estimate_bias else 'off'})")

    def on_odom(self, msg):
        self.stopped = abs(msg.twist.twist.linear.x) < self.speed_thr

    def on_imu(self, msg):
        gx = msg.angular_velocity.x * self.gyro_scale
        gy = msg.angular_velocity.y * self.gyro_scale
        gz = msg.angular_velocity.z * self.gyro_scale

        if self.estimate_bias and self.stopped:
            self.bias_sum[0] += gx
            self.bias_sum[1] += gy
            self.bias_sum[2] += gz
            self.bias_n += 1
            if self.bias_n >= self.min_samples:
                self.bias = [s / self.bias_n for s in self.bias_sum]
                if not self.bias_ready:
                    self.bias_ready = True
                    self.get_logger().info(
                        'gyro bias [%.4f %.4f %.4f] rad/s (%d samples)'
                        % (*self.bias, self.bias_n))

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id

        out.orientation = msg.orientation
        out.angular_velocity.x = gx - self.bias[0]
        out.angular_velocity.y = gy - self.bias[1]
        out.angular_velocity.z = gz - self.bias[2]
        out.linear_acceleration.x = msg.linear_acceleration.x * self.accel_scale
        out.linear_acceleration.y = msg.linear_acceleration.y * self.accel_scale
        out.linear_acceleration.z = msg.linear_acceleration.z * self.accel_scale

        for i in (0, 4, 8):
            out.orientation_covariance[i] = self.orient_var
            out.angular_velocity_covariance[i] = self.gyro_var
            out.linear_acceleration_covariance[i] = self.accel_var

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
