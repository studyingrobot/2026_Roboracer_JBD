#!/usr/bin/env python3
"""
Find the surface grip limit from a stepped-speed bag.

Signal choice matters here:

  /odom angular.z            NOT usable.  vesc.yaml sets
                             use_servo_cmd_to_calc_angular_velocity: true, so
                             it is the bicycle model recomputed from the servo
                             command -- comparing it against the model
                             compares the model with itself.
  /sensors/imu/raw gyro z    the real measurement, but in deg/s (see
                             src/0_EKF/imu_relay.py), so it needs pi/180.
  /sensors/imu/raw accel y   too noisy.  Chassis vibration put its peak at
                             4.62 m/s^2 on a run where v*omega peaked at 1.46.

The verdict is a *change* in measured/model yaw rate, not its absolute value.
That ratio sits near 1.17 on this car because the steering calibration is off,
and staying robust to that offset means watching the baseline break rather
than the number itself.

Usage:
  python3 analyze_grip_bag.py <bag_dir_or_db3> [--wheelbase 0.324]
"""

import argparse
import math
import sys

import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu

DEG2RAD = math.pi / 180.0
DRIVE_TOPICS = ('/auto', '/drive', '/teleop')


def read_bag(path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))
    odom, drive, imu = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        stamp *= 1e-9
        if topic == '/odom':
            message = deserialize_message(data, Odometry)
            odom.append((stamp, message.twist.twist.linear.x))
        elif topic in DRIVE_TOPICS:
            message = deserialize_message(data, AckermannDriveStamped)
            drive.append((stamp, message.drive.speed,
                          message.drive.steering_angle))
        elif topic == '/sensors/imu/raw':
            message = deserialize_message(data, Imu)
            imu.append((stamp, message.angular_velocity.z))
    return np.array(odom), np.array(drive), np.array(imu)


def speed_steps(commanded, tolerance=0.05):
    """Group samples by the commanded speed level they were held at."""
    levels = []
    for value in commanded:
        if value <= 0.05:
            continue
        if not any(abs(value - level) < tolerance for level in levels):
            levels.append(value)
    return sorted(levels)


def report_braking(stamps, speed, commanded, from_speed=0.5):
    """Measure deceleration wherever the command drops straight to zero."""
    dropped = (commanded[:-1] > from_speed) & (commanded[1:] <= 0.05)
    events = np.flatnonzero(dropped)
    rates = []
    for start in events:
        moving = np.flatnonzero(
            (stamps > stamps[start]) & (speed < 0.15))
        if not len(moving):
            continue
        stop = moving[0]
        span = stamps[stop] - stamps[start]
        if not 0.05 < span < 5.0:
            continue
        rates.append((speed[start] - speed[stop]) / span)
    print('\n=== 제동 (명령이 0 으로 떨어진 구간) ===')
    if not rates:
        print('  제동 구간 없음.  --brake-from 을 주고 다시 측정할 것.')
        return
    rates = np.array(rates)
    print('  n=%d  중앙=%.2f  최대=%.2f m/s^2' % (
        len(rates), np.median(rates), rates.max()))
    print('  speed_profile_deceleration 은 중앙값의 80%%인 %.2f 를 권장.'
          % (0.8 * np.median(rates)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='bag directory or .db3 file')
    parser.add_argument('--wheelbase', type=float, default=0.324,
                        help='wheelbase used by vesc_to_odom')
    parser.add_argument('--min-steering', type=float, default=0.05,
                        help='ignore samples straighter than this, in rad')
    args = parser.parse_args()

    odom, drive, imu = read_bag(args.bag)
    if not len(odom) or not len(drive) or not len(imu):
        sys.exit('bag is missing /odom, a drive topic, or /sensors/imu/raw')

    stamps = odom[:, 0]
    speed = odom[:, 1]
    steering = np.interp(stamps, drive[:, 0], drive[:, 2])
    commanded = np.interp(stamps, drive[:, 0], drive[:, 1])
    yaw_rate = np.interp(stamps, imu[:, 0], imu[:, 1]) * DEG2RAD
    model = speed * np.tan(steering) / args.wheelbase
    lateral = np.abs(speed * yaw_rate)

    usable = (np.abs(speed) > 0.3) & (np.abs(steering) > args.min_steering)
    if usable.sum() < 50:
        sys.exit('not enough cornering samples; was the steering held?')

    print('%d samples, %.1f s\n' % (len(odom), stamps[-1] - stamps[0]))
    print('%-9s %6s %7s %8s %9s' % (
        'cmd v', 'n', 'v real', 'a_y', 'ratio'))
    print('-' * 44)

    baseline = None
    verdict = None
    for level in speed_steps(commanded[usable]):
        step = usable & (np.abs(commanded - level) < 0.05)
        if step.sum() < 30:
            continue
        ratio = np.median(
            np.abs(yaw_rate[step]) / np.maximum(np.abs(model[step]), 1e-6))
        a_y = np.median(lateral[step])
        mark = ''
        if baseline is None:
            baseline = ratio
        elif ratio < 0.90 * baseline:
            mark = '  <-- 미끄러짐 시작'
            if verdict is None:
                verdict = a_y
        print('%-9.2f %6d %7.2f %8.2f %9.3f%s' % (
            level, step.sum(), np.median(speed[step]), a_y, ratio, mark))

    report_braking(stamps, speed, commanded)

    print('\nbaseline ratio = %.3f' % (baseline or float('nan')))
    if verdict is None:
        print('한계 도달 안 함.  max_lateral_acceleration >= %.2f m/s^2 는 확실.'
              % lateral[usable].max())
        print('더 빠른 계단을 추가해서 다시 측정할 것.')
    else:
        print('한계 도달.  max_lateral_acceleration 을 %.2f 로 설정할 것 '
              '(측정값 %.2f 의 80%%).' % (0.8 * verdict, verdict))


if __name__ == '__main__':
    main()
