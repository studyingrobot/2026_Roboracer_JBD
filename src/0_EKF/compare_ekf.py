#!/usr/bin/env python3
"""EKF 출력 bag(/odometry/filtered)과 원본 bag(/odom)의 궤적을 비교한다.
  python3 compare_ekf.py 0808_test_2
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__)))),
    'src', '1_slam_mapping', 'maps', 'test_bags')


def read(uri, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    xy, yaw, wz, vx = [], [], [], []
    while r.has_next():
        tp, d, _ = r.read_next()
        if tp != topic:
            continue
        m = deserialize_message(d, get_message(types[tp]))
        q = m.pose.pose.orientation
        xy.append([m.pose.pose.position.x, m.pose.pose.position.y])
        yaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z)))
        wz.append(m.twist.twist.angular.z)
        vx.append(m.twist.twist.linear.x)
    return np.array(xy), np.unwrap(np.array(yaw)), np.array(wz), np.array(vx)


def report(name, xy, yaw, wz, vx):
    d = np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()
    print(f'  {name:22s} n={len(xy):5d}  '
          f'끝점=({xy[-1, 0]:7.2f},{xy[-1, 1]:7.2f})  주행거리={d:6.1f}m  '
          f'yaw변화={math.degrees(yaw[-1] - yaw[0]):8.0f}deg  '
          f'|wz|max={np.abs(wz).max():6.2f}rad/s')


bag = sys.argv[1] if len(sys.argv) > 1 else '0808_test_2'
print(f'=== {bag} ===')
report('bag /odom (vesc)', *read(f'{ROOT}/{bag}', '/odom'))
report('EKF /odometry/filtered', *read(f'{ROOT}/{bag}_ekf', '/odometry/filtered'))
