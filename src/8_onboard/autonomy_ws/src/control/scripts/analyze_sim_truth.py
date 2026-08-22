#!/usr/bin/env python3
"""
Split a sim bag's tracking error into the controller's share and AMCL's.

analyze_cte_bag.py measures the pose the car believed it had, because that is
all a real bag contains.  A sim bag also carries /ground_truth/odom, so the
same run answers a question the real car cannot:

  지면진실 CTE   차가 실제로 벗어난 양      -> 제어기 몫
  TF 기준 CTE    차가 벗어났다고 믿은 양    -> analyze_cte_bag.py 출력
  AMCL 오차      둘 사이의 차이             -> 위치추정 몫

AMCL 오차가 CTE 보다 크면 제어기를 아무리 만져도 숫자가 안 움직인다.  벽
여유는 지면진실 기준으로만 따져야 한다 -- 믿은 위치가 아니라 실제 위치가
벽에 닿기 때문이다.

Usage:
  python3 analyze_sim_truth.py <bag> <raceline.csv>
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from analyze_cte_bag import load_path, project, yaw_of  # noqa: E402

from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

TRUTH_TOPIC = '/ground_truth/odom'


def read_bag(path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))
    frames = {}          # (parent, child) -> [(t, x, y, yaw)]
    truth = []           # (t, x, y)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        stamp *= 1e-9
        if topic in ('/tf', '/tf_static'):
            for tr in deserialize_message(data, TFMessage).transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                frames.setdefault(key, []).append(
                    (stamp, tr.transform.translation.x,
                     tr.transform.translation.y, yaw_of(tr.transform.rotation)))
        elif topic == TRUTH_TOPIC:
            pose = deserialize_message(data, Odometry).pose.pose
            truth.append((stamp, pose.position.x, pose.position.y))
    return frames, np.array(truth)


def sample_at(rows, times):
    """Nearest-earlier transform for each requested time."""
    arr = np.array(rows)
    idx = np.searchsorted(arr[:, 0], times).clip(1, len(arr) - 1)
    return arr[idx]


def estimated_xy(frames, times):
    """Compose map->odom with odom->base into the pose AMCL believed."""
    map_odom = frames.get(('map', 'odom'))
    odom_base = next((frames[key] for key in frames
                      if key[0] == 'odom' and key[1].endswith('base_link')), None)
    if map_odom is None or odom_base is None:
        return None, None
    outer = sample_at(map_odom, times)
    inner = sample_at(odom_base, times)
    cos, sin = np.cos(outer[:, 3]), np.sin(outer[:, 3])
    x = outer[:, 1] + cos * inner[:, 1] - sin * inner[:, 2]
    y = outer[:, 2] + sin * inner[:, 1] + cos * inner[:, 2]
    return np.column_stack((x, y)), np.array(map_odom)


def summarise(name, lateral):
    print('  %-10s RMS=%.4f  p95=%.4f  max=%.4f  평균(DC)=%+.4f'
          % (name, math.sqrt(float((lateral ** 2).mean())),
             float(np.percentile(np.abs(lateral), 95)),
             float(np.abs(lateral).max()), float(lateral.mean())))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('raceline')
    ap.add_argument('--half-width', type=float, default=0.148)
    args = ap.parse_args()

    frames, truth = read_bag(args.bag)
    if not len(truth):
        sys.exit('%s 가 bag 에 없다.  sim 에서 -a 로 녹화한 bag 이 맞는지 볼 것.'
                 % TRUTH_TOPIC)

    pts = load_path(args.raceline)
    truth_lateral, _ = project(pts, truth[:, 1:3])

    print('%d 샘플 · 지면진실 %s' % (len(truth), TRUTH_TOPIC))
    print('\n=== CTE [m] ===')
    summarise('지면진실', truth_lateral)

    estimate, map_odom = estimated_xy(frames, truth[:, 0])
    if estimate is None:
        print('  map->odom 또는 odom->base_link 가 없다.  AMCL 대조는 건너뛴다.')
        return

    believed_lateral, _ = project(pts, estimate)
    summarise('TF(믿음)', believed_lateral)

    error = np.hypot(estimate[:, 0] - truth[:, 1], estimate[:, 1] - truth[:, 2])
    print('\n=== AMCL 위치추정 오차 [m] ===')
    print('  RMS=%.4f  p95=%.4f  max=%.4f'
          % (math.sqrt(float((error ** 2).mean())),
             float(np.percentile(error, 95)), float(error.max())))
    print('  map->odom 보정량  최대 이동=%.4f m  최대 회전=%.4f rad'
          % (float(np.hypot(map_odom[:, 1], map_odom[:, 2]).max()),
             float(np.abs(map_odom[:, 3]).max())))

    print('\n=== 해석 ===')
    truth_rms = math.sqrt(float((truth_lateral ** 2).mean()))
    if math.sqrt(float((error ** 2).mean())) > truth_rms:
        print('  AMCL 오차가 CTE 보다 크다 -> 제어기 튜닝으로는 안 줄어든다.')
        print('  위치추정을 먼저 잡을 것.')
    else:
        print('  CTE 가 AMCL 오차보다 크다 -> 제어기 몫이 지배적이다.')
    print('  실제 벽 여유는 지면진실 max=%.3f m 기준으로 따질 것 '
          '(반폭 %.3f m 별도).'
          % (float(np.abs(truth_lateral).max()), args.half_width))


if __name__ == '__main__':
    main()
