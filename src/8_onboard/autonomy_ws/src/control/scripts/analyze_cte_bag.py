#!/usr/bin/env python3
"""
Measure cross-track error against the raceline the car was following.

CTE is the signed perpendicular distance from the vehicle to the path.  The
car touches a wall once |CTE| + half the vehicle width exceeds the clearance
the raceline was given, so the number to beat is small: the 0809 raceline runs
as close as 0.304 m to a wall, which leaves 0.156 m once the 0.148 m half
width is taken out.

The shape of the error says more than its size:

  sign flips repeatedly     loop gain too high -> raise lookahead_time
  steady offset to one side steering trim or a biased AMCL pose
  outward on corners only   understeer, or max_lateral_acceleration too high
  inward on corners only    corner cutting -> lower lookahead_max
  small CTE but still hits  the pose itself is wrong, not the tracking

That last case is why the pose comes from TF: CTE is measured in whatever the
localiser believes, so a clean CTE next to a scraped bumper points at AMCL
rather than at the controller.

Usage:
  python3 analyze_cte_bag.py <bag> <raceline.csv> [--map track.yaml]
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

from geometry_msgs.msg import TransformStamped  # noqa: F401  (type registry)
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage

ODOM_TOPICS = ('/odom', '/ego_racecar/odom')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))
    frames = {}          # child -> {'parent': str, 'rows': [(t, x, y, yaw)]}
    odom = []            # (t, frame_id, child, x, y, yaw, vx)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        stamp *= 1e-9
        if topic in ('/tf', '/tf_static'):
            for tr in deserialize_message(data, TFMessage).transforms:
                entry = frames.setdefault(
                    tr.child_frame_id,
                    {'parent': tr.header.frame_id, 'rows': []})
                entry['rows'].append((
                    stamp, tr.transform.translation.x,
                    tr.transform.translation.y,
                    yaw_of(tr.transform.rotation)))
        elif topic in ODOM_TOPICS:
            m = deserialize_message(data, Odometry)
            odom.append((stamp, m.header.frame_id, m.child_frame_id,
                         m.pose.pose.position.x, m.pose.pose.position.y,
                         yaw_of(m.pose.pose.orientation),
                         m.twist.twist.linear.x))
    for entry in frames.values():
        entry['rows'] = np.array(sorted(entry['rows']), dtype=float)
    return frames, odom


def sample_frame(entry, times):
    """Interpolate one transform, unwrapping yaw so it does not jump."""
    rows = entry['rows']
    if len(rows) == 1:
        return (np.full(len(times), rows[0, 1]),
                np.full(len(times), rows[0, 2]),
                np.full(len(times), rows[0, 3]))
    return (np.interp(times, rows[:, 0], rows[:, 1]),
            np.interp(times, rows[:, 0], rows[:, 2]),
            np.interp(times, rows[:, 0], np.unwrap(rows[:, 3])))


def chain_to_global(frames, base, global_frame, times):
    """Compose base -> global by walking parents, or return None."""
    x = np.zeros(len(times))
    y = np.zeros(len(times))
    yaw = np.zeros(len(times))
    frame = base
    for _ in range(12):
        if frame == global_frame:
            return x, y, yaw
        entry = frames.get(frame)
        if entry is None:
            return None
        px, py, pyaw = sample_frame(entry, times)
        cos, sin = np.cos(pyaw), np.sin(pyaw)
        x, y = px + cos * x - sin * y, py + sin * x + cos * y
        yaw = yaw + pyaw
        frame = entry['parent']
    return None


def load_path(path):
    rows = [r for r in csv.reader(open(path)) if r and not r[0].startswith('#')]
    head = [c.strip().lower() for c in rows[0]]
    xi = next((head.index(n) for n in ('x', 'x_m') if n in head), None)
    yi = next((head.index(n) for n in ('y', 'y_m') if n in head), None)
    body = rows[1:] if xi is not None else rows
    if xi is None:
        xi, yi = 0, 1
    pts = np.array([[float(r[xi]), float(r[yi])] for r in body if r])
    if len(pts) > 1 and np.linalg.norm(pts[0] - pts[-1]) < 1e-6:
        pts = pts[:-1]
    return pts


def project(pts, xy):
    """Signed lateral offset and nearest index for each pose."""
    seg = np.roll(pts, -1, axis=0) - pts
    length2 = np.maximum(np.sum(seg ** 2, axis=1), 1e-12)
    normal = np.column_stack((-seg[:, 1], seg[:, 0])) / np.sqrt(length2)[:, None]
    lateral = np.empty(len(xy))
    index = np.empty(len(xy), dtype=int)
    for i, point in enumerate(xy):
        rel = point - pts
        frac = np.clip(np.sum(rel * seg, axis=1) / length2, 0.0, 1.0)
        foot = pts + frac[:, None] * seg
        dist = np.linalg.norm(foot - point, axis=1)
        k = int(np.argmin(dist))
        index[i] = k
        lateral[i] = float(np.dot(point - foot[k], normal[k]))
    return lateral, index


def path_curvature(pts, stride_m=0.30):
    n = len(pts)
    ds = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    stride = max(1, int(round(stride_m / max(np.median(ds), 1e-6))))
    kappa = np.zeros(n)
    for i in range(n):
        p1, p2, p3 = pts[(i - stride) % n], pts[i], pts[(i + stride) % n]
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)
        if min(a, b, c) < 1e-6:
            continue
        cross = ((p2[0] - p1[0]) * (p3[1] - p1[1])
                 - (p2[1] - p1[1]) * (p3[0] - p1[0]))
        kappa[i] = 2.0 * abs(cross) / (a * b * c)
    return kappa


def wall_clearance(map_yaml, pts):
    """Distance from each path point to the nearest occupied cell."""
    try:
        import yaml
        from scipy.ndimage import distance_transform_edt
    except ImportError:
        return None
    meta = yaml.safe_load(open(map_yaml))
    image = os.path.join(os.path.dirname(os.path.abspath(map_yaml)),
                         meta['image'])
    blob = open(image, 'rb').read()
    tok, i = [], 0
    while len(tok) < 4:
        while blob[i:i + 1].isspace():
            i += 1
        if blob[i:i + 1] == b'#':
            while blob[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not blob[j:j + 1].isspace():
            j += 1
        tok.append(blob[i:j])
        i = j
    i += 1
    w, h = int(tok[1]), int(tok[2])
    img = np.frombuffer(blob[i:i + w * h], dtype=np.uint8).reshape(h, w)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    occupied = img < 255 * (1.0 - float(meta.get('occupied_thresh', 0.65)))
    field = distance_transform_edt(~occupied) * res
    px = np.clip(((pts[:, 0] - ox) / res).astype(int), 0, w - 1)
    py = np.clip((h - 1 - (pts[:, 1] - oy) / res).astype(int), 0, h - 1)
    return field[py, px]


def stats(name, values, travelled):
    if not len(values):
        print('  %-8s (샘플 없음)' % name)
        return
    centred = values - values.mean()
    flips = int(np.count_nonzero(np.diff(np.sign(centred)) != 0))
    per_m = flips / max(travelled, 1e-6)
    wave = 2.0 / per_m if per_m > 1e-6 else float('inf')
    print('  %-8s RMS=%.3f  p95=%.3f  max=%.3f  평균(DC)=%+.3f  '
          '반전=%.2f/m  파장=%.2fm'
          % (name, float(np.sqrt(np.mean(values ** 2))),
             float(np.percentile(np.abs(values), 95)),
             float(np.abs(values).max()), float(values.mean()),
             per_m, wave))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag')
    ap.add_argument('raceline')
    ap.add_argument('--map', help='track .yaml, 벽 여유까지 대조할 때')
    ap.add_argument('--global-frame', default='map')
    ap.add_argument('--base-frame', default='',
                    help='비우면 odom 메시지의 child_frame_id 를 쓴다')
    ap.add_argument('--half-width', type=float, default=0.148)
    ap.add_argument('--straight-kappa', type=float, default=0.30)
    args = ap.parse_args()

    frames, odom = read_bag(args.bag)
    if not odom:
        sys.exit('bag 에 %s 가 없다' % ' / '.join(ODOM_TOPICS))
    times = np.array([r[0] for r in odom])
    speed = np.array([r[6] for r in odom])
    base = args.base_frame or odom[0][2]

    if odom[0][1] == args.global_frame:
        xy = np.array([[r[3], r[4]] for r in odom])
    else:
        chained = chain_to_global(frames, base, args.global_frame, times)
        if chained is None:
            sys.exit(
                '%s -> %s TF 체인을 만들 수 없다. 기록된 프레임: %s\n'
                '실차면 AMCL 이 map->odom 을 내보내는 동안 녹화했는지 확인할 것.'
                % (base, args.global_frame, ', '.join(sorted(frames))))
        xy = np.column_stack(chained[:2])

    pts = load_path(args.raceline)
    lateral, index = project(pts, xy)
    kappa = path_curvature(pts)[index]
    moving = speed > 0.15
    lateral, kappa, xy, speed = (lateral[moving], kappa[moving],
                                 xy[moving], speed[moving])
    travelled = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))

    print('%d 샘플 · %.1f s · %.1f m · 평균 %.2f m/s\n'
          % (len(lateral), times[-1] - times[0], travelled, speed.mean()))
    print('=== CTE [m] ===')
    stats('전체', lateral, travelled)
    straight = kappa < args.straight_kappa
    stats('직선', lateral[straight], travelled * straight.mean())
    stats('코너', lateral[~straight], travelled * (~straight).mean())

    if args.map:
        clear = wall_clearance(args.map, pts)
        if clear is None:
            print('\n(맵 대조 생략: numpy/scipy/yaml 필요)')
        else:
            budget = clear[index[moving]] - args.half_width
            over = np.abs(lateral) > budget
            print('\n=== 벽 여유 대조 (반폭 %.3f m) ===' % args.half_width)
            print('  경로가 확보한 여유  최소=%.3f  중앙=%.3f m'
                  % (budget.min(), float(np.median(budget))))
            print('  여유를 넘긴 샘플    %d / %d  (%.1f%%)'
                  % (int(over.sum()), len(over), 100.0 * over.mean()))
            worst = float((np.abs(lateral) - budget).max())
            print('  최악 초과량        %+.3f m' % worst)

    print('\n=== 해석 ===')
    dc = float(lateral.mean())
    rms = float(np.sqrt(np.mean(lateral ** 2)))
    centred = lateral - dc
    per_m = np.count_nonzero(np.diff(np.sign(centred)) != 0) / max(travelled, 1e-6)
    if abs(dc) > 0.6 * rms:
        print('  한쪽으로 %+.3f m 치우쳐 있다 -> 조향 트림 또는 AMCL 편향.' % dc)
    if per_m > 0.6:
        print('  %.2f 회/m 로 부호가 뒤집힌다 (파장 %.2f m) -> 루프 게인 과다.'
              % (per_m, 2.0 / per_m))
        print('  lookahead_time 을 0.30 -> 0.45 로 올려서 다시 잴 것.')
    if (~straight).any() and np.abs(lateral[~straight]).mean() > \
            1.4 * max(np.abs(lateral[straight]).mean() if straight.any() else 0, 1e-6):
        print('  코너 오차가 직선보다 크다 -> max_lateral_acceleration 과다 '
              '또는 언더스티어.')
    if abs(dc) <= 0.6 * rms and per_m <= 0.6:
        print('  뚜렷한 진동도 치우침도 없다. 남은 후보는 위치추정 오차다 — '
              'RViz 에서 LaserScan 과 벽 정렬을 볼 것.')


if __name__ == '__main__':
    main()
