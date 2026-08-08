#!/usr/bin/env python3
"""Log map->odom, odom->base and map->base during a bag replay.

map->odom  : total correction SLAM applies (smooth = frontend, jump = loop closure)
odom->base : raw wheel odometry
map->base  : SLAM's corrected pose

Comparing cumulative yaw of odom->base vs map->base gives the odometry
yaw scale error directly.
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Logger(Node):
    def __init__(self, out_path, base):
        super().__init__('map_odom_logger')
        # bag 재생이면 /clock 이 있고, 실주행이면 없다. 잘못 잡으면 TF 조회가
        # 통째로 실패하므로 사용자에게 맡기지 않고 직접 확인한다.
        # 노드 생성 직후에는 DDS 디스커버리가 안 끝나 0 이 나온다. 원격
        # (다른 머신의 bag 재생) 이면 몇 초 걸리므로 그동안 기다린다.
        sim = False
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self.count_publishers('/clock'):
                sim = True
                break
            time.sleep(0.1)
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, sim)])
        self.get_logger().info(
            f'use_sim_time={sim} ({"bag replay" if sim else "live run"})')
        self.base = base
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.f = open(out_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['stamp',
                         'mo_x', 'mo_y', 'mo_yaw',
                         'ob_x', 'ob_y', 'ob_yaw',
                         'mb_x', 'mb_y', 'mb_yaw'])
        self.last_stamp = None
        self.n = 0
        self.t0 = None
        self.hist = []          # (elapsed, mo_yaw) for the live trend readout
        self.next_report = 5.0
        self.create_timer(0.02, self.tick)

    def report(self, el):
        """계통(추세)과 비계통(진동)을 주행 중에 바로 보여준다."""
        n = len(self.hist)
        mt = sum(h[0] for h in self.hist) / n
        mv = sum(h[1] for h in self.hist) / n
        den = sum((h[0] - mt) ** 2 for h in self.hist)
        if den <= 0:
            return
        slope = sum((h[0] - mt) * (h[1] - mv) for h in self.hist) / den
        inter = mv - slope * mt
        res = [h[1] - (slope * h[0] + inter) for h in self.hist]
        rms = math.sqrt(sum(r * r for r in res) / n)
        self.get_logger().info(
            f't={el:6.1f}s  mo_yaw={self.hist[-1][1]:+7.2f}deg  '
            f'trend={slope:+.3f}deg/s  osc_rms={rms:5.2f}deg  '
            f'peak={max(abs(r) for r in res):5.2f}deg  rows={self.n}')

    def look(self, a, b):
        try:
            return self.buffer.lookup_transform(a, b, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def tick(self):
        mo = self.look('map', 'odom')
        ob = self.look('odom', self.base)
        mb = self.look('map', self.base)
        if mo is None or ob is None or mb is None:
            return
        stamp = ob.header.stamp.sec + ob.header.stamp.nanosec * 1e-9
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        row = [f'{stamp:.4f}']
        for t in (mo, ob, mb):
            tr = t.transform.translation
            row += [f'{tr.x:.4f}', f'{tr.y:.4f}',
                    f'{math.degrees(yaw_of(t.transform.rotation)):.4f}']
        self.w.writerow(row)
        self.n += 1
        if self.t0 is None:
            self.t0 = stamp
        el = stamp - self.t0
        self.hist.append((el, math.degrees(yaw_of(mo.transform.rotation))))
        if el >= self.next_report:
            self.next_report = el + 5.0
            self.f.flush()
            self.report(el)


def main():
    rclpy.init()
    base = sys.argv[2] if len(sys.argv) > 2 else 'base_link'
    node = Logger(sys.argv[1], base)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy._rclpy_pybind11.RCLError:
        pass        # SIGTERM 중 컨텍스트 종료 — 기록은 이미 끝난 상태
    node.f.flush()
    node.f.close()
    print(f'\nwrote {node.n} rows -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
