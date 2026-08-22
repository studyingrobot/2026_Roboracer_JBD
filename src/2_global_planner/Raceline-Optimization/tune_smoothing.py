#!/usr/bin/env python3
"""
Find the smoothing and vehicle-width settings a track can actually support,
and show what each of them costs.

main_globaltraj_f110.py fails in two opposite ways, and the settings that fix
one make the other worse:

  "At least two spline normals are crossed"   -> 평활화가 부족하다.  s_reg 를
                                                 키워야 한다.
  "constraints are inconsistent, no solution" -> 평활화된 기준선이 코리도를
                                                 벗어났다.  s_reg 를 줄이거나
                                                 width_opt 을 낮춰야 한다.

두 제약은 트랙 형상이 정한다:

  법선 교차 안 함    중심선 곡률 반경 > 트랙 반폭
  QP 실행 가능       스플라인 최대 편차 < 반폭 - width_opt/2

s_reg 는 scipy splprep 의 s 로 그대로 들어가는 잔차 제곱'합'이라 점 개수에,
즉 트랙 길이에 비례한다.  24 m 트랙의 0.5 는 240 m 트랙의 5 에 해당하므로
트랙이 바뀌면 반드시 다시 잡아야 한다.

width_opt 은 공짜가 아니다.  키우면 주행선이 중심선 쪽으로 밀려 벽에서는
멀어지지만 코너가 날카로워진다.  곡률이 커지면 v = sqrt(a_lat/kappa) 로
코너 속도가 떨어지고, 제어기가 요구하는 조향각이 서보 한계에 먼저 닿는다.
그래서 이 스크립트는 후보마다 실제로 min-curvature 최적화를 풀어서 최소
반경과 예상 랩타임까지 같이 보여준다 -- 벽 여유만 보고 고르면 랩타임을
모르는 채로 내주게 된다.

Usage:
  python3 tune_smoothing.py --map_name track04
  python3 tune_smoothing.py --map_name <이름> --map-yaml <맵.yaml>
"""

import argparse
import configparser
import contextlib
import io
import json
import math
import os
import re
import sys

import numpy as np

import helper_funcs_glob
import trajectory_planning_helpers as tph

# QP 는 반폭에서 차폭 절반을 뺀 만큼만 기준선이 흔들리는 것을 허용한다.
# 그 예산을 다 쓰면 코너 한 곳만 어긋나도 풀리지 않으므로 여유를 남긴다.
BUDGET_USAGE = 0.85


def load_track(path):
    rows = [line for line in open(path)
            if line.strip() and not line.lstrip().startswith('#')]
    return np.array([[float(v) for v in line.split(',')[:4]] for line in rows])


def track_length(points):
    closed = np.vstack((points, points[:1]))
    return float(np.sum(np.linalg.norm(np.diff(closed, axis=0), axis=1)))


def smooth(track, s_reg, stepsize_prep, stepsize_reg):
    """옵티마이저와 같은 경로로 평활화하고 편차·법선 교차를 돌려준다."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        smoothed = tph.spline_approximation.spline_approximation(
            track=track, k_reg=3, s_reg=s_reg,
            stepsize_prep=stepsize_prep, stepsize_reg=stepsize_reg,
            debug=True)
    deviations = [float(v) for v in re.findall(r'([0-9]+\.[0-9]+)m',
                                               buf.getvalue())]
    mean_dev, max_dev = (deviations + [float('nan')] * 2)[:2]

    closed = np.vstack((smoothed[:, :2], smoothed[:1, :2]))
    coeffs_x, coeffs_y, a_interp, normvec = tph.calc_splines.calc_splines(
        path=closed)
    _, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x, coeffs_y=coeffs_y,
        ind_spls=np.arange(len(coeffs_x)),
        t_spls=np.zeros(len(coeffs_x)))
    crossing = tph.check_normals_crossing.check_normals_crossing(
        track=smoothed, normvec_normalized=normvec, horizon=10)
    radius = 1.0 / max(float(np.abs(kappa).max()), 1e-9)
    return {
        'mean_dev': mean_dev, 'max_dev': max_dev, 'radius': radius,
        'crossing': bool(crossing), 'track': smoothed,
        'normvec': normvec, 'A': a_interp,
    }


def solve_raceline(smoothed, width_opt, curvlim, stepsize_interp):
    """min-curvature 최적화를 실제로 풀어 주행선 좌표를 돌려준다."""
    with contextlib.redirect_stdout(io.StringIO()):
        alpha = tph.opt_min_curv.opt_min_curv(
            reftrack=smoothed['track'], normvectors=smoothed['normvec'],
            A=smoothed['A'], kappa_bound=curvlim, w_veh=width_opt,
            print_debug=False, plot_debug=False)[0]
        raceline = tph.create_raceline.create_raceline(
            refline=smoothed['track'][:, :2],
            normvectors=smoothed['normvec'], alpha=alpha,
            stepsize_interp=stepsize_interp)[0]
    return raceline


def speed_profile(points, a_lat, max_speed, deceleration):
    """minjae_pp_node 와 같은 방식: 정적 한계 뒤 후진 감속 패스."""
    segments = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(segments, axis=1)
    kappa = np.abs(path_curvature(points))
    speeds = np.minimum(max_speed, np.sqrt(a_lat / np.maximum(kappa, 1e-3)))
    for _ in range(3):
        for i in range(len(speeds) - 1, -1, -1):
            ahead = (i + 1) % len(speeds)
            speeds[i] = min(speeds[i], math.sqrt(
                speeds[ahead] ** 2 + 2.0 * deceleration * lengths[i]))
    lap = float(np.sum(lengths / np.maximum(speeds, 1e-3)))
    return float(kappa.max()), float(speeds.min()), lap


def path_curvature(points, stride_m=0.30):
    count = len(points)
    spacing = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    stride = max(1, int(round(stride_m / max(float(np.median(spacing)), 1e-6))))
    kappa = np.zeros(count)
    for i in range(count):
        p1 = points[(i - stride) % count]
        p2 = points[i]
        p3 = points[(i + stride) % count]
        area = ((p2[0] - p1[0]) * (p3[1] - p1[1])
                - (p2[1] - p1[1]) * (p3[0] - p1[0]))
        denominator = (np.linalg.norm(p2 - p1) * np.linalg.norm(p3 - p2)
                       * np.linalg.norm(p3 - p1))
        kappa[i] = 0.0 if denominator < 1e-9 else 2.0 * area / denominator
    return kappa


def wall_clearance(points, map_yaml):
    """맵 점유격자 기준 각 점에서 벽까지의 거리."""
    import yaml
    from PIL import Image
    from scipy.ndimage import distance_transform_edt

    meta = yaml.safe_load(open(map_yaml))
    image = np.array(Image.open(
        os.path.join(os.path.dirname(map_yaml), meta['image'])
    ).convert('L')).astype(float) / 255.0
    occupied = (1.0 - image) > meta.get('occupied_thresh', 0.65)
    distance = distance_transform_edt(~occupied) * meta['resolution']

    height = image.shape[0]
    col = ((points[:, 0] - meta['origin'][0]) / meta['resolution']).astype(int)
    row = (height - 1
           - ((points[:, 1] - meta['origin'][1]) / meta['resolution'])
           ).astype(int)
    inside = ((col >= 0) & (col < image.shape[1])
              & (row >= 0) & (row < height))
    return distance[row[inside], col[inside]]


def read_vehicle_params(module, veh_params_file):
    parser = configparser.ConfigParser()
    parser.read(os.path.join(module, 'params', veh_params_file))
    return json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map_name', required=True)
    parser.add_argument('--map-yaml',
                        help='맵 .yaml.  주면 실제 벽 거리를 측정한다')
    parser.add_argument('--veh-params-file', default='f110.ini')
    parser.add_argument('--half-width', type=float, default=0.148,
                        help='차량 반폭 [m]')
    parser.add_argument('--cte-max', type=float, default=0.15,
                        help='감당하기로 한 CTE 최댓값 [m].  이보다 예산이 '
                             '작은 width_opt 은 벽에 닿으므로 제외한다')
    parser.add_argument('--a-lat', type=float, default=1.5,
                        help='랩타임 추정에 쓸 max_lateral_acceleration')
    parser.add_argument('--speed', type=float, default=3.0)
    parser.add_argument('--deceleration', type=float, default=2.0)
    parser.add_argument('--stepsize-prep', type=float, default=0.1)
    parser.add_argument('--s-reg', type=float, nargs='*',
                        default=[0.2, 0.5, 1.0, 2.0, 5.0, 10.0])
    parser.add_argument('--stepsize-reg', type=float, nargs='*',
                        default=[0.3, 0.5, 0.8])
    parser.add_argument('--width-opt', type=float, nargs='*',
                        default=[0.9, 0.8, 0.7, 0.6, 0.5])
    args = parser.parse_args()

    module = os.path.dirname(os.path.abspath(__file__))
    track_file = os.path.join(module, 'inputs', 'tracks',
                              args.map_name + '.csv')
    if not os.path.exists(track_file):
        sys.exit('트랙 CSV 가 없다: %s' % track_file)

    veh = read_vehicle_params(module, args.veh_params_file)
    track = load_track(track_file)
    length = track_length(track[:, :2])
    half_width = float(((track[:, 2] + track[:, 3]) / 2.0).min())

    print('트랙 %s' % args.map_name)
    print('  길이 %.2f m · 입력 %d점 · 선언 최소 반폭 %.3f m'
          % (length, len(track), half_width))
    print('  차량 반폭 %.3f m · curvlim %.2f rad/m · 랩타임 추정 a_lat %.2f'
          % (args.half_width, veh['curvlim'], args.a_lat))
    print('  CTE 예산 기준 %.3f m' % args.cte_max)

    optimism = 0.0
    if args.map_yaml:
        measured = float(wall_clearance(track[:, :2], args.map_yaml).min())
        optimism = half_width - measured
        print('  맵 실측 최소 벽거리 %.3f m  (선언이 %+.3f m 낙관적)'
              % (measured, optimism))

    print('\n=== 1단계: 평활화 조합 (법선 교차 회피) ===')
    print('%-8s %-9s %-9s %-9s %-9s %s'
          % ('s_reg', 'step_reg', '평균편차', '최대편차', '최소반경', '법선'))
    feasible = []
    for stepsize_reg in args.stepsize_reg:
        for s_reg in args.s_reg:
            try:
                result = smooth(track, s_reg, args.stepsize_prep, stepsize_reg)
            except Exception as error:
                print('%-8s %-9s 실패: %s'
                      % (s_reg, stepsize_reg, str(error)[:40]))
                continue
            print('%-8s %-9s %-9.3f %-9.3f %-9.3f %s'
                  % (s_reg, stepsize_reg, result['mean_dev'],
                     result['max_dev'], result['radius'],
                     '교차' if result['crossing'] else 'OK'))
            if not result['crossing']:
                result.update(s_reg=s_reg, stepsize_reg=stepsize_reg)
                feasible.append(result)

    if not feasible:
        print('\n법선 교차를 피하는 조합이 없다.')
        print('중심선 최소 곡률 반경이 트랙 반폭(%.3f m)보다 작다는 뜻이다.'
              % half_width)
        print('--stepsize-reg 를 더 크게 주거나, 맵/중심선 추출을 다시 볼 것.')
        return

    feasible.sort(key=lambda r: r['max_dev'])

    print('\n=== 2단계: width_opt 별 실제 최적화 결과 ===')
    print('%-10s %-8s %-9s %-9s %-9s %-9s %-8s %s'
          % ('width_opt', 's_reg', 'step_reg', '벽거리', 'CTE예산',
             '최소반경', '랩타임', '판정'))
    rows = []
    for width_opt in sorted(args.width_opt, reverse=True):
        budget = (half_width - width_opt / 2.0) * BUDGET_USAGE
        usable = [r for r in feasible if r['max_dev'] <= budget]
        if not usable:
            print('%-10.2f %-8s %-9s %-9s %-9s %-9s %-8s 불가 (예산 %.3f < 최소편차 %.3f)'
                  % (width_opt, '-', '-', '-', '-', '-', '-',
                     budget, feasible[0]['max_dev']))
            continue
        # 예산을 만족하는 것 중 가장 매끄러운 쪽이 법선 여유가 크다.
        chosen = usable[-1]
        try:
            raceline = solve_raceline(
                chosen, width_opt, veh['curvlim'], args.stepsize_prep)
        except Exception as error:
            print('%-10.2f %-8s %-9s 최적화 실패: %s'
                  % (width_opt, chosen['s_reg'], chosen['stepsize_reg'],
                     str(error)[:40]))
            continue
        kappa_max, min_speed, lap = speed_profile(
            raceline, args.a_lat, args.speed, args.deceleration)
        if args.map_yaml:
            clearance = float(wall_clearance(raceline, args.map_yaml).min())
        else:
            clearance = width_opt / 2.0 - optimism
        cte_budget = clearance - args.half_width
        safe = cte_budget >= args.cte_max
        print('%-10.2f %-8s %-9s %-9.3f %-9.3f %-9.2f %-8.1f %s'
              % (width_opt, chosen['s_reg'], chosen['stepsize_reg'],
                 clearance, cte_budget, 1.0 / max(kappa_max, 1e-9), lap,
                 '가능' if safe else 'CTE 초과 (예산 < %.3f)' % args.cte_max))
        if safe:
            rows.append((width_opt, chosen, clearance, cte_budget,
                         1.0 / max(kappa_max, 1e-9), lap, min_speed))

    if not rows:
        print('\nCTE 예산 %.3f m 를 감당하는 width_opt 이 없다.' % args.cte_max)
        print('트랙이 그만큼 좁다는 뜻이다.  제어기를 더 조여 CTE 를 줄이거나,')
        print('--cte-max 를 실측에 맞춰 낮춰서 다시 볼 것.')
        return

    print('\n벽거리가 클수록 안전하고, 랩타임이 작을수록 빠르다.  둘은 반대로'
          ' 움직인다 --\nwidth_opt 을 키우면 주행선이 중심선으로 밀려 코너가'
          ' 날카로워진다.')
    print('CTE예산 = 벽거리 - 차량반폭.  실주행 CTE 최댓값이 이 값을 넘으면'
          ' 벽에 닿는다.')

    # CTE 예산을 감당하는 것 중 가장 빠른 쪽.  벽에 닿지 않는 것이 먼저고,
    # 그 조건을 만족하는 안에서만 랩타임을 다툰다.
    print('\n=== params/f110.ini 에 넣을 값 ===')
    width_opt, chosen, clearance, cte_budget, radius, lap, _ = min(
        rows, key=lambda r: r[5])
    print('reg_smooth_opts={"k_reg": 3, "s_reg": %g}' % chosen['s_reg'])
    print('stepsize_opts={"stepsize_prep": %g,' % args.stepsize_prep)
    print('               "stepsize_reg": %g,' % chosen['stepsize_reg'])
    print('               "stepsize_interp_after_opt": 0.2}')
    print('optim_opts_mincurv={"width_opt": %g,' % width_opt)
    print('                    "iqp_iters_min": 3,')
    print('                    "iqp_curverror_allowed": 0.01}')
    print('\nCTE 예산 %.3f m 를 감당하는 것 중 가장 빠른 조합이다 (랩 %.1f s).'
          % (args.cte_max, lap))
    print('이 설정의 실제 예산은 %.3f m.' % cte_budget)
    print('\n실차에는 지면진실이 없어 analyze_cte_bag.py 가 주는 값은 TF 기준,')
    print('즉 차가 "믿는" 이탈량이다.  시뮬 대조에서 실제의 약 80% 로 나오므로,')
    print('첫 주행 bag 의 TF 기준 CTE 최댓값이 %.3f m 를 넘으면 예산 초과로 보고'
          % (args.cte_max * 0.8))
    print('width_opt 을 한 단계 올려 재생성할 것.')


if __name__ == '__main__':
    main()
