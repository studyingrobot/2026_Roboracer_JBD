#!/usr/bin/env python3
import csv, math, sys

rows = list(csv.DictReader(open(sys.argv[1])))
f = lambda r, k: float(r[k])
t0 = f(rows[0], 'stamp')
T = [f(r, 'stamp') - t0 for r in rows]
print(f'samples={len(rows)}  duration={T[-1]:.1f}s (sim)')


def unwrap(vals):
    out, acc, prev = [], 0.0, vals[0]
    for v in vals:
        d = (v - prev + 180.0) % 360.0 - 180.0
        acc += d
        out.append(acc)
        prev = v
    return out


ob = unwrap([f(r, 'ob_yaw') for r in rows])   # raw odometry heading
mb = unwrap([f(r, 'mb_yaw') for r in rows])   # SLAM-corrected heading
mo = [f(r, 'mo_yaw') for r in rows]           # correction (wrapped, small-ish)
mo_u = unwrap(mo)

print(f'\n--- cumulative heading over run ---')
print(f'odom (raw)      : {ob[-1]:9.1f} deg   ({ob[-1]/360:.2f} turns)')
print(f'SLAM (corrected): {mb[-1]:9.1f} deg   ({mb[-1]/360:.2f} turns)')
print(f'map->odom yaw   : {mo_u[-1]:9.1f} deg  (final {mo[-1]:.1f})')

# total absolute rotation (sum of |delta|) - scale factor estimate
def abs_rot(u):
    return sum(abs(u[i] - u[i - 1]) for i in range(1, len(u)))


ao, am = abs_rot(ob), abs_rot(mb)
print(f'\ntotal |rotation| odom={ao:.0f} deg  slam={am:.0f} deg  '
      f'ratio slam/odom={am/ao:.4f}')

# path length
def plen(kx, ky):
    s = 0.0
    for i in range(1, len(rows)):
        s += math.hypot(f(rows[i], kx) - f(rows[i - 1], kx),
                        f(rows[i], ky) - f(rows[i - 1], ky))
    return s


print(f'path length odom={plen("ob_x","ob_y"):.1f} m  '
      f'slam={plen("mb_x","mb_y"):.1f} m')

# jump detection on map->odom
print('\n--- map->odom discontinuities (loop closures) ---')
jumps = []
for i in range(1, len(rows)):
    dy = (mo[i] - mo[i - 1] + 180.0) % 360.0 - 180.0
    dt = math.hypot(f(rows[i], 'mo_x') - f(rows[i - 1], 'mo_x'),
                    f(rows[i], 'mo_y') - f(rows[i - 1], 'mo_y'))
    if abs(dy) > 1.0 or dt > 0.10:
        jumps.append((T[i], dy, dt))
print(f'count={len(jumps)}')
for t, dy, dt in jumps[:40]:
    print(f'  t={t:7.2f}s  dyaw={dy:+7.2f} deg  dtrans={dt:.3f} m')
if len(jumps) > 40:
    print(f'  ... {len(jumps)-40} more')

# monotonicity of the correction
ups = sum(1 for i in range(1, len(mo_u)) if mo_u[i] - mo_u[i - 1] > 0.01)
dns = sum(1 for i in range(1, len(mo_u)) if mo_u[i] - mo_u[i - 1] < -0.01)
print(f'\ncorrection steps: +{ups}  -{dns}')

print('\n--- map->odom yaw trajectory (every ~5s) ---')
step = max(1, len(rows) // 40)
for i in range(0, len(rows), step):
    print(f'  t={T[i]:7.2f}s  mo_yaw={mo_u[i]:8.2f}  '
          f'mo_xy=({f(rows[i],"mo_x"):6.2f},{f(rows[i],"mo_y"):6.2f})  '
          f'odom_hdg={ob[i]:8.1f}  slam_hdg={mb[i]:8.1f}')
