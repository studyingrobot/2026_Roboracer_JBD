#!/usr/bin/env python3
"""Separate systematic odom yaw-scale error from loop-closure jumps."""
import csv, math, sys

rows = list(csv.DictReader(open(sys.argv[1])))
g = lambda r, k: float(r[k])
t0 = g(rows[0], 'stamp')
T = [g(r, 'stamp') - t0 for r in rows]


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


# per-sample deltas
d_ob, d_mb, d_mo, d_mot = [], [], [], []
for i in range(1, len(rows)):
    d_ob.append(wrap(g(rows[i], 'ob_yaw') - g(rows[i - 1], 'ob_yaw')))
    d_mb.append(wrap(g(rows[i], 'mb_yaw') - g(rows[i - 1], 'mb_yaw')))
    d_mo.append(wrap(g(rows[i], 'mo_yaw') - g(rows[i - 1], 'mo_yaw')))
    d_mot.append(math.hypot(g(rows[i], 'mo_x') - g(rows[i - 1], 'mo_x'),
                            g(rows[i], 'mo_y') - g(rows[i - 1], 'mo_y')))

# a sample is "clean" when the correction did not step this frame
clean = [i for i in range(len(d_ob)) if abs(d_mo[i]) < 0.05 and d_mot[i] < 0.005]
print(f'clean samples {len(clean)} / {len(d_ob)}')

# --- yaw scale: regress d_slam on d_odom through the origin, clean frames only
num = sum(d_ob[i] * d_mb[i] for i in clean)
den = sum(d_ob[i] ** 2 for i in clean)
print(f'\n[yaw scale, no-jump frames]  slam/odom = {num/den:.4f}')

moving = [i for i in clean if abs(d_ob[i]) > 0.05]
num = sum(d_ob[i] * d_mb[i] for i in moving)
den = sum(d_ob[i] ** 2 for i in moving)
print(f'[yaw scale, turning only  ]  slam/odom = {num/den:.4f}  (n={len(moving)})')

# --- path length excluding jump frames
def plen(kx, ky, idx):
    return sum(math.hypot(g(rows[i + 1], kx) - g(rows[i], kx),
                          g(rows[i + 1], ky) - g(rows[i], ky)) for i in idx)


print(f'\npath (clean frames) odom={plen("ob_x","ob_y",clean):.1f} m  '
      f'slam={plen("mb_x","mb_y",clean):.1f} m  '
      f'ratio={plen("mb_x","mb_y",clean)/plen("ob_x","ob_y",clean):.4f}')
print(f'jump translation total = {sum(d_mot):.1f} m over '
      f'{sum(1 for v in d_mot if v > 0.005)} stepped frames')

# --- trend vs oscillation in map->odom yaw
mo = [g(r, 'mo_yaw') for r in rows]
n = len(T)
mt = sum(T) / n
mv = sum(mo) / n
slope = sum((T[i] - mt) * (mo[i] - mv) for i in range(n)) / \
        sum((T[i] - mt) ** 2 for i in range(n))
inter = mv - slope * mt
resid = [mo[i] - (slope * T[i] + inter) for i in range(n)]
rms = math.sqrt(sum(r * r for r in resid) / n)
print(f'\nmap->odom yaw trend  = {slope:+.3f} deg/s  '
      f'({slope*T[-1]:+.1f} deg over the run)')
print(f'oscillation about trend: rms={rms:.1f} deg  '
      f'peak={max(abs(r) for r in resid):.1f} deg')

# --- lap detection: return near the starting odom position
x0, y0 = g(rows[0], 'mb_x'), g(rows[0], 'mb_y')
near, laps = False, []
for i in range(n):
    d = math.hypot(g(rows[i], 'mb_x') - x0, g(rows[i], 'mb_y') - y0)
    if d < 1.0 and not near and T[i] > 5:
        laps.append(T[i]); near = True
    elif d > 2.0:
        near = False
print(f'\nreturns near start at t = {[round(v,1) for v in laps]}')

# --- biggest discontinuities
big = sorted(range(len(d_mo)), key=lambda i: -abs(d_mo[i]))[:12]
print('\nlargest map->odom yaw steps:')
for i in sorted(big):
    print(f'  t={T[i+1]:6.2f}s  dyaw={d_mo[i]:+7.2f} deg  dtrans={d_mot[i]:.3f} m')
