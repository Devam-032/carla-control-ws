"""Replay time at which <actor_id> passes closest to world point (x, y) [metres, CARLA frame].

    python3 time_at_point.py <file.log> <actor_id> <x> <y>   -> prints "t dist"
    python3 time_at_point.py <file.log> <actor_id> --pos <t> -> prints "x y" at time t
"""
import re
import sys

import carla
import numpy as np

path, aid = sys.argv[1], sys.argv[2]
c = carla.Client('localhost', 2000)
c.set_timeout(60)
info = c.show_recorder_file_info(path, True)
T, X, Y, t = [], [], [], None
for line in info.splitlines():
    m = re.match(r'\s*Frame \d+ at ([\d.]+) seconds', line)
    if m:
        t = float(m.group(1)); continue
    m = re.match(rf'\s*Id: {aid} Location: \(([-\d.]+), ([-\d.]+), [-\d.]+\)', line)
    if m and t is not None:
        T.append(t); X.append(float(m.group(1)) / 100); Y.append(float(m.group(2)) / 100)
T, X, Y = map(np.array, (T, X, Y))
if sys.argv[3] == '--pos':
    i = np.argmin(abs(T - float(sys.argv[4])))
    print(f'{X[i]:.2f} {Y[i]:.2f}')
else:
    d = np.hypot(X - float(sys.argv[3]), Y - float(sys.argv[4]))
    i = np.argmin(d)
    print(f'{T[i]:.2f} {d[i]:.2f}')
