"""Summarise the latest command-mode run log: python3 summ.py stanley|lqr"""
import glob
import os
import sys

import numpy as np
import pandas as pd

which = sys.argv[1]
if which == 'stanley':
    f = sorted(glob.glob(os.path.expanduser('~/stanley_logs/run_*_ego_path.csv')))[-1]
    d = pd.read_csv(f)
    e = d.cte
else:
    f = sorted(glob.glob(os.path.expanduser('~/roar_lqr_logs/run_*_ego_path.csv')))[-1]
    d = pd.read_csv(f)
    e = d.ey
step = np.hypot(np.diff(d.x), np.diff(d.y))
s = np.r_[0, np.cumsum(step)]
print(f'{f}: {len(d)} rows; dist {s[-1]:.1f} m; max|e| {e.abs().max():.3f} m; '
      f'rms {np.sqrt((e ** 2).mean()):.3f} m; max step {step.max():.2f} m; frozen steps {(step < 1e-6).sum()}')
