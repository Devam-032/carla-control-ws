"""
Align the latest Stanley and LQR road runs on distance along the shared route
and write Portfolio/assets/stanley-vs-lqr.csv.

Both runs start from the same pose and follow the same live road reference
(junction_choice=right), but the controllers tick at different rates, so rows
are aligned by arc length s (projected onto the Stanley path) rather than time.

    python3 build_comparison.py [out.csv]
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

OUT = sys.argv[1] if len(sys.argv) > 1 else 'stanley-vs-lqr.csv'
STANLEY_LOGS = os.path.expanduser('~/stanley_logs')
LQR_LOGS = os.path.expanduser('~/roar_lqr_logs')
SKIP_START_M = 10.0      # start transient: LQR is gated off below 3 m/s
BIN_M = 0.5

st = pd.read_csv(sorted(glob.glob(f'{STANLEY_LOGS}/run_*_ego_path.csv'))[-1])
lq = pd.read_csv(sorted(glob.glob(f'{LQR_LOGS}/run_*_ego_path.csv'))[-1])

st['s'] = np.r_[0, np.cumsum(np.hypot(np.diff(st.x), np.diff(st.y)))]
P = st[['x', 'y']].values
nearest = [np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)) for x, y in lq[['x', 'y']].values]
lq['s'] = st.s.values[nearest]

s_end = min(st.s.max(), lq.s.max())
edges = np.arange(0.0, s_end + BIN_M, BIN_M)


def binned(df, col):
    idx = np.digitize(df.s, edges) - 1
    return pd.Series(df[col].values).groupby(idx).mean().reindex(range(len(edges) - 1))


out = pd.DataFrame({
    's_m': edges[:-1] + BIN_M / 2,
    'speed_stanley_mps': binned(st, 'ego_speed').values,
    'cte_stanley_m': binned(st, 'cte').values,
    'cte_lqr_m': binned(lq, 'ey').values,
    'heading_err_stanley_rad': binned(st, 'heading_error').values,
    'heading_err_lqr_rad': binned(lq, 'epsi').values,
    'steer_stanley': binned(st, 'steer').values,
    'steer_lqr': binned(lq, 'steer').values,
}).dropna(subset=['cte_stanley_m', 'cte_lqr_m'])
out.round(4).to_csv(OUT, index=False)

steady = out[out.s_m > SKIP_START_M]
for name, col in (('Stanley', 'cte_stanley_m'), ('LQR', 'cte_lqr_m')):
    e = steady[col]
    print(f'{name:8s} s>{SKIP_START_M:.0f} m: max|cte| {e.abs().max():.3f} m, rms {np.sqrt((e ** 2).mean()):.3f} m')
print(f'wrote {OUT}: {len(out)} rows, s = 0..{out.s_m.max():.0f} m')
