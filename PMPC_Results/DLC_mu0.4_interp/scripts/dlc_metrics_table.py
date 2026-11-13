# -*- coding: utf-8 -*-
"""DLC 70/75/80 km/h 三控制器指标表（减振器力 x3，Q_norm(5)=50）。

三控制器整定（pmpc_init case 1）：
  MPC    slack_w=5e4
  ZENG   slack_w=0.2e4, rho_I0=1.0, Q_norm(6)=10
  Prop   slack_w=0.2e4, w1_max=w2_max=0.1e4, Wb=10, Q_norm(6)=10

输出：dlc_metrics_70_75_80.csv
用法：python dlc_metrics_table.py [结果目录]
"""
import csv
import json
import os
import struct
import sys

import numpy as np
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)

SPEEDS = [70, 75, 80]
CTRLS = [('MPC',      'q50_mpc_%d'),
         ('ZENG',     'q50_zeng_%d'),
         ('Proposed', 'p_w10_%d')]
T_STAT = 2.0


def load_vs(vs_path):
    raw = open(vs_path, 'rb').read().decode('utf-8', errors='ignore')
    depth, in_str, esc, end = 0, False, False, None
    for i, ch in enumerate(raw):
        if in_str:
            if esc:
                esc = False
            elif ch == chr(92):
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    hdr = json.loads(raw[:end])
    names = [c['Name Aliases'][0] for c in hdr['VsChannelGroup']['Channels']]
    buf = open(vs_path[:-3] + '.vsb', 'rb').read()
    nch = struct.unpack_from('<6i', buf, 0)[5]
    data = np.frombuffer(buf, dtype='<f4', offset=24)
    ns = data.size // nch
    return names, data[:nch * ns].reshape(ns, nch)


def main():
    rows = []
    for sp in SPEEDS:
        for name, pat in CTRLS:
            stem = pat % sp
            S = loadmat(os.path.join(OUT, stem + '.mat'))
            D, t = S['D'], S['t'].ravel()
            ch, M = load_vs(os.path.join(OUT, stem + '.vs'))
            c = lambda n: M[:, ch.index(n)]
            w = t >= T_STAT
            ey = D[0, w]
            rows.append(dict(
                speed=sp, controller=name, file=stem,
                ey_max=round(float(np.nanmax(np.abs(ey))), 4),
                ey_rms=round(float(np.sqrt(np.nanmean(ey ** 2))), 4),
                ey_final=round(float(D[0, -1]), 4),
                epsi_max_deg=round(float(np.nanmax(np.abs(np.rad2deg(D[1, w])))), 3),
                sw_max_deg=round(float(np.nanmax(np.abs(D[18, w]))), 2),
                mz_max=round(float(np.nanmax(np.abs(D[17, w]))), 1),
                md_max=round(float(np.nanmax(np.abs(D[5, w]))), 1),
                beta_max_deg=round(float(np.nanmax(np.abs(c('Beta')))), 3),
                r_max_degs=round(float(np.nanmax(np.abs(c('AVz')))), 3),
                ltr_max=round(float(np.nanmax(np.abs(c('LTR')))), 4),
                qp_pct=round(float(100 * np.mean(D[23, :] > 0.5)), 1)))

    path = os.path.join(OUT, 'dlc_metrics_70_75_80.csv')
    with open(path, 'w', newline='', encoding='utf-8') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    print('%-6s %-10s %8s %8s %8s %8s %8s %8s' %
          ('speed', 'ctrl', 'ey_max', 'ey_rms', 'epsi', 'SW', 'beta', 'r'))
    for r in rows:
        print('%6d %-10s %8.3f %8.3f %8.2f %8.1f %8.2f %8.2f' % (
            r['speed'], r['controller'], r['ey_max'], r['ey_rms'],
            r['epsi_max_deg'], r['sw_max_deg'], r['beta_max_deg'], r['r_max_degs']))
    print('saved ->', path)


if __name__ == '__main__':
    main()
