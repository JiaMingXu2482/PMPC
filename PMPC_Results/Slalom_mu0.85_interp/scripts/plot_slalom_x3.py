# -*- coding: utf-8 -*-
"""Slalom 80 km/h / mu=0.85 / A=1.2 —— 减振器力 x3 后的定稿三曲线。

  曲线                     文件          ey max / LTR max
  MPC                      d3_sla_mpc     0.961 / 0.775
  ZENG (rho_I0=0.7)        d3_zeng_paper  1.006 / 0.750
  Proposed (no comp)       sw200_nc       0.774 / 0.68
  Proposed (comp, td=0.2)  sw200_c        0.743 / 0.69

x3 阻尼把 LTR 从 1.00 压到 0.74-0.78（不再举轮），同时 comp 比 nc 好 3.5%。

输出：slalom_80_x3.pdf（只出 PDF）
用法：python plot_slalom_x3.py [结果目录]
"""
import json
import os
import struct
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)

CASES = [
    ('d3_sla_mpc',    'MPC',                 '#1f77b4'),
    ('d3_zeng_paper', 'ZENG (I0=0.7)',       '#d62728'),
    ('sw200_nc',      'Proposed (no comp)',  '#9467bd'),
    ('sw200_c',       'Proposed (comp)',     '#2ca02c'),
]
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


def load(stem):
    S = loadmat(os.path.join(OUT, stem + '.mat'))
    D, t = S['D'], S['t'].ravel()
    ch, M = load_vs(os.path.join(OUT, stem + '.vs'))
    tv = M[:, ch.index('T_Record')]
    return D, t, ch, M, tv


def main():
    fig, axes = plt.subplots(3, 2, figsize=(13, 11))
    ax_ey, ax_sw = axes[0]
    ax_mz, ax_md = axes[1]
    ax_beta, ax_xy = axes[2]

    rows = []
    for stem, lab, col in CASES:
        D, t, ch, M, tv = load(stem)
        c = lambda n: M[:, ch.index(n)]
        w = t >= T_STAT
        ax_ey.plot(t, D[0, :], color=col, lw=1.7, label=lab)
        ax_sw.plot(t, D[18, :], color=col, lw=1.4, label=lab)
        ax_mz.plot(t, D[17, :], color=col, lw=1.4, label=lab)
        ax_md.plot(t, D[5, :], color=col, lw=1.4, label=lab)
        ax_beta.plot(tv, c('Beta'), color=col, lw=1.4, label=lab)
        ax_xy.plot(c('Xo'), c('Yo'), color=col, lw=1.6, label=lab)
        rows.append((lab,
                     np.nanmax(np.abs(D[0, w])),
                     np.sqrt(np.nanmean(D[0, w] ** 2)),
                     np.nanmax(np.abs(D[18, w])),
                     np.nanmax(np.abs(D[17, w])),
                     np.nanmax(np.abs(D[5, w])),
                     np.nanmax(np.abs(c('LTR')))))

    ax_ey.set_ylabel(r'$e_y$ (m)')
    ax_ey.set_title('(a) Lateral error', fontsize=10)
    ax_sw.set_ylabel('SW (deg)')
    ax_sw.set_title('(b) Steering wheel', fontsize=10)
    ax_mz.set_ylabel(r'$M_z$ (N$\cdot$m)')
    ax_mz.set_title('(c) Differential-braking yaw moment', fontsize=10)
    ax_md.set_ylabel(r'$M_d$ (N$\cdot$m)')
    ax_md.set_title('(d) Damper roll moment', fontsize=10)
    ax_beta.set_ylabel(r'$\beta$ (deg)')
    ax_beta.set_title('(e) Sideslip angle', fontsize=10)
    ax_xy.set_ylabel('Y (m)')
    ax_xy.set_xlabel('X (m)')
    ax_xy.set_title('(f) Trajectory', fontsize=10)

    for a in (ax_ey, ax_sw, ax_mz, ax_md, ax_beta):
        a.grid(alpha=0.3)
        a.set_xlabel('time (s)')
        a.legend(fontsize=8, loc='best')
    ax_xy.grid(alpha=0.3)
    ax_xy.legend(fontsize=8, loc='best')
    ax_xy.set_aspect('equal', adjustable='datalim')

    fig.suptitle('Slalom 80 km/h, $\\mu$=0.85, A=1.2 m, damper force $\\times$3\n'
                 'LTR 0.74-0.78 (no wheel lift); ZENG uses the paper rho_I0=0.7',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'slalom_80_x3.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('%-22s %7s %7s %7s %7s %7s %7s' %
          ('controller', 'ey_max', 'ey_rms', 'SW', 'Mz', 'Md', 'LTR'))
    for r in rows:
        print('%-22s %7.3f %7.3f %7.1f %7.0f %7.0f %7.3f' % r)
    print('saved ->', os.path.join(OUT, 'slalom_80_x3.pdf'))


if __name__ == '__main__':
    main()
