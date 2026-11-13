# -*- coding: utf-8 -*-
"""DLC 80 km/h / mu=0.4 / 减振器力 x3 —— 三控制器单图（ey / epsi / beta-r）。

三个控制器统一 Q_norm(5)=50（ey 权重减半），其余按各自整定：
  MPC    q50_mpc_80    Q_norm=[0;0;10;0;50;20;0], slack_w=5e4
  ZENG   q50_zeng_80   Q_norm=[0;0;10;0;50;10;0], slack_w=0.2e4, rho_I0=1.0
  Prop   p_w10_80      Q_norm=[0;0;10;0;50;10;0], w1_max=w2_max=0.1e4, Wb=10

输出（只存 PDF）：
  dlc80_ey.pdf       横向偏差 ey
  dlc80_epsi.pdf     航向偏差 epsi
  dlc80_beta_r.pdf   质心侧偏角 - 横摆角速度相平面

用法：python plot_dlc80_three.py [结果目录]
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
    ('q50_mpc_80',  'MPC',      '#1f77b4'),
    ('q50_zeng_80', 'ZENG',     '#d62728'),
    ('p_w10_80',    'Proposed', '#2ca02c'),
]

plt.rcParams.update({
    'font.size': 9, 'axes.grid': True, 'grid.alpha': 0.3,
    'axes.labelsize': 10, 'legend.fontsize': 8,
    'figure.dpi': 150, 'savefig.dpi': 300,
})


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
    c = lambda n: M[:, ch.index(n)]
    return D, t, c('Beta'), c('AVz'), c('T_Record')


def main():
    data = {stem: load(stem) for stem, *_ in CASES}

    # ---- (1) ey ----
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    for stem, lab, col in CASES:
        D, t, *_ = data[stem]
        ax.plot(t, D[0, :], color=col, lw=1.6, label=lab)
    ax.axhline(0, color='k', lw=0.4)
    ax.set_xlabel('time (s)')
    ax.set_ylabel(r'$e_y$ (m)')
    ax.set_xlim(0, 8.5)
    ax.legend(loc='lower left')
    ax.set_title('DLC 80 km/h, $\\mu$=0.4: lateral error', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'dlc80_ey.pdf'), bbox_inches='tight')
    plt.close(fig)

    # ---- (2) epsi ----
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    for stem, lab, col in CASES:
        D, t, *_ = data[stem]
        ax.plot(t, np.rad2deg(D[1, :]), color=col, lw=1.6, label=lab)
    ax.axhline(0, color='k', lw=0.4)
    ax.set_xlabel('time (s)')
    ax.set_ylabel(r'$\varepsilon_\psi$ (deg)')
    ax.set_xlim(0, 8.5)
    ax.legend(loc='lower left')
    ax.set_title('DLC 80 km/h, $\\mu$=0.4: heading error', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'dlc80_epsi.pdf'), bbox_inches='tight')
    plt.close(fig)

    # ---- (3) beta - r 相平面 ----
    fig, ax = plt.subplots(figsize=(5.4, 4.6))
    for stem, lab, col in CASES:
        D, t, beta, avz, tv = data[stem]
        ax.plot(beta, avz, color=col, lw=1.5, label=lab)
        ax.plot(beta[0], avz[0], 'o', color=col, ms=5, mfc='w', mew=1.2)
        ax.plot(beta[-1], avz[-1], 's', color=col, ms=5)
    ax.axhline(0, color='k', lw=0.4)
    ax.axvline(0, color='k', lw=0.4)
    ax.set_xlabel(r'$\beta$ (deg)')
    ax.set_ylabel(r'$r$ (deg/s)')
    ax.legend(loc='best')
    ax.set_title('DLC 80 km/h, $\\mu$=0.4: $\\beta$-$r$ phase plane\n'
                 'circle = start, square = end', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'dlc80_beta_r.pdf'), bbox_inches='tight')
    plt.close(fig)

    # ---- 控制台汇总 ----
    print('%-10s %8s %8s %8s %8s %8s' %
          ('ctrl', 'ey_max', 'ey_rms', 'epsi_max', 'beta_max', 'r_max'))
    for stem, lab, col in CASES:
        D, t, beta, avz, tv = data[stem]
        w = t >= 2.0
        print('%-10s %8.3f %8.3f %8.2f %8.2f %8.2f' % (
            lab, np.nanmax(np.abs(D[0, w])), np.sqrt(np.nanmean(D[0, w] ** 2)),
            np.nanmax(np.abs(np.rad2deg(D[1, w]))),
            np.nanmax(np.abs(beta)), np.nanmax(np.abs(avz))))
    print('saved -> dlc80_ey.pdf / dlc80_epsi.pdf / dlc80_beta_r.pdf')


if __name__ == '__main__':
    main()
