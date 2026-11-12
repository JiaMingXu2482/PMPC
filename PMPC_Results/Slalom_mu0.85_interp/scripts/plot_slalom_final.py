# -*- coding: utf-8 -*-
"""Slalom mu=0.85 定稿：四控制器 × 60/80/90（b98835a 代码 + case2 参数）。

case2 参数：A=1.2 / k=0.18 / mu=0.85 / 力界预瞄关闭；
ZENG rho_min=1.0（≡MPC）；Proposed w1_max=w2_max=50、Wb=10、tau_d=0.15（comp），
no-comp 由 run_slalom 覆盖为 tau_d=1e-4。

输出（只存 PDF）：
  sv_four_controllers_<sp>.pdf   每个速度 6 面板（ey/SW/Mz/Md/beta/XY）
  sv_summary_ey.pdf              ey max/RMS 汇总（t>=2 s）
  sv_metrics.csv

用法：python plot_slalom_final.py [结果目录]
"""
import csv
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

SPEEDS = [60, 80, 90]
# ZENG 在 Slalom 用 rho_min=1.0，与 MPC 逐点等价（数据完全重合），
# 画出来只是把蓝线盖住，故不列入。ZENG 的对比放在 DLC。
CTRLS = [(1, 'MPC',                 '#1f77b4', 'sv_c1_%d'),
         (3, 'Proposed (no comp)',  '#9467bd', 'sv_c3nc_%d'),
         (3, 'Proposed (comp)',     '#2ca02c', 'sv_c3c_%d')]
T_STAT = 2.0   # 跟踪误差统计起点 [s]


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
    tv = np.linspace(t[0], t[-1], M.shape[0])
    return D, t, ch, M, tv


def main():
    rows = []
    for sp in SPEEDS:
        fig, axes = plt.subplots(6, 1, figsize=(10, 15))
        for cid, name, col, pat in CTRLS:
            stem = pat % sp
            D, t, ch, M, tv = load(stem)
            c = lambda n: M[:, ch.index(n)]
            w = t >= T_STAT
            axes[0].plot(t, D[0, :], color=col, lw=1.7, label=name)
            axes[1].plot(t, D[18, :], color=col, lw=1.4, label=name)
            axes[2].plot(t, D[17, :], color=col, lw=1.4, label=name)
            axes[3].plot(t, D[5, :], color=col, lw=1.4, label=name)
            axes[4].plot(tv, c('Beta'), color=col, lw=1.4, label=name)
            axes[5].plot(c('Xo'), c('Yo'), color=col, lw=1.6, label=name)
            rows.append(dict(
                speed=sp, controller=name, t_end=float(t[-1]),
                ey_max=float(np.nanmax(np.abs(D[0, w]))),
                ey_rms=float(np.sqrt(np.nanmean(D[0, w] ** 2))),
                ey_final=float(D[0, -1]),
                sw_max=float(np.nanmax(np.abs(D[18, w]))),
                mz_max=float(np.nanmax(np.abs(D[17, w]))),
                md_max=float(np.nanmax(np.abs(D[5, w]))),
                md_mean=float(np.nanmean(np.abs(D[5, w]))),
                beta_max=float(np.nanmax(np.abs(c('Beta')))),
                yaw_max=float(np.nanmax(np.abs(c('AVz')))),
                ltr_max=float(np.nanmax(np.abs(D[4, w]))),
                qp=float(100 * np.mean(D[23, :] > 0.5))))
        axes[0].set_ylabel(r'$e_y$ (m)')
        axes[1].set_ylabel('SW (deg)')
        axes[2].set_ylabel(r'$M_z$ (N$\cdot$m)')
        axes[3].set_ylabel(r'$M_d$ (N$\cdot$m)')
        axes[4].set_ylabel(r'$\beta$ (deg)')
        axes[5].set_ylabel('Y (m)')
        axes[5].set_xlabel('X (m)')
        axes[0].set_title('Slalom %d km/h, mu=0.85' % sp)
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend(loc='best', fontsize=9)
        axes[4].set_xlabel('time (s)')
        axes[5].set_aspect('equal', adjustable='datalim')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, 'sv_four_controllers_%d.pdf' % sp),
                    bbox_inches='tight')
        plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    x = np.arange(len(SPEEDS))
    w = 0.25
    for k, (cid, name, col, pat) in enumerate(CTRLS):
        pick = [next(r for r in rows if r['speed'] == sp and r['controller'] == name)
                for sp in SPEEDS]
        axes[0].bar(x + (k - 1.0) * w, [r['ey_max'] for r in pick], w,
                    color=col, label=name)
        axes[1].bar(x + (k - 1.0) * w, [r['ey_rms'] for r in pick], w,
                    color=col, label=name)
    for ax, title in zip(axes, [r'$e_y$ max (m)', r'$e_y$ RMS (m)']):
        ax.set_xticks(x)
        ax.set_xticklabels(['%d km/h' % s for s in SPEEDS])
        ax.set_title(title)
        ax.grid(alpha=0.3, axis='y')
        ax.legend(fontsize=8)
    fig.suptitle('Slalom mu=0.85 — MPC / Proposed no-comp / Proposed comp '
                 '(t $\\geq$ %.0f s)' % T_STAT, fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'sv_summary_ey.pdf'), bbox_inches='tight')
    plt.close(fig)

    with open(os.path.join(OUT, 'sv_metrics.csv'), 'w', newline='', encoding='utf-8') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    print('%-6s %-22s %7s %7s %7s %7s %7s %7s %6s' % (
        'speed', 'ctrl', 'ey_max', 'ey_rms', 'SW_max', 'Mz_max', 'Md_max', 'beta', 'QP'))
    for r in rows:
        print('%6d %-22s %7.3f %7.3f %7.1f %7.0f %7.0f %7.2f %5.0f%%' % (
            r['speed'], r['controller'], r['ey_max'], r['ey_rms'], r['sw_max'],
            r['mz_max'], r['md_max'], r['beta_max'], r['qp']))
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
