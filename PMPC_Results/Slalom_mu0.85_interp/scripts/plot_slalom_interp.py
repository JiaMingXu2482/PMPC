# -*- coding: utf-8 -*-
"""Slalom mu=0.85，力界插值逆模型：四控制器（MPC/ZENG/Proposed 无补偿/有补偿）× 60/80/90 km/h。

输出：
  four_controllers_<sp>.pdf         每个速度 6 面板（ey/SW/Mz/Md/beta/XY）
  summary_ey.pdf                    ey max/RMS 汇总
  metrics.csv

用法：python plot_slalom_interp.py [结果目录]
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
# (cid, 图例名, 颜色, 文件前缀模板)
CTRLS = [(1, 'MPC',                 '#1f77b4', 'sla_c1_%d'),
         (2, 'ZENG',                '#d62728', 'sla_c2_%d'),
         (3, 'Proposed (no comp)',  '#9467bd', 't_nc_%d'),
         (3, 'Proposed (comp)',     '#2ca02c', 'sla_c3_%d')]


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
            ey, sw, mz, md = D[0, :], D[18, :], D[17, :], D[5, :]
            # 前 2 s 是初始投影瞬态（车辆 CG 在路径起点之后），不参与跟踪指标
            msk = t >= 2.0
            beta = c('Beta')
            axes[0].plot(t, ey, color=col, lw=1.7, label=name)
            axes[1].plot(t, sw, color=col, lw=1.4, label=name)
            axes[2].plot(t, mz, color=col, lw=1.4, label=name)
            axes[3].plot(t, md, color=col, lw=1.4, label=name)
            axes[4].plot(tv, beta, color=col, lw=1.4, label=name)
            axes[5].plot(c('Xo'), c('Yo'), color=col, lw=1.6, label=name)
            rows.append(dict(
                speed=sp, controller=name, t_end=float(t[-1]),
                ey_max=float(np.nanmax(np.abs(ey[msk]))),
                ey_rms=float(np.sqrt(np.nanmean(ey[msk] ** 2))),
                ey_final=float(ey[-1]),
                sw_max=float(np.nanmax(np.abs(sw))),
                mz_max=float(np.nanmax(np.abs(mz))),
                md_max=float(np.nanmax(np.abs(md))),
                md_mean=float(np.nanmean(np.abs(md))),
                beta_max=float(np.nanmax(np.abs(beta))),
                yaw_max=float(np.nanmax(np.abs(c('AVz')))),
                ltr_max=float(np.nanmax(np.abs(D[4, :]))),
                qp=float(100 * np.mean(D[23, :] > 0.5))))
        axes[0].set_ylabel(r'$e_y$ (m)')
        axes[1].set_ylabel('SW (deg)')
        axes[2].set_ylabel(r'$M_z$ (N$\cdot$m)')
        axes[3].set_ylabel(r'$M_d$ (N$\cdot$m)')
        axes[4].set_ylabel(r'$\beta$ (deg)')
        axes[5].set_ylabel('Y (m)')
        axes[5].set_xlabel('X (m)')
        axes[0].set_title('Slalom %d km/h, mu=0.85, A=1.2 m, interpolated inverse' % sp)
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend(loc='best', fontsize=9)
        axes[4].set_xlabel('time (s)')
        axes[5].set_aspect('equal', adjustable='datalim')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, 'four_controllers_%d.pdf' % sp), bbox_inches='tight')
        plt.close(fig)

    # 汇总
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    x = np.arange(len(SPEEDS))
    w = 0.2
    for k, (cid, name, col, pat) in enumerate(CTRLS):
        mx = [r['ey_max'] for r in rows if r['controller'] == name]
        rm = [r['ey_rms'] for r in rows if r['controller'] == name]
        axes[0].bar(x + (k - 1.5) * w, mx, w, color=col, label=name)
        axes[1].bar(x + (k - 1.5) * w, rm, w, color=col, label=name)
    for ax, title in zip(axes, ['$e_y$ max (m)', '$e_y$ RMS (m)']):
        ax.set_xticks(x)
        ax.set_xticklabels(['%d km/h' % s for s in SPEEDS])
        ax.set_title(title)
        ax.grid(alpha=0.3, axis='y')
        ax.legend(fontsize=8)
    fig.suptitle('Slalom mu=0.85, A=1.2 m, interpolated damper inverse', fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'summary_ey.pdf'), bbox_inches='tight')
    plt.close(fig)

    with open(os.path.join(OUT, 'metrics.csv'), 'w', newline='', encoding='utf-8') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    print('%-6s %-9s %7s %7s %7s %7s %7s %7s %6s' % (
        'speed', 'ctrl', 'ey_max', 'ey_rms', 'SW_max', 'Mz_max', 'Md_max', 'beta', 'QP'))
    for r in rows:
        print('%6d %-9s %7.3f %7.3f %7.1f %7.0f %7.0f %7.2f %5.0f%%' % (
            r['speed'], r['controller'], r['ey_max'], r['ey_rms'], r['sw_max'],
            r['mz_max'], r['md_max'], r['beta_max'], r['qp']))
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
