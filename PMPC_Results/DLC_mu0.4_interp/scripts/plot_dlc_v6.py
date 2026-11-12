# -*- coding: utf-8 -*-
"""DLC mu=0.4 定稿（V6 口径，b98835a 代码 + case1 参数）：四控制器 × 60/70/75。

数据前缀 v6_*，全部由当前仓库代码（b98835a + pmpc_init case 1 的
mu=0.4 / slack_w=0.2e4 / ZENG rho_I0=1.0 / Proposed Q_epsi=5,Wb=10）重跑。

输出（只存 PDF）：
  v6_four_controllers_<sp>.pdf   每个速度 6 面板（ey/SW/Mz/Md/beta/XY）
  v6_summary_ey.pdf              ey max/RMS 汇总
  v6_metrics.csv

用法：python plot_dlc_v6.py [结果目录]
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

SPEEDS = [60, 70, 75]
CTRLS = [(1, 'MPC',                '#1f77b4', 'v6_c1_%d'),
         (2, 'ZENG',               '#d62728', 'v6_c2_%d'),
         (3, 'Proposed (no comp)', '#9467bd', 'v6_c3nc_%d'),
         (3, 'Proposed (comp)',    '#2ca02c', 'v6_c3c_%d')]
# 定稿只展示三控制器。主对比的 Proposed 用【含补偿】的完整方法，
# no-comp 只作为消融对照（两者在 DLC 里差异 <1%，见 CONTROLLERS.md 第 23 节）。
CTRLS3 = [(1, 'MPC',      '#1f77b4', 'v6_c1_%d'),
          (2, 'ZENG',     '#d62728', 'v6_c2_%d'),
          (3, 'Proposed', '#2ca02c', 'v6_c3c_%d')]


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
            beta = c('Beta')
            axes[0].plot(t, ey, color=col, lw=1.7, label=name)
            axes[1].plot(t, sw, color=col, lw=1.4, label=name)
            axes[2].plot(t, mz, color=col, lw=1.4, label=name)
            axes[3].plot(t, md, color=col, lw=1.4, label=name)
            axes[4].plot(tv, beta, color=col, lw=1.4, label=name)
            axes[5].plot(c('Xo'), c('Yo'), color=col, lw=1.6, label=name)
            rows.append(dict(
                speed=sp, controller=name, stem=stem, t_end=float(t[-1]),
                ey_max=float(np.nanmax(np.abs(ey))),
                ey_rms=float(np.sqrt(np.nanmean(ey ** 2))),
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
        axes[0].set_title('DLC %d km/h, mu=0.4 (controller and CarSim)' % sp)
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend(loc='best', fontsize=9)
        axes[4].set_xlabel('time (s)')
        axes[5].set_aspect('equal', adjustable='datalim')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, 'v6_four_controllers_%d.pdf' % sp),
                    bbox_inches='tight')
        plt.close(fig)

    # 定稿图：只画 MPC / ZENG / Proposed（no-comp）
    for sp in SPEEDS:
        fig, axes = plt.subplots(6, 1, figsize=(10, 15))
        for cid, name, col, pat in CTRLS3:
            D, t, ch, M, tv = load(pat % sp)
            c = lambda n: M[:, ch.index(n)]
            axes[0].plot(t, D[0, :], color=col, lw=1.7, label=name)
            axes[1].plot(t, D[18, :], color=col, lw=1.4, label=name)
            axes[2].plot(t, D[17, :], color=col, lw=1.4, label=name)
            axes[3].plot(t, D[5, :], color=col, lw=1.4, label=name)
            axes[4].plot(tv, c('Beta'), color=col, lw=1.4, label=name)
            axes[5].plot(c('Xo'), c('Yo'), color=col, lw=1.6, label=name)
        axes[0].set_ylabel(r'$e_y$ (m)')
        axes[1].set_ylabel('SW (deg)')
        axes[2].set_ylabel(r'$M_z$ (N$\cdot$m)')
        axes[3].set_ylabel(r'$M_d$ (N$\cdot$m)')
        axes[4].set_ylabel(r'$\beta$ (deg)')
        axes[5].set_ylabel('Y (m)')
        axes[5].set_xlabel('X (m)')
        axes[0].set_title('DLC %d km/h, mu=0.4 (controller and CarSim)' % sp)
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend(loc='best', fontsize=9)
        axes[4].set_xlabel('time (s)')
        axes[5].set_aspect('equal', adjustable='datalim')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, 'v6_three_controllers_%d.pdf' % sp),
                    bbox_inches='tight')
        plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    x = np.arange(len(SPEEDS))
    w = 0.25
    for k, (cid, name, col, pat) in enumerate(CTRLS3):
        pick = [next(r for r in rows if r['stem'] == pat % sp) for sp in SPEEDS]
        mx = [r['ey_max'] for r in pick]
        rm = [r['ey_rms'] for r in pick]
        axes[0].bar(x + (k - 1.0) * w, mx, w, color=col, label=name)
        axes[1].bar(x + (k - 1.0) * w, rm, w, color=col, label=name)
    for ax, title in zip(axes, ['$e_y$ max (m)', '$e_y$ RMS (m)']):
        ax.set_xticks(x)
        ax.set_xticklabels(['%d km/h' % s for s in SPEEDS])
        ax.set_title(title)
        ax.grid(alpha=0.3, axis='y')
        ax.legend(fontsize=8)
    fig.suptitle('DLC mu=0.4 — MPC / ZENG / Proposed (with delay compensation)',
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'v6_summary_ey.pdf'), bbox_inches='tight')
    plt.close(fig)

    with open(os.path.join(OUT, 'v6_metrics.csv'), 'w', newline='', encoding='utf-8') as f:
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
