# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 170 kg：三控制器 LTR 对比 + Md 补偿时序。

输出：
  fishhook_170_ltr_compare.png/.pdf
    (a) LTR 全程
    (b) LTR 峰值窗口放大
    (c) Md 指令/实测放大（标出补偿提前量）
    (d) 各阈值下 Md 指令首次到达时刻（no-comp vs comp25）

用法：
  python plot_fishhook_ltr.py [结果目录]
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
    ('fh170_prep1_nocomp_c1', 'MPC baseline', '#d62728', '-'),
    ('fh170_prep1_nocomp_c3', 'Proposed no-comp', '#1f77b4', '-'),
    ('fh170_prep1_comp25_c3', 'Proposed comp 25 ms', '#2ca02c', '-'),
]
THRESH = [500, 1000, 1500, 2000, 2500, 3000]


def load_vs(vs_path):
    raw = open(vs_path, 'rb').read().decode('utf-8', errors='ignore')
    depth, in_str, esc, end = 0, False, False, None
    for i, ch in enumerate(raw):
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
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


def load_case(stem):
    S = loadmat(os.path.join(OUT, stem + '.mat'))
    D, t = S['D'], S['t'].ravel()
    ch, M = load_vs(os.path.join(OUT, stem + '.vs'))
    tv = np.linspace(t[0], t[-1], M.shape[0])
    return D, t, ch, M, tv


def first_cross(t, x, thr, sign=+1):
    idx = np.where(sign * x >= thr)[0]
    return t[idx[0]] if len(idx) else np.nan


def main():
    data = {stem: load_case(stem) for stem, *_ in CASES}

    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.22)
    ax_ltr = fig.add_subplot(gs[0, 0])
    ax_zoom = fig.add_subplot(gs[0, 1])
    ax_md = fig.add_subplot(gs[1, 0])
    ax_thr = fig.add_subplot(gs[1, 1])

    # (a) LTR 全程
    for stem, lab, col, ls in CASES:
        D, t, ch, M, tv = data[stem]
        ltr = M[:, ch.index('LTR')]
        ax_ltr.plot(tv, ltr, color=col, lw=1.8, ls=ls, label=lab)
    ax_ltr.axhline(1, color='k', ls=':', lw=0.8)
    ax_ltr.axhline(-1, color='k', ls=':', lw=0.8)
    ax_ltr.axhline(0.9, color='gray', ls='--', lw=0.7)
    ax_ltr.axhline(-0.9, color='gray', ls='--', lw=0.7)
    ax_ltr.axvspan(2.2, 3.6, color='0.9', zorder=0)
    ax_ltr.set_xlim(0, 4.0)
    ax_ltr.set_ylim(-1.05, 1.05)
    ax_ltr.set_xlabel('time (s)')
    ax_ltr.set_ylabel('LTR (right $-$ left)')
    ax_ltr.set_title('(a) LTR, full run (baseline ends at 2.62 s rollover)', fontsize=10)
    ax_ltr.grid(alpha=0.3)
    ax_ltr.legend(fontsize=8, loc='lower right')
    ax_ltr.annotate('MPC baseline\nrollover 2.62 s', xy=(2.61, -0.2), xytext=(2.9, 0.35),
                    fontsize=8, color='#d62728',
                    arrowprops=dict(arrowstyle='->', color='#d62728', lw=0.8))

    # (b) LTR 峰值窗口
    for stem, lab, col, ls in CASES:
        D, t, ch, M, tv = data[stem]
        ltr = M[:, ch.index('LTR')]
        m = (tv >= 2.2) & (tv <= 3.6)
        ax_zoom.plot(tv[m], ltr[m], color=col, lw=2.0, ls=ls, label=lab)
    ax_zoom.axhline(-1, color='k', ls=':', lw=0.8)
    ax_zoom.axhline(-0.9, color='gray', ls='--', lw=0.7)
    ax_zoom.set_xlim(2.2, 3.6)
    ax_zoom.set_xlabel('time (s)')
    ax_zoom.set_ylabel('LTR')
    ax_zoom.set_title('(b) LTR, critical window (2.2-3.6 s)', fontsize=10)
    ax_zoom.grid(alpha=0.3)
    ax_zoom.legend(fontsize=8, loc='lower right')
    ax_zoom.text(1.02, 0.55, 'rebound peak: baseline 0.81\nProposed 0.71 (both variants)',
                 fontsize=8, va='top')

    # (c) Md 指令 / 实测（放大）
    for stem, lab, col, ls in CASES:
        D, t, ch, M, tv = data[stem]
        m = (t >= 2.0) & (t <= 2.35)
        ax_md.plot(t[m], D[6, m], color=col, lw=1.8, ls='-', label=lab + ' cmd')
        ax_md.plot(t[m], D[5, m], color=col, lw=1.0, ls=':', alpha=0.8)
    # 标出 no-comp / comp25 在 |Md_cmd|>=2000、>=2500 的到达时刻
    Dn, tn = data['fh170_prep1_nocomp_c3'][0], data['fh170_prep1_nocomp_c3'][1]
    Dc, tc = data['fh170_prep1_comp25_c3'][0], data['fh170_prep1_comp25_c3'][1]
    for thr, ytxt in [(2000, -3050), (2500, -3350)]:
        a = first_cross(tn, Dn[6, :], thr, -1)
        b = first_cross(tc, Dc[6, :], thr, -1)
        ax_md.axvline(a, color='#1f77b4', ls='--', lw=0.9)
        ax_md.axvline(b, color='#2ca02c', ls='--', lw=0.9)
        ax_md.annotate('', xy=(a, ytxt), xytext=(b, ytxt),
                       arrowprops=dict(arrowstyle='<-', color='k', lw=0.9))
        ax_md.text((a + b) / 2, ytxt + 120, '%.0f ms' % ((b - a) * 1000),
                   ha='center', fontsize=8)
    ax_md.set_xlim(2.0, 2.35)
    ax_md.set_xlabel('time (s)')
    ax_md.set_ylabel(r'$M_d$ (N$\cdot$m)')
    ax_md.set_title(r'(c) $M_d$ command (solid) / actual (dotted), zoom', fontsize=10)
    ax_md.grid(alpha=0.3)
    ax_md.legend(fontsize=7, loc='lower left', ncol=2)
    ax_md.text(0.02, 0.97,
               'comp25 command leads by 10-20 ms\nactual $M_d$ unchanged (velocity-dominated)',
               transform=ax_md.transAxes, fontsize=8, va='top')

    # (d) 阈值到达时刻
    y = np.arange(len(THRESH))
    dt = []
    for thr in THRESH:
        a = first_cross(tn, Dn[6, :], thr, -1)
        b = first_cross(tc, Dc[6, :], thr, -1)
        dt.append((b - a) * 1000)
    ax_thr.barh(y, dt, color='#2ca02c', height=0.5)
    ax_thr.set_yticks(y)
    ax_thr.set_yticklabels(['%d' % v for v in THRESH])
    ax_thr.axvline(0, color='k', lw=0.8)
    ax_thr.set_xlabel(r'$\Delta t$ = comp25 $-$ no-comp (ms)   negative = earlier')
    ax_thr.set_ylabel(r'$|M_d^{\rm cmd}|$ threshold (N$\cdot$m)')
    ax_thr.set_title('(d) Command reaches threshold earlier with compensation', fontsize=10)
    ax_thr.set_xlim(-30, 5)
    ax_thr.grid(alpha=0.3, axis='x')
    for yi, v in zip(y, dt):
        ax_thr.text(v - 0.8, yi, '%+.0f' % v, va='center', ha='right', fontsize=8)

    fig.suptitle('FishHook 80 km/h, mu=0.9, roof cargo 170 kg — LTR and $M_d$ timing\n'
                 'no-comp: $\\tau_d$=0.1 ms, preview off   |   comp25: $\\tau_d$=25 ms, preview 25 ms',
                 fontsize=12)
    fig.savefig(os.path.join(OUT, 'fishhook_170_ltr_compare.png'), dpi=200, bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fishhook_170_ltr_compare.pdf'), bbox_inches='tight')
    plt.close(fig)

    for thr, v in zip(THRESH, dt):
        print('|Md_cmd| >= %4d : comp25 - no-comp = %+.0f ms' % (thr, v))
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
