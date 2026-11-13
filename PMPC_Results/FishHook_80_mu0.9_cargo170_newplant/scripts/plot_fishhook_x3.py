# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 190 kg —— 减振器力 x3 后的定稿三曲线。

被动基线是"恒定 1 A"（中等阻尼）：inv_current_impl_10ms.m 里
SH_MODE=2、SH_I=1.0 时跑出。

  曲线                     文件            roll max / LTR max
  Passive (1 A)            p1a_190         79.65 deg @4.18 s / 1.00  （翻车）
  MPC (Q_norm(3)=15)       x3m_q15          5.75 deg / 1.00
  Proposed (Q_norm(3)=40)  x3t_q40nc        5.10 deg / 0.81

输出：fishhook_190_x3.pdf（只出 PDF）
用法：python plot_fishhook_x3.py [结果目录]
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
    ('p1a_190',   'Passive (1 A)',        '#7f7f7f'),
    ('x3m_q15',   'MPC',                  '#d62728'),
    ('x3t_q40nc', 'Proposed',             '#1f77b4'),
]


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


def load_case(stem):
    S = loadmat(os.path.join(OUT, stem + '.mat'))
    D, t = S['D'], S['t'].ravel()
    ch, M = load_vs(os.path.join(OUT, stem + '.vs'))
    tv = M[:, ch.index('T_Record')]
    return D, t, ch, M, tv


def main():
    data = {stem: load_case(stem) for stem, *_ in CASES}

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax_roll, ax_ltr = axes[0]
    ax_md, ax_mz = axes[1]

    rows = []
    for stem, lab, col in CASES:
        D, t, ch, M, tv = data[stem]
        c = lambda n: M[:, ch.index(n)]
        roll, ltr = c('Roll'), c('LTR')
        ax_roll.plot(tv, roll, color=col, lw=1.8, label=lab)
        ax_ltr.plot(tv, ltr, color=col, lw=1.8, label=lab)
        ax_md.plot(t, D[5, :], color=col, lw=1.4, label=lab + ' actual')
        ax_md.plot(t, D[6, :], color=col, lw=0.8, ls='--', alpha=0.6)
        ax_mz.plot(t, D[17, :], color=col, lw=1.4, label=lab)
        rows.append((lab, np.nanmax(np.abs(roll)), np.nanmax(np.abs(ltr)),
                     np.nanmax(np.abs(D[5, :])), np.nanmax(np.abs(D[17, :]))))

    ax_roll.set_ylabel('roll (deg)')
    ax_roll.set_title('(a) Roll angle', fontsize=10)
    ax_ltr.set_ylabel('LTR (left $-$ right)')
    ax_ltr.set_title('(b) Lateral load transfer ratio', fontsize=10)
    ax_ltr.axhline(1, color='k', ls=':', lw=0.9)
    ax_ltr.axhline(-1, color='k', ls=':', lw=0.9)
    ax_md.set_ylabel(r'$M_d$ (N$\cdot$m)')
    ax_md.set_title(r'(c) Damper roll moment: actual (solid) / command (dashed)', fontsize=10)
    ax_mz.set_ylabel(r'$M_z$ (N$\cdot$m)')
    ax_mz.set_title('(d) Differential-braking yaw moment', fontsize=10)

    for a in (ax_roll, ax_ltr, ax_md, ax_mz):
        a.grid(alpha=0.3)
        a.set_xlim(0, 5.0)
        a.set_xlabel('time (s)')
        a.legend(fontsize=7, loc='best')

    ax_roll.annotate('Passive rolls over\nat 4.18 s', xy=(4.18, -55), xytext=(2.6, -45),
                     fontsize=8, color='#7f7f7f',
                     arrowprops=dict(arrowstyle='->', color='#7f7f7f', lw=0.8))

    fig.suptitle('FishHook 80 km/h, $\\mu$=0.9, cargo 190 kg, damper force $\\times$3\n'
                 'Passive (1 A) rolls over; MPC LTR = 1.00; Proposed LTR = 0.81',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_190_x3.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('%-18s %8s %7s %8s %8s' % ('controller', 'roll', 'LTR', 'Md_act', 'Mz'))
    for r in rows:
        print('%-18s %8.3f %7.3f %8.0f %8.0f' % r)
    print('saved ->', os.path.join(OUT, 'fishhook_190_x3.pdf'))


if __name__ == '__main__':
    main()
