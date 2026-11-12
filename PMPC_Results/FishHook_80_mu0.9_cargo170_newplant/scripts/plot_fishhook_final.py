# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 190 kg / 1 s 预备段：定稿四控制器对比。

数据全部由 b98835a（HEAD）的代码产生：
  fin190_passive  Passive
  fh_c1           MPC baseline
  fh_c2           ZENG 2025
  fh_c3_nc        Proposed 无补偿
  fh_c3_c         Proposed + 25 ms 补偿

输出：fishhook_190_final.pdf（只出 PDF）
用法：python plot_fishhook_final.py [结果目录]
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
    ('fin190_passive', 'Passive (0 A)',          '#7f7f7f'),
    ('fh_c1',          'MPC baseline',           '#d62728'),
    ('fh_c3_nc',       'Proposed (no comp)',     '#1f77b4'),
    ('fh_c3_c',        'Proposed + comp 25 ms',  '#2ca02c'),
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
    tv = np.linspace(t[0], t[-1], M.shape[0])
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
        ax_roll.plot(tv, roll, color=col, lw=1.7, label=lab)
        ax_ltr.plot(tv, ltr, color=col, lw=1.7, label=lab)
        ax_md.plot(t, D[5, :], color=col, lw=1.3, label=lab + ' actual')
        ax_md.plot(t, D[6, :], color=col, lw=0.8, ls='--', alpha=0.6)
        ax_mz.plot(t, D[17, :], color=col, lw=1.3, label=lab)
        rows.append((lab, t[-1], np.nanmax(np.abs(roll)), np.nanmax(np.abs(ltr)),
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
    ax_mz.set_title('(d) Differential-braking yaw moment (same QP for all)', fontsize=10)

    for a in (ax_roll, ax_ltr, ax_md, ax_mz):
        a.grid(alpha=0.3)
        a.set_xlim(0, 5.0)
        a.set_xlabel('time (s)')
        a.legend(fontsize=7, loc='best')

    ax_roll.annotate('Passive rolls over\nat 3.44 s', xy=(3.44, -50), xytext=(3.62, 6),
                     fontsize=8, color='#7f7f7f', ha='left', va='top',
                     arrowprops=dict(arrowstyle='->', color='#7f7f7f', lw=0.8))

    fig.suptitle('FishHook 80 km/h, $\\mu$=0.9, roof cargo 190 kg, 1 s preparation — four controllers\n'
                 'Passive rolls over; MPC / Proposed all survive', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_190_final.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('%-24s %6s %8s %7s %8s %8s' %
          ('controller', 't_end', 'roll', 'LTR', 'Md_act', 'Mz'))
    for r in rows:
        print('%-24s %6.2f %8.3f %7.3f %8.0f %8.0f' % r)
    print('saved ->', os.path.join(OUT, 'fishhook_190_final.pdf'))


if __name__ == '__main__':
    main()
