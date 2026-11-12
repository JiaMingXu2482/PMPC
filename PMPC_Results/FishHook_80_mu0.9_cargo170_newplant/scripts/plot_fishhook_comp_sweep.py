# -*- coding: utf-8 -*-
"""FishHook F5 口径：Proposed comp 的补偿参数扫描（vs no-comp）。

基准：cargo 190 kg，80 km/h，mu=0.9，1 s 预备段，力界插值逆模型。
no-comp : tau_d=1e-4, preview_L=0
comp25  : tau_d=0.025, preview_L=0.025, preview_alpha=1  （原定稿）
扫描    : preview_L ∈ {0.04, 0.05}；tau_d ∈ {0.05, 0.10, 0.20}；组合 L=0.05+td=0.10

输出：fishhook_190_comp_sweep.pdf（只出 PDF）
用法：python plot_fishhook_comp_sweep.py [结果目录]
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
    ('r3_50_nc',    'no-comp',                 '#000000'),
    ('r3_50_c',     'comp  L=25 td=25 (base)', '#1f77b4'),
    ('fc_L040',     'comp  L=40 td=25',        '#2ca02c'),
    ('fc_td200',    'comp  L=25 td=200',       '#d62728'),
    ('fc_L050td100', 'comp  L=50 td=100',      '#9467bd'),
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
    ax_md, ax_mdc = axes[1]

    rows = []
    for stem, lab, col in CASES:
        D, t, ch, M, tv = data[stem]
        c = lambda n: M[:, ch.index(n)]
        roll, ltr = c('Roll'), c('LTR')
        ax_roll.plot(tv, roll, color=col, lw=1.6, label=lab)
        ax_ltr.plot(tv, ltr, color=col, lw=1.6, label=lab)
        ax_md.plot(t, D[5, :], color=col, lw=1.4, label=lab)
        ax_mdc.plot(t, D[6, :], color=col, lw=1.4, label=lab)
        rows.append((lab, np.nanmax(np.abs(roll)), np.nanmax(np.abs(ltr)),
                     np.nanmax(np.abs(D[5, :])), np.nanmax(np.abs(D[6, :])),
                     np.nanmax(np.abs(D[17, :]))))

    ax_roll.set_ylabel('roll (deg)')
    ax_roll.set_title('(a) Roll angle', fontsize=10)
    ax_ltr.set_ylabel('LTR (left $-$ right)')
    ax_ltr.set_title('(b) Lateral load transfer ratio', fontsize=10)
    ax_ltr.axhline(1, color='k', ls=':', lw=0.9)
    ax_ltr.axhline(-1, color='k', ls=':', lw=0.9)
    ax_md.set_ylabel(r'$M_d$ actual (N$\cdot$m)')
    ax_md.set_title(r'(c) Damper roll moment, actual', fontsize=10)
    ax_mdc.set_ylabel(r'$M_d$ command (N$\cdot$m)')
    ax_mdc.set_title(r'(d) Damper roll moment, command', fontsize=10)

    for a in (ax_roll, ax_ltr, ax_md, ax_mdc):
        a.grid(alpha=0.3)
        a.set_xlim(0, 5.0)
        a.set_xlabel('time (s)')
        a.legend(fontsize=7, loc='best')

    fig.suptitle('FishHook 80 km/h, $\\mu$=0.9, cargo 190 kg — Proposed comp parameter sweep\n'
                 'all variants collapse onto the same roll response', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_190_comp_sweep.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('%-26s %8s %7s %8s %8s %8s' %
          ('variant', 'roll', 'LTR', 'Md_act', 'Md_cmd', 'Mz'))
    for r in rows:
        print('%-26s %8.3f %7.3f %8.0f %8.0f %8.0f' % r)
    print('saved ->', os.path.join(OUT, 'fishhook_190_comp_sweep.pdf'))


if __name__ == '__main__':
    main()
