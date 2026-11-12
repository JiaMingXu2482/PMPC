# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 190 kg / 1 s 预备段：四控制器对比（方案1 参数）。

  1. MPC baseline        (ctrl=1, Q_norm=[0;10;20;0;0;0;0], slack_w=[100;100;1e4;1e-3])
  2. ZENG                (ctrl=2, rho_min=1.0 -> 与 MPC 等价)
  3. Proposed no-comp    (ctrl=3, w2_max=100 / w1_max=1e4, tau_d=1e-4)
  4. Proposed comp25     (ctrl=3, 同上, tau_d=0.025)

输出：fishhook_190_w1_four.pdf（只出 PDF）
用法：python plot_fishhook_w1.py [结果目录]
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
    ('w1_mpc',  'MPC baseline',        '#d62728'),
    ('w1_zeng', 'ZENG (rho=1)',        '#ff7f0e'),
    ('w1_nc',   'Proposed (no comp)',  '#1f77b4'),
    ('w1_c',    'Proposed + comp 25 ms', '#2ca02c'),
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
    ax_md, ax_env = axes[1]

    rows = []
    for stem, lab, col in CASES:
        D, t, ch, M, tv = data[stem]
        c = lambda n: M[:, ch.index(n)]
        roll, ltr = c('Roll'), c('LTR')
        ax_roll.plot(tv, roll, color=col, lw=1.8, label=lab)
        ax_ltr.plot(tv, ltr, color=col, lw=1.8, label=lab)
        ax_md.plot(t, D[6, :], color=col, lw=1.5, label=lab + ' cmd')
        ax_env.plot(t, D[9, :] - D[8, :], color=col, lw=1.2, ls='--',
                    label=lab)
        rows.append((lab, t[-1], np.nanmax(np.abs(roll)),
                     np.nanmax(np.abs(ltr)), np.nanmax(np.abs(D[6, :])),
                     np.nanmax(np.abs(D[17, :]))))

    ax_roll.set_ylabel('roll (deg)')
    ax_roll.set_title('(a) Roll angle', fontsize=10)
    ax_ltr.set_ylabel('LTR (left $-$ right)')
    ax_ltr.set_title('(b) Lateral load transfer ratio', fontsize=10)
    ax_ltr.axhline(1, color='k', ls=':', lw=0.9)
    ax_ltr.axhline(-1, color='k', ls=':', lw=0.9)
    ax_md.set_ylabel(r'$M_d$ command (N$\cdot$m)')
    ax_md.set_title(r'(c) Damper roll-moment command', fontsize=10)
    ax_env.set_ylabel(r'$M_{d,max}-M_{d,min}$ (N$\cdot$m)')
    ax_env.set_title(r'(d) Available damper roll-moment authority', fontsize=10)

    for a in (ax_roll, ax_ltr, ax_md, ax_env):
        a.grid(alpha=0.3)
        a.set_xlim(0, 5.0)
        a.set_xlabel('time (s)')
        a.legend(fontsize=7, loc='best')

    ax_roll.annotate('peak roll at 1.67 s\nall four within 1%',
                     xy=(1.67, 6.8), xytext=(2.3, 5.2), fontsize=8,
                     arrowprops=dict(arrowstyle='->', lw=0.8))

    fig.suptitle('FishHook 80 km/h, $\\mu$=0.9, roof cargo 190 kg — four controllers\n'
                 'all pinned at the damper force envelope', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_190_w1_four.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('%-24s %6s %8s %7s %8s %8s' %
          ('controller', 't_end', 'roll', 'LTR', 'Md_cmd', 'Mz'))
    for r in rows:
        print('%-24s %6.2f %8.3f %7.3f %8.0f %8.0f' % r)
    print('saved ->', os.path.join(OUT, 'fishhook_190_w1_four.pdf'))


if __name__ == '__main__':
    main()
