# -*- coding: utf-8 -*-
"""FishHook 190 kg：四控制器 × 四角减振器力，16 个子图。

行 = 控制器（Passive / MPC / Proposed / Proposed+comp）
列 = 四角（L1 / L2 / R1 / R2）
实线 = 实测 Fd，虚线 = 指令 Fdes。

输出：fishhook_170_damper_force.png/.pdf
用法：python plot_fishhook_damper_force.py [结果目录]
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)

CASES = [
    ('fin190_passive', 'Passive vehicle',        '#7f7f7f'),
    ('fh190_c1',       'MPC',                    '#d62728'),
    ('r3_50_nc',       'Proposed',               '#1f77b4'),
    ('r3_50_c',        'Proposed + comp 10 ms',  '#2ca02c'),
]
CORNERS = ['L1', 'L2', 'R1', 'R2']


def load(stem):
    S = loadmat(os.path.join(OUT, stem + '.mat'))
    return S['D'], S['t'].ravel()


def main():
    data = {stem: load(stem) for stem, *_ in CASES}

    # 每行独立 y 轴（被动车的力比受控车大 3 倍，统一刻度会压扁受控车细节）
    yrow = {}
    for stem, lab, col in CASES:
        D, t = data[stem]
        yrow[stem] = np.ceil(np.nanmax(np.abs(D[9:13, :])) / 1000) * 1000

    fig, axes = plt.subplots(4, 4, figsize=(18, 14), sharex=True)
    for r, (stem, lab, col) in enumerate(CASES):
        D, t = data[stem]
        for c, corner in enumerate(CORNERS):
            ax = axes[r, c]
            fd = D[9 + c, :]
            fdes = D[13 + c, :]
            ax.plot(t, fd, color=col, lw=1.3, label='Fd actual')
            ax.plot(t, fdes, color='k', lw=0.8, ls='--', alpha=0.6, label='Fdes cmd')
            ax.axhline(0, color='k', lw=0.5, ls=':')
            ax.grid(alpha=0.3)
            ax.set_xlim(0, 6.0)
            ax.set_ylim(-yrow[stem], yrow[stem])
            if r == 0:
                ax.set_title(corner, fontsize=12)
            if c == 0:
                ax.set_ylabel('%s\nforce (N)' % lab, fontsize=9)
            if r == 3:
                ax.set_xlabel('time (s)')
            if r == 0 and c == 0:
                ax.legend(fontsize=7, loc='upper right')
            ax.text(0.02, 0.03, 'max %.0f N' % np.nanmax(np.abs(fd)),
                    transform=ax.transAxes, fontsize=7, va='bottom')

    fig.suptitle('FishHook 80 km/h, mu=0.9, cargo 190 kg — damper force, four corners\n'
                 'solid = actual Fd, dashed = command Fdes', fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_170_damper_force.png'), dpi=200, bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fishhook_170_damper_force.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('per-row y range: ' + ', '.join('%s +-%.0f' % (l, yrow[s]) for s, l, _ in CASES))
    for stem, lab, col in CASES:
        D, t = data[stem]
        print('%-22s per-corner |Fd| max: %s' % (
            lab, ' '.join('%5.0f' % np.nanmax(np.abs(D[9 + k, :])) for k in range(4))))
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
