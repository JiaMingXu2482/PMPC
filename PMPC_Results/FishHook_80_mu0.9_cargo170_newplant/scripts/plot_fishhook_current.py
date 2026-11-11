# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 170 kg：Proposed 有/无 25 ms 补偿的四角电流。

数据来自 pmpc_fishhook_log.slx（在逆模型输出加 To Workspace，变量 pmpc_Icmd），
由 run_fishhook 保存为 <prefix>.mat 里的 Icmd。

输出：
  fishhook_170_current_compare.png/.pdf
    上排 0-8 s 全程，下排 1.0-1.5 s 放大；四列 = L1/L2/R1/R2。

用法：
  python plot_fishhook_current.py [结果目录]
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
    ('fh170_prep1_log_nocomp_c3', 'no-comp', '#1f77b4'),
    ('fh170_prep1_log_comp25_c3', 'comp 25 ms', '#2ca02c'),
]
CORNERS = ['L1', 'L2', 'R1', 'R2']
I_MAX = 1.6


def get_i(stem):
    S = loadmat(os.path.join(OUT, stem + '.mat'))
    I = S['Icmd']
    t = np.asarray(I['time'][0, 0]).ravel().astype(float)
    V = np.asarray(I['signals'][0, 0]['values'][0, 0], dtype=float)
    return t, V


def main():
    data = {stem: get_i(stem) for stem, *_ in CASES}
    t = data[CASES[0][0]][0]

    fig, axes = plt.subplots(2, 4, figsize=(16, 6.4))   # 不共享 x：上排全程、下排放大
    for k, name in enumerate(CORNERS):
        for row, (tlo, thi, tag) in enumerate([(0, 8, 'full'), (2.0, 2.5, 'zoom')]):
            ax = axes[row, k]
            for stem, lab, col in CASES:
                tt, V = data[stem]
                m = (tt >= tlo) & (tt <= thi)
                ax.plot(tt[m], V[m, k], color=col, lw=1.4 if row == 0 else 1.8, label=lab)
            ax.axhline(I_MAX, color='k', ls=':', lw=0.8)
            ax.axhline(0, color='k', ls=':', lw=0.8)
            ax.set_ylim(-0.05, 1.75)
            ax.grid(alpha=0.3)
            if row == 0:
                ax.set_title('%s — full run' % name, fontsize=10)
                ax.set_xlim(0, 8)
            else:
                ax.set_title('%s — zoom 2.0-2.5 s' % name, fontsize=10)
                ax.set_xlim(2.0, 2.5)
                ax.set_xlabel('time (s)')
            if k == 0:
                ax.set_ylabel('damper current (A)')
            if row == 0 and k == 0:
                ax.legend(fontsize=8, loc='upper right')

    # 平均电流标注
    for k, name in enumerate(CORNERS):
        txt = []
        for stem, lab, col in CASES:
            _, V = data[stem]
            txt.append('%s mean %.2f A, max %.2f A' % (lab, V[:, k].mean(), V[:, k].max()))
        axes[0, k].text(0.02, 0.03, '\n'.join(txt), transform=axes[0, k].transAxes,
                        fontsize=7, va='bottom')

    fig.suptitle('FishHook 80 km/h, mu=0.9, roof cargo 170 kg — Proposed damper current, '
                 'no compensation vs 25 ms compensation (1 s preparation)', fontsize=12)
    for k in range(4):
        print('col %d xlim: top %s  bottom %s' % (
            k, axes[0, k].get_xlim(), axes[1, k].get_xlim()))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_170_current_compare.png'), dpi=200, bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fishhook_170_current_compare.pdf'), bbox_inches='tight')
    plt.close(fig)

    for k, name in enumerate(CORNERS):
        line = ['%s:' % name]
        for stem, lab, col in CASES:
            _, V = data[stem]
            line.append('%s mean %.2f max %.2f |' % (lab, V[:, k].mean(), V[:, k].max()))
        print(' '.join(line))
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
