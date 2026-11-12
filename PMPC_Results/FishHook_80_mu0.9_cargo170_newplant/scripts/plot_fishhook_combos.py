# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 190 kg：两侧两档电流、四组合选择（下层新方案）。

上层 MPC 不变；下层由"连续力分配 + 连续逆模型"改为：
  同侧前后同电流 I_L / I_R，每侧只有 0 A / 1.6 A 两档，共 4 个组合，
  用 force_bounds 的 10 ms 平均力端点 F_at0/F_at16 预测各组合的 Md，
  取 |Md_pred - Md_des| 最小者，带 200 N·m 切换滞环。

  1. MPC baseline        (ctrl=1)
  2. ZENG                (ctrl=2, rho_min=1.0 -> 与 MPC 等价)
  3. Proposed no-comp    (ctrl=3, tau_d=1e-4)
  4. Proposed comp25     (ctrl=3, tau_d=0.025)

输出：fishhook_190_combos.pdf（只出 PDF）
用法：python plot_fishhook_combos.py [结果目录]
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
    ('new_mpc',  'MPC baseline',          '#d62728'),
    ('new_zeng', 'ZENG (rho=1)',          '#ff7f0e'),
    ('new_nc',   'Proposed (no comp)',    '#1f77b4'),
    ('new_c',    'Proposed + comp 25 ms', '#2ca02c'),
]
MAX_I = 1.6


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
    ax_md, ax_cur = axes[1]

    rows = []
    for stem, lab, col in CASES:
        D, t, ch, M, tv = data[stem]
        c = lambda n: M[:, ch.index(n)]
        roll, ltr = c('Roll'), c('LTR')
        ax_roll.plot(tv, roll, color=col, lw=1.7, label=lab)
        ax_ltr.plot(tv, ltr, color=col, lw=1.7, label=lab)
        ax_md.plot(t, D[6, :], color=col, lw=1.3, label=lab + ' $M_{d,des}$')
        ax_md.plot(t, D[13:17, :].T @ (0.5 * np.array([-1.35, -1.55, 1.35, 1.55])),
                   color=col, lw=0.9, ls='--', alpha=0.75)
        k = D[24, :]
        IL = MAX_I * ((k == 1) | (k == 3))
        IR = MAX_I * ((k == 2) | (k == 4))
        lab_l = lab + ' $I_L$' if stem == CASES[0][0] else None
        lab_r = lab + ' $I_R$' if stem == CASES[0][0] else None
        ax_cur.step(t, IL, color=col, lw=1.0, ls='-', where='post', label=lab_l)
        ax_cur.step(t, IR, color=col, lw=1.0, ls=':', where='post', label=lab_r)
        rows.append((lab, np.nanmax(np.abs(roll)), np.nanmax(np.abs(ltr)),
                     np.nanmax(np.abs(D[6, :])), np.nanmax(np.abs(D[16, :])),
                     sum(np.abs(np.diff(k)) > 0.5),
                     100 * np.mean(k == 0.5 * (k[0] + k[-1]))))

    ax_roll.set_ylabel('roll (deg)')
    ax_roll.set_title('(a) Roll angle', fontsize=10)
    ax_ltr.set_ylabel('LTR (left $-$ right)')
    ax_ltr.set_title('(b) Lateral load transfer ratio', fontsize=10)
    ax_ltr.axhline(1, color='k', ls=':', lw=0.9)
    ax_ltr.axhline(-1, color='k', ls=':', lw=0.9)
    ax_md.set_ylabel(r'$M_d$ (N$\cdot$m)')
    ax_md.set_title(r'(c) $M_d$ command (solid) vs predicted actual (dashed)', fontsize=10)
    ax_cur.set_ylabel('current (A)')
    ax_cur.set_title('(d) Left (solid) / right (dotted) damper current', fontsize=10)
    ax_cur.set_yticks([0, 0.4, 0.8, 1.2, 1.6])

    for a in (ax_roll, ax_ltr, ax_md, ax_cur):
        a.grid(alpha=0.3)
        a.set_xlim(0, 6.0)
        a.set_xlabel('time (s)')
        a.legend(fontsize=7, loc='best')

    fig.suptitle('FishHook 80 km/h, $\\mu$=0.9, cargo 190 kg — two-level / four-combination CDC allocation\n'
                 'same-side front and rear share one current; 0 A or 1.6 A per side', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_190_combos.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('%-24s %8s %7s %8s %8s %8s' %
          ('controller', 'roll', 'LTR', 'Md_des', 'Md_pred', 'switch'))
    for r in rows:
        print('%-24s %8.3f %7.3f %8.0f %8.0f %8d' % r[:6])
    print('saved ->', os.path.join(OUT, 'fishhook_190_combos.pdf'))


if __name__ == '__main__':
    main()
