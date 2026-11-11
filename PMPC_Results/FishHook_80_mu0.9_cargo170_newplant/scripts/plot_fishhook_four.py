# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 170 kg / 1 s 预备段：四控制器对比。

  1. MPC baseline          (c1, 连续 CDC)
  2. Proposed no-comp      (c3, 连续 CDC)
  3. Proposed comp25       (c3, 连续 CDC + 25 ms 补偿)
  4. SH hard               (c3, 开关 CDC，恒 0 A)

Mz/AFS 由同一套 QP 给出；第 4 个只换 CDC 实现层（做法 A）。

输出：fishhook_170_four_compare.png/.pdf
用法：python plot_fishhook_four.py [结果目录]
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
    ('fin190_passive',            'Passive vehicle',            '#7f7f7f'),
    ('fh190_c1',                  'MPC (continuous CDC)',       '#d62728'),
    ('r3_50_nc',                  'Proposed (no comp)',         '#1f77b4'),
    ('r3_50_c',                   'Proposed + delay comp 10 ms','#2ca02c'),
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
    I = S.get('Icmd', None)
    if I is not None:
        ti = np.asarray(I['time'][0, 0]).ravel().astype(float)
        V = np.asarray(I['signals'][0, 0]['values'][0, 0], dtype=float)
    else:
        ti, V = None, None
    return D, t, ch, M, tv, ti, V


def main():
    data = {stem: load_case(stem) for stem, *_ in CASES}

    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    ax_roll, ax_ltr = axes[0]
    ax_md, ax_mz = axes[1]
    ax_i, ax_d = axes[2]

    rows = []
    for stem, lab, col in CASES:
        D, t, ch, M, tv, ti, V = data[stem]
        c = lambda n: M[:, ch.index(n)]
        roll, ltr = c('Roll'), c('LTR')
        ax_roll.plot(tv, roll, color=col, lw=1.8, label=lab)
        ax_ltr.plot(tv, ltr, color=col, lw=1.8, label=lab)
        ax_md.plot(t, D[5, :], color=col, lw=1.5, label=lab + ' actual')
        ax_md.plot(t, D[6, :], color=col, lw=0.9, ls='--', alpha=0.7)
        ax_mz.plot(t, D[17, :], color=col, lw=1.4, label=lab)
        if ti is not None:
            ax_i.plot(ti, V.mean(1), color=col, lw=1.4, label=lab)
            ax_d.plot(ti, V[:, [0, 1]].mean(1) - V[:, [2, 3]].mean(1),
                      color=col, lw=1.4, label=lab)
            imean = float(V.mean())
            imax = float(V.max())
        else:
            imean = imax = np.nan
        rows.append((lab, t[-1], np.nanmax(np.abs(roll)), np.nanmax(np.abs(ltr)),
                     np.nanmax(np.abs(D[5, :])), np.nanmax(np.abs(D[17, :])),
                     imean, imax, 100 * np.mean(D[23, :] > 0.5)))

    ax_roll.set_ylabel('roll (deg)')
    ax_roll.set_title('(a) Roll angle', fontsize=10)
    ax_ltr.set_ylabel('LTR (right $-$ left)')
    ax_ltr.set_title('(b) Lateral load transfer ratio', fontsize=10)
    ax_ltr.axhline(1, color='k', ls=':', lw=0.8)
    ax_ltr.axhline(-1, color='k', ls=':', lw=0.8)
    ax_ltr.axhline(0.9, color='gray', ls='--', lw=0.7)
    ax_ltr.axhline(-0.9, color='gray', ls='--', lw=0.7)
    ax_md.set_ylabel(r'$M_d$ (N$\cdot$m)')
    ax_md.set_title(r'(c) Damper roll moment: actual (solid) / command (dashed)', fontsize=10)
    ax_mz.set_ylabel(r'$M_z$ (N$\cdot$m)')
    ax_mz.set_title('(d) Differential-braking yaw moment (same QP for all)', fontsize=10)
    ax_i.set_ylabel('mean damper current (A)')
    ax_i.set_title('(e) Mean damper current', fontsize=10)
    ax_d.set_ylabel(r'$I_L-I_R$ (A)')
    ax_d.set_title('(f) Left-right current difference', fontsize=10)
    ax_d.set_xlabel('time (s)')

    for a in (ax_roll, ax_ltr, ax_md, ax_mz, ax_i, ax_d):
        a.grid(alpha=0.3)
        a.set_xlim(0, 5.0)
        a.legend(fontsize=7, loc='best')
    for a in (ax_i, ax_d):
        a.set_xlabel('time (s)')

    ax_roll.annotate('MPC baseline\nrollover 3.72 s', xy=(3.81, 40), xytext=(4.0, 45),
                     fontsize=8, color='#d62728',
                     arrowprops=dict(arrowstyle='->', color='#d62728', lw=0.8))

    fig.suptitle('FishHook 80 km/h, mu=0.9, roof cargo 190 kg, 1 s preparation — four controllers\n'
                 'Passive / MPC / Proposed / Proposed + delay comp',
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_170_four_compare.png'), dpi=200, bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fishhook_170_four_compare.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('%-22s %6s %8s %7s %8s %8s %8s %6s' %
          ('controller', 't_end', 'roll', 'LTR', 'Md_act', 'Mz', 'I_mean', 'QP'))
    for r in rows:
        print('%-22s %6.2f %8.2f %7.2f %8.0f %8.0f %8.2f %5.0f%%' %
              (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[8]))
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
