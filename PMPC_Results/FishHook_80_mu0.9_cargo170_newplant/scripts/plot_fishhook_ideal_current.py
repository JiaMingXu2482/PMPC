# -*- coding: utf-8 -*-
"""FishHook 理想 CDC 电流（开关型）vs 实测 PMPC 电流（连续型）。

理想律（fail-hard：0 A 硬 / 1.6 A 软）：
  roll rate > 0（车身向右滚、左侧拉伸）-> 左侧 0 A，右侧 1.6 A
  roll rate < 0（车身向左滚、右侧拉伸）-> 右侧 0 A，左侧 1.6 A
  切换点 = 侧倾角速度过零点（工程上再提前一个执行器延迟 ~25 ms）

实测：PMPC 的 Md 是连续力矩，四角分配 QP 用二次代价（最小范数解），
所以电流是连续的中间值，不是 0/1.6 A 的开关量。

输出：fishhook_170_ideal_current.png/.pdf
用法：python plot_fishhook_ideal_current.py [结果目录]
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

CASE = 'fh170_prep1_nocomp_c3'
LOGCASE = 'fh170_prep1_log_nocomp_c3'
I_HARD = 0.0
I_SOFT = 1.6
LEAD = 0.025


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


def get_i(stem):
    I = loadmat(os.path.join(OUT, stem + '.mat'))['Icmd']
    t = np.asarray(I['time'][0, 0]).ravel().astype(float)
    V = np.asarray(I['signals'][0, 0]['values'][0, 0], dtype=float)
    return t, V


def main():
    ch, M = load_vs(os.path.join(OUT, CASE + '.vs'))
    S = loadmat(os.path.join(OUT, CASE + '.mat'))
    t = S['t'].ravel()
    tv = np.linspace(t[0], t[-1], M.shape[0])
    c = lambda n: M[:, ch.index(n)]

    sw = c('Steer_SW')
    roll = c('Roll')
    rr = c('AV_R')

    zc = [tv[i] for i in range(1, len(rr))
          if rr[i - 1] * rr[i] < 0 and abs(rr[i] - rr[i - 1]) > 0.5]

    # 理想开关量
    I_L = np.where(rr > 0, I_HARD, I_SOFT)
    I_R = np.where(rr > 0, I_SOFT, I_HARD)

    ti, V = get_i(LOGCASE)
    IL = V[:, [0, 1]].mean(1)
    IR = V[:, [2, 3]].mean(1)
    d_act = IL - IR
    d_ideal = np.where(np.interp(ti, tv, rr) > 0, I_HARD - I_SOFT, I_SOFT - I_HARD)
    m = (ti >= 1.0) & (ti <= 5.0)
    agree = 100 * np.mean((d_act[m] < 0) == (d_ideal[m] < 0))

    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True,
                             gridspec_kw={'height_ratios': [0.8, 1.0, 1.5, 1.2]})

    ax = axes[0]
    ax.plot(tv, sw, color='0.25', lw=1.5)
    ax.set_ylabel('SW (deg)')
    ax.set_title('(a) FishHook steering wheel angle (1 s preparation)', fontsize=10)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(tv, roll, color='#d62728', lw=1.6, label='roll angle')
    ax.set_ylabel('roll (deg)', color='#d62728')
    ax.tick_params(axis='y', labelcolor='#d62728')
    ax2 = ax.twinx()
    ax2.plot(tv, rr, color='#1f77b4', lw=1.2, ls='--', label='roll rate')
    ax2.axhline(0, color='#1f77b4', lw=0.6, ls=':')
    ax2.set_ylabel('roll rate (deg/s)', color='#1f77b4')
    ax2.tick_params(axis='y', labelcolor='#1f77b4')
    for z in zc:
        ax2.axvline(z, color='k', ls=':', lw=0.8)
    ax.set_title('(b) Roll angle / roll rate; dotted = roll-rate zero crossings', fontsize=10)
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.step(tv, I_L, where='post', color='k', lw=2.4, label='ideal L (L1/L2)')
    ax.step(tv, I_R, where='post', color='k', lw=2.4, ls='--', label='ideal R (R1/R2)')
    ax.plot(ti, V[:, 0], color='#1f77b4', lw=1.0, alpha=0.9, label='actual L1')
    ax.plot(ti, V[:, 1], color='#1f77b4', lw=1.0, alpha=0.55, ls='-.', label='actual L2')
    ax.plot(ti, V[:, 2], color='#d62728', lw=1.0, alpha=0.9, label='actual R1')
    ax.plot(ti, V[:, 3], color='#d62728', lw=1.0, alpha=0.55, ls='-.', label='actual R2')
    for z in zc:
        ax.axvline(z, color='k', ls=':', lw=0.8)
    ax.set_ylim(-0.08, 1.75)
    ax.set_ylabel('damper current (A)')
    ax.set_title('(c) Ideal switching current (thick) vs actual PMPC current (thin)', fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=3, loc='upper right')

    ax = axes[3]
    ax.step(ti, d_ideal, where='post', color='k', lw=2.0, label='ideal $I_L-I_R$')
    ax.plot(ti, d_act, color='#2ca02c', lw=1.4, label='actual $I_L-I_R$')
    ax.axhline(0, color='k', lw=0.6, ls=':')
    for z in zc:
        ax.axvline(z, color='k', ls=':', lw=0.8)
    ax.set_ylim(-1.9, 1.9)
    ax.set_ylabel(r'$I_L-I_R$ (A)')
    ax.set_xlabel('time (s)')
    ax.set_title('(d) Left-right current difference: sign agrees %.0f%%, amplitude only 0.2-0.6 A'
                 % agree, fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='upper right')

    for a in axes:
        a.set_xlim(0, 5.0)

    txt = ('ideal switch at roll-rate zero crossings: '
           + ', '.join('%.2f' % z for z in zc) + ' s\n'
           'PMPC: continuous Md -> quadratic allocation -> continuous current')
    axes[2].text(0.01, 0.02, txt, transform=axes[2].transAxes, fontsize=8, va='bottom')

    fig.suptitle('FishHook 80 km/h, mu=0.9, cargo 170 kg — ideal switching CDC current '
                 'vs actual PMPC current (fail-hard: 0 A hard, 1.6 A soft)', fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_170_ideal_current.png'), dpi=200, bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fishhook_170_ideal_current.pdf'), bbox_inches='tight')
    plt.close(fig)

    print('zero crossings:', ['%.2f' % z for z in zc])
    print('sign agreement: %.1f%%' % agree)
    print('actual |I_L-I_R| mean %.2f max %.2f A' % (np.mean(np.abs(d_act[m])), np.max(np.abs(d_act[m]))))
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
