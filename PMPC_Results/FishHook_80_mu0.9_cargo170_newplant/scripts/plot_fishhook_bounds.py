# -*- coding: utf-8 -*-
"""FishHook 80 km/h / mu=0.9 / cargo 170 kg：三方案对比 + Md/Mz 边界。

读取 run_fishhook.m 保存的 <prefix>.mat（控制器 logVec）与
<prefix>.vs/.vsb（CarSim 通道），输出：
  fishhook_170_bounds.png/.pdf        三方案总览（含 Md 边界带）
  fishhook_170_md_allocation.png/.pdf  Md 指令 vs 分配可达 vs 实测
  metrics_bounds.csv

用法：
  python plot_fishhook_bounds.py [结果目录]
"""
import csv
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
    ('fh170_nocomp_c1', 'MPC baseline', '#d62728'),
    ('fh170_nocomp_c3', 'Proposed no-comp', '#1f77b4'),
    ('fh170_comp25_c3', 'Proposed comp 25 ms', '#2ca02c'),
]

# prepare_step.m / pmpc_init.m 里的车辆与轮胎常量
MU = 0.9
RT = 0.393
TF = 1.575
LF = 1.219
B_MD = np.array([-0.55, -0.55, 0.55, 0.55])   # Md = B_MD * F，四角 [L1;L2;R1;R2]


def load_vs(vs_path):
    """读 CarSim .vs/.vsb，返回 (channel_names, data[ns, nch])。"""
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


def mz_bounds(ch, M):
    """按 prepare_step.m 的摩擦圆公式重算 Mz 可达正/负界。"""
    c = lambda n: M[:, ch.index(n)]
    Fz = [c('Fz_L1'), c('Fz_L2'), c('Fz_R1'), c('Fz_R2')]
    Fy = [c('Fy_L1'), c('Fy_L2'), c('Fy_R1'), c('Fy_R2')]
    Td = [np.maximum(c('MY_DR_' + k), 0) for k in ('L1', 'L2', 'R1', 'R2')]
    brake = [np.maximum(RT * np.sqrt(np.maximum((MU * Fz[k]) ** 2 - Fy[k] ** 2, 0)) - Td[k], 0)
             for k in range(4)]
    delta = 0.5 * (c('Steer_L1') + c('Steer_R1')) * np.pi / 180.0
    small = np.abs(delta) < 0.15
    mzpos = np.where(small,
                     TF / (2 * RT) * (brake[0] + brake[1]),
                     (TF / 2 * (brake[0] * np.cos(delta) + brake[1]) - LF * np.sin(delta) * brake[0]) / RT)
    mzneg = np.where(small,
                     -TF / (2 * RT) * (brake[2] + brake[3]),
                     (-TF / 2 * (brake[2] * np.cos(delta) + brake[3]) - LF * np.sin(delta) * brake[2]) / RT)
    return np.maximum(mzpos, 0), np.minimum(mzneg, 0)


def load_case(stem):
    S = loadmat(os.path.join(OUT, stem + '.mat'))
    D, t = S['D'], S['t'].ravel()
    ch, M = load_vs(os.path.join(OUT, stem + '.vs'))
    tv = np.linspace(t[0], t[-1], M.shape[0])
    return D, t, ch, M, tv


def main():
    data = {}
    for stem, lab, col in CASES:
        data[stem] = load_case(stem)

    # ---------- 图 1：三方案总览（含 Md 边界带） ----------
    fig, axes = plt.subplots(3, 3, figsize=(15, 11), sharex='col')
    for k, (stem, lab, col) in enumerate(CASES):
        D, t, ch, M, tv = data[stem]
        c = lambda n: M[:, ch.index(n)]
        md_act, md_cmd, md_lo, md_hi = D[5, :], D[6, :], D[7, :], D[8, :]
        mz = D[17, :]
        mzpos, mzneg = mz_bounds(ch, M)
        mzpos_t = np.interp(t, tv, mzpos)
        mzneg_t = np.interp(t, tv, mzneg)

        ax = axes[0, k]
        ax.plot(tv, c('Roll'), color=col, lw=1.8, label='Roll')
        ax.set_ylabel('Roll (deg)')
        ax2 = ax.twinx()
        ax2.plot(tv, c('LTR'), color='0.35', lw=1.2, ls='--', label='LTR')
        ax2.axhline(1, color='k', ls=':', lw=0.8)
        ax2.axhline(-1, color='k', ls=':', lw=0.8)
        ax2.axhline(0.9, color='gray', ls='--', lw=0.7)
        ax2.axhline(-0.9, color='gray', ls='--', lw=0.7)
        ax2.set_ylabel('LTR')
        ax2.set_ylim(-1.05, 1.05)
        ax.set_title('%s\nroll max %.2f°, LTR max %.2f' %
                     (lab, np.nanmax(np.abs(c('Roll'))), np.nanmax(np.abs(c('LTR')))), fontsize=10)

        ax = axes[1, k]
        ax.fill_between(t, md_lo, md_hi, color='0.85', alpha=0.8, lw=0, label='Md bounds')
        ax.plot(t, md_hi, 'k--', lw=0.9)
        ax.plot(t, md_lo, 'k--', lw=0.9)
        ax.plot(t, md_cmd, color=col, lw=1.6, label='Md command')
        ax.plot(t, md_act, color=col, lw=1.0, ls=':', label='Md actual')
        ax.set_ylabel(r'$M_d$ (N$\cdot$m)')
        ax.set_title('damper roll moment', fontsize=9)

        ax = axes[2, k]
        ax.fill_between(t, mzneg_t, mzpos_t, color='0.85', alpha=0.8, lw=0, label='Mz bounds')
        ax.plot(t, mzpos_t, 'k--', lw=0.9)
        ax.plot(t, mzneg_t, 'k--', lw=0.9)
        ax.plot(t, mz, color=col, lw=1.5, label='Mz command')
        ax.set_ylabel(r'$M_z$ (N$\cdot$m)')
        ax.set_xlabel('time (s)')
        ax.set_title('differential-braking yaw moment', fontsize=9)

        for a in (axes[0, k], axes[1, k], axes[2, k]):
            a.grid(alpha=0.3)
        axes[0, k].legend(fontsize=7, loc='upper left')
        axes[1, k].legend(fontsize=7, loc='lower left')
        axes[2, k].legend(fontsize=7, loc='lower left')

    fig.suptitle('FishHook 80 km/h, mu=0.9, roof cargo 170 kg — Ts=0.05 s, Np=12, Nc=6\n'
                 'no-comp: tau_d=0.1 ms, preview_L=0, alpha=0   |   comp25: tau_d=25 ms, preview_L=25 ms, alpha=1',
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_170_bounds.png'), dpi=200, bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fishhook_170_bounds.pdf'), bbox_inches='tight')
    plt.close(fig)

    # ---------- 图 2：Md 指令 vs 分配可达 vs 实测 ----------
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    for k, (stem, lab, col) in enumerate(CASES):
        D, t, ch, M, tv = data[stem]
        md_act, md_cmd, md_lo, md_hi = D[5, :], D[6, :], D[7, :], D[8, :]
        md_fdes = B_MD @ D[13:17, :]          # 四角 Fdes 折算的 Md
        ax = axes[k]
        ax.fill_between(t, md_lo, md_hi, color='0.85', alpha=0.8, lw=0, label='Md bounds')
        ax.plot(t, md_cmd, color=col, lw=1.7, label='Md command (QP)')
        ax.plot(t, md_fdes, color='k', lw=1.1, ls='--', label='Md from Fdes (allocation)')
        ax.plot(t, md_act, color=col, lw=1.0, ls=':', label='Md actual (CarSim)')
        ax.set_title(lab, fontsize=10)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(r'$M_d$ (N$\cdot$m)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='lower left')
        gap = np.nanmean(np.abs(md_cmd - md_fdes))
        ax.text(0.02, 0.97, 'mean |cmd - alloc| = %.0f N·m' % gap,
                transform=ax.transAxes, fontsize=8, va='top')
    fig.suptitle('Md command vs allocated (from four-corner Fdes) vs actual', fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fishhook_170_md_allocation.png'), dpi=200, bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fishhook_170_md_allocation.pdf'), bbox_inches='tight')
    plt.close(fig)

    # ---------- 指标 ----------
    rows = []
    for stem, lab, col in CASES:
        D, t, ch, M, tv = data[stem]
        c = lambda n: M[:, ch.index(n)]
        md_cmd, md_act, md_lo, md_hi = D[6, :], D[5, :], D[7, :], D[8, :]
        md_fdes = B_MD @ D[13:17, :]
        ltr = c('LTR')
        i = int(np.nanargmax(np.abs(ltr)))
        tpk = float(tv[i])
        cmd_pk = float(np.interp(tpk, t, md_cmd))
        lo_pk = float(np.interp(tpk, t, md_lo))
        hi_pk = float(np.interp(tpk, t, md_hi))
        bound_pk = lo_pk if abs(cmd_pk - lo_pk) <= abs(cmd_pk - hi_pk) else hi_pk
        rows.append(dict(
            case=lab, t_end=float(t[-1]),
            roll_max=float(np.nanmax(np.abs(c('Roll')))),
            ltr_min=float(np.nanmin(ltr)), ltr_max=float(np.nanmax(ltr)),
            ltr_peak_t=tpk,
            md_at_ltr_peak=float(np.interp(tpk, t, md_act)),
            md_cmd_at_ltr_peak=cmd_pk,
            md_bound_at_ltr_peak=bound_pk,
            beta_max=float(np.nanmax(np.abs(c('Beta')))),
            yaw_max=float(np.nanmax(np.abs(c('AVz')))),
            ay_max=float(np.nanmax(np.abs(c('Ay')))),
            vx_min=float(np.nanmin(c('Vx'))),
            mz_mean=float(np.nanmean(np.abs(D[17, :]))), mz_max=float(np.nanmax(np.abs(D[17, :]))),
            md_mean=float(np.nanmean(np.abs(md_act))), md_max=float(np.nanmax(np.abs(md_act))),
            md_cmd_max=float(np.nanmax(np.abs(md_cmd))),
            md_alloc_gap_mean=float(np.nanmean(np.abs(md_cmd - md_fdes))),
            md_track_err_mean=float(np.nanmean(np.abs(md_fdes - md_act))),
            qp=float(100 * np.mean(D[23, :] > 0.5)),
        ))
    with open(os.path.join(OUT, 'metrics_bounds.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(r)
    print('saved ->', OUT)


if __name__ == '__main__':
    main()
