# -*- coding: utf-8 -*-
"""动态松弛权重的归一化展示（Proposed）。

归一化口径：w_hat = w / w_max  in (0, 1]
  w_hat = 1  -> 顶到上限，约束最紧
  w_hat -> 0 -> 落到下限（w=1），约束几乎免费

w_r 恒等于 w_alpha（都等于 w2），所以只有三条独立曲线：
  w_hat_stab = w2 / w2_max      （横摆 + 侧偏松弛）
  w_hat_LTR  = w1 / w1_max      （LTR 松弛）
  w_hat_env  = w_env / w1_max   （道路包络；FishHook 是常数覆盖 1e-3）

输出：w_normalized.pdf（线性）、w_normalized_log.pdf（对数）
用法：python plot_w_normalized.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.dirname(HERE)

CASES = [
    ('FishHook_80_mu0.9_cargo170_newplant/x3t_q40nc', 'FishHook 80 km/h', 1e4, 2e5, 1e-3),
    ('DLC_mu0.4_interp/p_w10_80',                     'DLC 80 km/h',      1e3, 1e3, None),
    ('Slalom_mu0.85_interp/sw200_c',                  'Slalom 80 km/h',   200, 200, None),
]

L_STAB = r'$\hat w_{r}=\hat w_{\alpha}$ (stab)'
L_LTR = r'$\hat w_{LTR}$'
L_ENV_C = r'$\hat w_{env}$ = %.0e (const)'
L_ENV_E = r'$\hat w_{env}=\hat w_{LTR}$'

plt.rcParams.update({
    'font.size': 9, 'axes.grid': True, 'grid.alpha': 0.3,
    'axes.labelsize': 10, 'legend.fontsize': 7.5,
    'figure.dpi': 150, 'savefig.dpi': 300,
})


def series(stem, w2m, w1m):
    S = loadmat(os.path.join(RES, stem + '.mat'))
    D, t = S['D'], S['t'].ravel()
    ok = np.isfinite(D[25, :]) & np.isfinite(D[26, :])
    return t[ok], D[25, ok] / w2m, D[26, ok] / w1m


def draw(logscale):
    fig, axes = plt.subplots(3, 1, figsize=(7.4, 8.6), sharey=True)
    for ax, (stem, lab, w2m, w1m, wenv) in zip(axes, CASES):
        tt, ws, wl = series(stem, w2m, w1m)
        plot = ax.semilogy if logscale else ax.plot
        plot(tt, ws, color='#d62728', lw=1.4, label=L_STAB)
        plot(tt, wl, color='#1f77b4', lw=1.4, label=L_LTR)
        if wenv is not None:
            ax.axhline(wenv / w1m, color='#2ca02c', lw=1.2, ls='--',
                       label=L_ENV_C % (wenv / w1m))
        else:
            plot(tt, wl, color='#2ca02c', lw=0.9, ls='--', alpha=0.8, label=L_ENV_E)
        ax.set_xlim(0, tt[-1])
        ax.set_ylim(1e-9 if logscale else -0.02, 2.0 if logscale else 1.05)
        ax.set_ylabel(r'$\hat w = w/w_{\max}$')
        ax.set_title('%s   ($w_{2,\max}$=%g, $w_{1,\max}$=%g)' % (lab, w2m, w1m),
                     fontsize=9)
        ax.legend(loc='lower right' if logscale else 'upper right',
                  ncol=3, framealpha=0.9)
    axes[-1].set_xlabel('time (s)')
    fig.suptitle('Proposed: normalised slack weights  $\hat w=w/w_{\max}$%s\n'
                 r'$\hat w=1$ means pinned at the cap (constraint tightest)'
                 % ('  (log scale)' if logscale else ''), fontsize=10)
    fig.tight_layout()
    name = 'w_normalized_log.pdf' if logscale else 'w_normalized.pdf'
    fig.savefig(os.path.join(RES, name), bbox_inches='tight')
    plt.close(fig)
    print('saved ->', name)


if __name__ == '__main__':
    draw(False)
    draw(True)
