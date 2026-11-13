# -*- coding: utf-8 -*-
"""Section V figures (15) for the TVT revision, per
TEX/SECTION_V_MIL_FIGURE_PLAN.md (2026-11-13, 15 figures + 2 tables).

Names/colours are fixed by the plan:
  Controller A (MPC)                 green   #2CA02C
  Controller B (ZENG)                blue    #1F77B4
  Controller C (Proposed, comp)      red     #D62728
  Controller D (Proposed, no comp)   brown   #8C564B
  Reference                          black
  Passive                            grey    #7F7F7F

Figures
  D1 XY, D2 beta-r + stability boundary, D3 Mz, D4 Vx, D5 ey, D6 epsi
  F1 LTR, F2 roll (+inset), F3 Md cmd/actual
  S1 ey, S2 Mz, S3 LTR, S4 Md cmd/act (C vs D), S5 front axle, S6 rear axle

Usage: python tools/plot_paper_figs.py
"""
import json
import os
import struct

import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(os.path.dirname(os.path.dirname(ROOT)), 'TEX', 'newfigures')
os.makedirs(OUT, exist_ok=True)
FW, FH = 3.5, 2.55

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'STIXGeneral', 'DejaVu Serif'],
    'mathtext.fontset': 'stix',
    'font.size': 9,
    'axes.labelsize': 9,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'legend.fontsize': 7.2,
    'lines.linewidth': 1.0,
    'axes.linewidth': 0.6,
    'grid.linewidth': 0.25,
    'grid.alpha': 0.35,
    'savefig.bbox': 'standard',
    'xtick.major.pad': 2,
    'ytick.major.pad': 2,
})

STY = {
    'A': dict(c='#2CA02C', ls=(0, (5, 1.6)), m='o', ms=2.9, z=5, lab='Controller A'),
    'B': dict(c='#1F77B4', ls=(0, (1.4, 1.4)), m='s', ms=2.7, z=4, lab='Controller B'),
    'C': dict(c='#D62728', ls='-', m=None, z=3, lw=1.4, lab='Controller C'),
    'D': dict(c='#8C564B', ls=(0, (5, 1.4, 1.4, 1.4)), m='^', ms=2.9, z=3,
              lab='Controller D'),
    'PAS': dict(c='#7F7F7F', ls=(0, (0.7, 1.3)), m=None, z=2, lw=1.4,
                lab='Passive'),
    'REF': dict(c='k', ls=(0, (1.2, 1.2)), m=None, z=7, lw=1.0, lab='Reference'),
    'BND': dict(c='0.45', ls=(0, (4, 2)), m=None, z=6, lw=0.9,
                lab='Stability boundary (75 km/h)'),
}

DIR_DLC = 'DLC_mu0.4_interp'
DIR_FH = 'FishHook_80_mu0.9_cargo170_newplant'
DIR_SL = 'Slalom_mu0.85_interp'

DLC_CASES = [('v6_c3c_%d', 'C'), ('v6_c2_%d', 'B'), ('v6_c1_%d', 'A')]
FH_CASES = [('fin190_passive', 'PAS'), ('fh_c3_nc', 'D'), ('fh_c3_c', 'C'),
            ('fh_c1', 'A')]
SL_CASES = [('sv_c3c_%d', 'C'), ('sv_c3nc_%d', 'D'), ('sv_c1_%d', 'A')]

# vehicle parameters of the controller version used for this batch
V_P = dict(m=1830.0, lf=1.219, lr=1.731, g=9.81, CarHat=103.8e3, CbarR=0.0)

_cache = {}


def read_vs(path):
    raw = open(path, 'rb').read().decode('utf-8', errors='ignore')
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
    buf = open(path[:-3] + '.vsb', 'rb').read()
    nch = struct.unpack_from('<6i', buf, 0)[5]
    data = np.frombuffer(buf, dtype='<f4', offset=24)
    ns = data.size // nch
    return names, data[:nch * ns].reshape(ns, nch)


def load(scn, stem):
    key = (scn, stem)
    if key in _cache:
        return _cache[key]
    d = os.path.join(ROOT, scn)
    S = loadmat(os.path.join(d, stem + '.mat'))
    out = {'D': S['D'], 't': S['t'].ravel()}
    Vm = loadmat(os.path.join(d, stem + '_sig.mat'))
    names = [str(n[0]) for n in Vm['names'].ravel()]
    out['C'] = {n: Vm['data'][:, i] for i, n in enumerate(names)}
    out['tv'] = Vm['t'].ravel()
    _cache[key] = out
    return out


def newfig(h=FH, left=0.155, bottom=0.185, width=0.83, height=0.785):
    fig = plt.figure(figsize=(FW, h))
    ax = fig.add_axes([left, bottom, width, height])
    ax.grid(True)
    ax.tick_params(direction='in', length=2, width=0.6)
    return fig, ax


def draw(ax, x, y, key, markevery=None, lab=None, lw=None, zorder=None):
    s = STY[key]
    kw = dict(color=s['c'], ls=s['ls'], lw=lw or s.get('lw', 1.0),
              label=lab or s['lab'], zorder=zorder or s['z'])
    if s['m']:
        kw.update(marker=s['m'], ms=s['ms'], mfc='none', mew=0.9)
        if markevery:
            kw['markevery'] = markevery
    ax.plot(x, y, **kw)


def auto_y(ax, pad=0.08):
    lo, hi = np.inf, -np.inf
    for ln in ax.lines:
        y = np.asarray(ln.get_ydata(), dtype=float)
        if y.size < 3 or not np.isfinite(y).any():
            continue
        a, b = np.nanmin(y), np.nanmax(y)
        if b - a < 1e-9:
            continue
        lo, hi = min(lo, a), max(hi, b)
    if np.isfinite(lo):
        span = hi - lo
        ax.set_ylim(lo - pad * span, hi + pad * span)





LEG_ORDER = ['Controller A', 'Controller B', 'Controller C', 'Controller D',
             'Passive', 'Reference', 'Stability boundary']


def legend_order(h, l):
    def key(item):
        lab = item[1]
        for i, pre in enumerate(LEG_ORDER):
            if lab.startswith(pre):
                return i
        return len(LEG_ORDER)
    pairs = sorted(zip(h, l), key=key)
    return [p[0] for p in pairs], [p[1] for p in pairs]


def legend_inside(ax, pad_frac=None):
    """Legend inside the frame, placed in an empty band; adds headroom if needed."""
    h0, l0 = ax.get_legend_handles_labels()
    if pad_frac is None:
        pad_frac = 0.10 + 0.062 * max(len(h0), 1)
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    span = y1 - y0
    xs, ys = [], []
    for l in ax.lines:
        lab = l.get_label()
        if not lab or lab.startswith('_'):
            continue
        xs.append(np.asarray(l.get_xdata(), float))
        ys.append(np.asarray(l.get_ydata(), float))
    if xs:
        X = np.concatenate(xs)
        Y = np.concatenate(ys)
        ok = np.isfinite(X) & np.isfinite(Y)
        X, Y = X[ok], Y[ok]
        fx = (X - x0) / (x1 - x0)
        fy = (Y - y0) / span
        top_cnt = np.sum(fy > 1.0 - pad_frac)
        bot_cnt = np.sum(fy < pad_frac)
    else:
        fx = fy = np.array([])
        top_cnt = bot_cnt = 0
    dmin = np.nanmin(Y) if len(ys) else y0
    dmax = np.nanmax(Y) if len(ys) else y1
    if top_cnt <= bot_cnt:
        ax.set_ylim(dmin - 0.06 * span, y1 + pad_frac * span)
        y0n, y1n = ax.get_ylim()
        band = fy > 1.0 - pad_frac
        vert = 'upper'
    else:
        ax.set_ylim(y0 - pad_frac * span, dmax + 0.06 * span)
        y0n, y1n = ax.get_ylim()
        band = fy < pad_frac
        vert = 'lower'
    if len(fx):
        left = np.sum(band & (fx < 0.5))
        right = np.sum(band & (fx >= 0.5))
        horiz = 'left' if left <= right else 'right'
    else:
        horiz = 'left'
    h, l = ax.get_legend_handles_labels()
    h, l = legend_order(h, l)
    if h:
        ax.legend(h, l, loc='%s %s' % (vert, horiz), frameon=True,
                  edgecolor='0.35', framealpha=1.0, fancybox=False,
                  handlelength=2.2, handletextpad=0.5, borderpad=0.35,
                  labelspacing=0.22)

def save(fig, ax, name, xlab='time (s)', ylab='', xlim=None, ylim=None,
         legend=True, loc='best'):
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    if xlim is not None:
        ax.set_xlim(*xlim)
    if ylim is not None:
        ax.set_ylim(*ylim)
    if legend:
        legend_inside(ax)
    path = os.path.join(OUT, name + '.pdf')
    fig.savefig(path)
    fig.savefig(path[:-4] + '.png', dpi=400)
    plt.close(fig)
    print('saved', path)


def ctrl_on_sig(d, row):
    return np.interp(d['tv'], d['t'], d['D'][row])


def ctrl_on_x(d, row):
    return np.interp(d['t'], d['tv'], d['C']['Xo']), d['D'][row]


# ------------------------------------------------------------ stability set
def alpha_r_sat(mu):
    """Rear slip limit used by prepare_step.m for this batch."""
    fzr = V_P['m'] * V_P['g'] * V_P['lf'] / (V_P['lf'] + V_P['lr'])
    cr_cur = max(abs(V_P['CbarR']), 1e3)
    return min(np.arctan(mu * fzr / cr_cur),
               np.arctan(3 * mu * fzr / V_P['CarHat']))


def stability_set(ax, vx, mu, label=True):
    """Controller envelope |r|<=mu g/vx and |tan(beta)-lr r/vx|<=a_sat."""
    rmax = mu * V_P['g'] / vx
    a = alpha_r_sat(mu)
    rr = np.linspace(-rmax, rmax, 120)
    c = V_P['lr'] / vx
    lo = np.degrees(np.arctan(c * rr - a))
    hi = np.degrees(np.arctan(c * rr + a))
    rdeg = np.degrees(rr)
    ax.fill(np.concatenate([lo, hi[::-1]]), np.concatenate([rdeg, rdeg[::-1]]),
            color='0.93', lw=0, zorder=0.5)
    kw = dict(color=STY['BND']['c'], ls=STY['BND']['ls'], lw=STY['BND']['lw'],
              zorder=6)
    ax.plot(lo, rdeg, label=STY['BND']['lab'] if label else None, **kw)
    ax.plot(hi, rdeg, **kw)
    rl = np.degrees([rmax, -rmax])
    xl = np.array([-6.0, 6.0])
    ax.plot(xl, [rl[0], rl[0]], **kw)
    ax.plot(xl, [rl[1], rl[1]], **kw)


# ===================================================================== DLC
def dlc_ref(x):
    return (3.556 / 2 * (1 + np.tanh(4 / 27.852 * (x - 80.059)))
            - 3.556 / 2 * (1 + np.tanh(4 / 24.008 * (x - 132.357))))


EVENTS = [80, 132, 160]


def event_lines(ax):
    for k, xe in enumerate(EVENTS):
        ax.axvline(xe, color='0.6', lw=0.5, ls=(0, (2, 2)), zorder=1)
        ax.text(xe, 0.995, str(k + 1), transform=ax.get_xaxis_transform(),
                fontsize=7, ha='center', va='top', color='0.25')


def d1_xy():
    fig, ax = newfig(h=2.35, bottom=0.20, height=0.74)
    xr = np.linspace(25, 200, 500)
    ax.plot(xr, dlc_ref(xr), color=STY['REF']['c'], ls=STY['REF']['ls'],
            lw=1.0, label=STY['REF']['lab'], zorder=7)
    for pat, key in DLC_CASES:
        d = load(DIR_DLC, pat % 75)
        draw(ax, d['C']['Xo'], d['C']['Yo'], key,
             markevery=max(1, len(d['t']) // 10))
    for k, xe in enumerate(EVENTS):
        ax.axvline(xe, color='0.6', lw=0.5, ls=(0, (2, 2)), zorder=1)
        ax.text(xe, 5.2, str(k + 1), fontsize=7.5, ha='center', va='bottom',
                color='0.25')
    ax.annotate('Controller A' + chr(10) + 'departs', xy=(168, -8.4),
                xytext=(110, -7.0), fontsize=7, ha='left', va='top',
                arrowprops=dict(arrowstyle='->', lw=0.7))
    save(fig, ax, 'fig_dlc_xy', xlab='X (m)', ylab='Y (m)',
         xlim=(20, 200), ylim=(-9, 6), loc='upper left')


def d2_betar():
    fig, ax = newfig()
    stability_set(ax, 75 / 3.6, 0.4)
    for pat, key in DLC_CASES:
        d = load(DIR_DLC, pat % 75)
        b, r = d['C']['Beta'], d['C']['AVz']
        draw(ax, b, r, key, markevery=max(1, len(d['tv']) // 7))
        n = len(b)
        for f in (0.25, 0.65):
            i = int(f * n)
            ax.annotate('', xy=(b[i + 3], r[i + 3]), xytext=(b[i], r[i]),
                        arrowprops=dict(arrowstyle='-|>', lw=0.8,
                                        color=STY[key]['c'],
                                        mutation_scale=7), zorder=9)
        ax.plot(b[0], r[0], ls='none', marker='*', ms=8, mfc='k', mec='k',
                zorder=10)
    ax.axhline(0, color='k', lw=0.4)
    ax.axvline(0, color='k', lw=0.4)
    save(fig, ax, 'fig_dlc_betar', xlab=r'$\beta$ (deg)', ylab=r'$r$ (deg/s)',
         xlim=(-6, 6), ylim=(-18, 18), loc='upper right')


def d3_mz():
    fig, ax = newfig()
    for pat, key in DLC_CASES:
        d = load(DIR_DLC, pat % 75)
        s, mz = ctrl_on_x(d, 17)
        draw(ax, s, mz, key, markevery=max(1, len(s) // 10))
    ax.axhline(0, color='k', lw=0.4)
    event_lines(ax)
    auto_y(ax, pad=0.10)
    save(fig, ax, 'fig_dlc_mz', xlab='X (m)',
         ylab=r'$M_z^{\mathrm{c}}$ (N$\cdot$m)', xlim=(30, 195),
         loc='upper left')


def d4_vx():
    fig, ax = newfig()
    for pat, key in DLC_CASES:
        d = load(DIR_DLC, pat % 75)
        draw(ax, d['C']['Xo'], d['C']['Vx'], key,
             markevery=max(1, len(d['t']) // 10))
    ax.axhline(75, color='k', lw=0.6, ls=(0, (1.2, 1.2)))
    event_lines(ax)
    save(fig, ax, 'fig_dlc_vx', xlab='X (m)', ylab=r'$V_x$ (km/h)',
         xlim=(30, 195), ylim=(66, 78), loc='lower left')


def d5_ey():
    fig = plt.figure(figsize=(FW, FH))
    ax = fig.add_axes([0.155, 0.185, 0.83, 0.785])
    ax.grid(True)
    ax.tick_params(direction='in', length=2, width=0.6)
    for pat, key in DLC_CASES:
        d = load(DIR_DLC, pat % 75)
        s, ey = ctrl_on_x(d, 0)
        draw(ax, s, ey, key, markevery=max(1, len(s) // 10))
    ax.axhline(0, color='k', lw=0.5)
    ax.axvline(132, color='0.6', lw=0.5, ls=(0, (2, 2)), zorder=1)
    ax.set_xlim(30, 195)
    ax.set_ylim(-10.5, 6.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel(r'$e_y$ (m)')
    axin = ax.inset_axes([0.05, 0.06, 0.50, 0.32])
    axin.grid(True, alpha=0.5)
    axin.tick_params(direction='in', length=1.6, width=0.5, labelsize=6.5)
    for pat, key in DLC_CASES:
        d = load(DIR_DLC, pat % 75)
        s, ey = ctrl_on_x(d, 0)
        axin.plot(s, ey, color=STY[key]['c'], ls=STY[key]['ls'], lw=1.0)
    axin.set_xlim(120, 195)
    axin.set_ylim(-1.2, 1.2)
    axin.set_yticks([-1, 0, 1])
    h, l = legend_order(*ax.get_legend_handles_labels())
    ax.legend(h, l, loc='upper left', frameon=True, edgecolor='0.35',
              framealpha=1.0, fancybox=False, handlelength=2.2,
              handletextpad=0.5, borderpad=0.35, labelspacing=0.22)
    path = os.path.join(OUT, 'fig_dlc_ey.pdf')
    fig.savefig(path)
    fig.savefig(path[:-4] + '.png', dpi=400)
    plt.close(fig)
    print('saved', path)


def d6_epsi():
    fig, ax = newfig()
    for pat, key in DLC_CASES:
        d = load(DIR_DLC, pat % 75)
        s, ep = ctrl_on_x(d, 1)
        draw(ax, s, np.rad2deg(ep), key, markevery=max(1, len(s) // 10))
    ax.axhline(0, color='k', lw=0.5)
    event_lines(ax)
    save(fig, ax, 'fig_dlc_epsi', xlab='X (m)', ylab=r'$e_\psi$ (deg)',
         xlim=(30, 195), loc='upper left')


# ================================================================ FishHook
def f1_ltr():
    fig, ax = newfig()
    for stem, key in FH_CASES:
        d = load(DIR_FH, stem)
        tv, y = d['tv'], d['C']['LTR']
        if key == 'PAS':
            i = np.where(np.abs(y) > 0.995)[0]
            if len(i):
                tv, y = tv[:i[0] + 3], y[:i[0] + 3]
        draw(ax, tv, y, key, markevery=max(1, len(tv) // 8))
    ax.axhline(0, color='k', lw=0.4)
    for y in (0.8, -0.8):
        ax.axhline(y, color='k', lw=0.6, ls=(0, (6, 2)))
    for y in (1.0, -1.0):
        ax.axhline(y, color='k', lw=0.6, ls=(0, (1.2, 1.2)))
    d0 = load(DIR_FH, 'fh_c1')
    sw = d0['C']['Steer_SW']
    i1 = int(np.argmax(np.abs(sw)))
    j = np.where(np.sign(sw[i1:]) != np.sign(sw[i1]))[0]
    tmarks = [d0['tv'][i1]] + ([d0['tv'][i1 + j[0]]] if len(j) else [])
    for tr in tmarks:
        ax.axvline(tr, color='0.55', lw=0.6, ls=(0, (2, 2)), zorder=1)
    save(fig, ax, 'fig_fh_ltr', ylab='LTR (-)', xlim=(0, 5),
         ylim=(-1.08, 1.08), loc='upper right')


def f2_roll():
    """Roll angle; the closed-loop zoom is a strip below, so it covers nothing."""
    fig = plt.figure(figsize=(FW, 3.1))
    ax = fig.add_axes([0.155, 0.44, 0.83, 0.53])
    for a in (ax,):
        a.grid(True)
        a.tick_params(direction='in', length=2, width=0.6)
    for stem, key in FH_CASES:
        d = load(DIR_FH, stem)
        draw(ax, d['tv'], d['C']['Roll'], key,
             markevery=max(1, len(d['tv']) // 8))
    ax.axhline(0, color='k', lw=0.4)
    ax.set_xlim(0, 5)
    ax.set_ylim(-85, 22)
    ax.set_ylabel(r'$\phi$ (deg)')
    ax.set_xlabel('time (s)')
    ax.tick_params(labelbottom=True)
    legend_inside(ax, pad_frac=0.16)

    ax2 = fig.add_axes([0.155, 0.10, 0.83, 0.22])
    ax2.grid(True)
    ax2.tick_params(direction='in', length=2, width=0.6)
    for stem, key in FH_CASES:
        if key == 'PAS':
            continue
        d = load(DIR_FH, stem)
        ax2.plot(d['tv'], d['C']['Roll'], color=STY[key]['c'],
                 ls=STY[key]['ls'], lw=1.0)
    ax2.axhline(0, color='k', lw=0.4)
    ax2.set_xlim(1.0, 3.8)
    ax2.set_ylim(-8, 8)
    ax2.set_yticks([-6, 0, 6])
    ax2.set_xlabel('time (s)')
    ax2.text(0.985, 0.92, 'closed loop', transform=ax2.transAxes,
             fontsize=7, va='top', ha='right')
    path = os.path.join(OUT, 'fig_fh_roll.pdf')
    fig.savefig(path)
    fig.savefig(path[:-4] + '.png', dpi=400)
    plt.close(fig)
    print('saved', path)


def f3_md():
    """Anti-roll moment: delivered (solid) and commanded (dashed)."""
    fig, ax = newfig()
    for stem, key in [('fh_c3_c', 'C'), ('fh_c1', 'A')]:
        d = load(DIR_FH, stem)
        me = max(1, len(d['t']) // 10)
        ax.plot(d['t'], d['D'][5, :], color=STY[key]['c'], lw=1.2, ls='-',
                label=STY[key]['lab'] + ' actual', zorder=STY[key]['z'])
        ax.plot(d['t'], d['D'][6, :], color=STY[key]['c'], lw=0.9,
                ls=(0, (1.2, 1.2)), label=STY[key]['lab'] + ' cmd',
                zorder=STY[key]['z'] - 1)
    ax.axhline(0, color='k', lw=0.4)
    save(fig, ax, 'fig_fh_md', ylab=r'$M_d$ (N$\cdot$m)', xlim=(0, 5))


# ================================================================== Slalom
def s1_ey():
    fig, ax = newfig()
    for pat, key in SL_CASES:
        d = load(DIR_SL, pat % 90)
        s, ey = ctrl_on_x(d, 0)
        draw(ax, s, ey, key, markevery=max(1, len(s) // 10))
    ax.axhline(0, color='k', lw=0.4)
    auto_y(ax, pad=0.10)
    save(fig, ax, 'fig_sl_ey', xlab='X (m)', ylab=r'$e_y$ (m)')


def s2_mz():
    fig, ax = newfig()
    for pat, key in SL_CASES:
        d = load(DIR_SL, pat % 90)
        s, mz = ctrl_on_x(d, 17)
        draw(ax, s, mz, key, markevery=max(1, len(s) // 10))
    ax.axhline(0, color='k', lw=0.4)
    auto_y(ax, pad=0.10)
    save(fig, ax, 'fig_sl_mz', xlab='X (m)',
         ylab=r'$M_z^{\mathrm{c}}$ (N$\cdot$m)')


def s3_ltr():
    fig, ax = newfig()
    for pat, key in SL_CASES:
        d = load(DIR_SL, pat % 90)
        draw(ax, d['tv'], d['C']['LTR'], key,
             markevery=max(1, len(d['tv']) // 8))
    ax.axhline(0, color='k', lw=0.4)
    for y in (0.8, -0.8):
        ax.axhline(y, color='k', lw=0.6, ls=(0, (6, 2)))
    for y in (1.0, -1.0):
        ax.axhline(y, color='k', lw=0.6, ls=(0, (1.2, 1.2)))
    save(fig, ax, 'fig_sl_ltr', ylab='LTR (-)', ylim=(-1.08, 1.08),
         loc='lower left')


def s4_md():
    """Anti-roll moment: delivered (solid) and commanded (dashed)."""
    fig, ax = newfig()
    for pat, key in [('sv_c3nc_90', 'D'), ('sv_c3c_90', 'C')]:
        d = load(DIR_SL, pat)
        ax.plot(d['t'], d['D'][5, :], color=STY[key]['c'], lw=1.2, ls='-',
                label=STY[key]['lab'] + ' actual', zorder=STY[key]['z'])
        ax.plot(d['t'], d['D'][6, :], color=STY[key]['c'], lw=0.9,
                ls=(0, (1.2, 1.2)), label=STY[key]['lab'] + ' cmd',
                zorder=STY[key]['z'] - 1)
    ax.axhline(0, color='k', lw=0.4)
    save(fig, ax, 'fig_sl_md', ylab=r'$M_d$ (N$\cdot$m)', xlim=(1.5, 3.6))


def axle_fd(axle, name):
    """Controller C only: left/right, target/actual (same red family)."""
    ia = {'front': (9, 11), 'rear': (10, 12)}[axle]
    it = {'front': (13, 15), 'rear': (14, 16)}[axle]
    mk = {'front': ('o', 's'), 'rear': ('o', 's')}[axle]
    side = {'front': ('LF', 'RF'), 'rear': ('LR', 'RR')}[axle]
    cols = ('#D62728', '#F08A80')
    d = load(DIR_SL, 'sv_c3c_90')
    fig, ax = newfig()
    for k in range(2):
        ax.plot(d['t'], d['D'][ia[k], :], color=cols[k], lw=1.1, ls='-',
                marker=mk[k], ms=3.4, markevery=max(1, len(d['t']) // 12),
                mfc='none', mew=0.9, label=side[k] + ' actual', zorder=6 - k)
        ax.plot(d['t'], d['D'][it[k], :], color=cols[k], lw=0.8,
                ls=(0, (1.2, 1.2)), label=side[k] + ' target', zorder=4 - k)
    ax.axhline(0, color='k', lw=0.4)
    save(fig, ax, name, ylab=r'$F_d$ (N)', xlim=(1.5, 3.6), loc='upper right')


if __name__ == '__main__':
    d1_xy()
    d2_betar()
    d3_mz()
    d4_vx()
    d5_ey()
    d6_epsi()
    f1_ltr()
    f2_roll()
    f3_md()
    s1_ey()
    s2_mz()
    s3_ltr()
    s4_md()
    axle_fd('front', 'fig_sl_fdf')
    axle_fd('rear', 'fig_sl_fdr')
