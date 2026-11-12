# -*- coding: utf-8 -*-
"""从 CarSim 的 .vs/.vsb 里提取画图常用的通道，存成 <prefix>_sig.mat。

为什么要这一步
--------------
CarSim 每次运行会在结果目录留下 `<prefix>.vs` + `<prefix>.vsb`（915 个通道，
每个 ~3 MB）。这些文件体积大、格式是 CarSim 私有的，不适合直接进 git，
但里面包含了画论文图需要的全部车辆侧信号（侧倾/侧滑/轮荷/减振器/轮胎…）。

本脚本把它们压成一个小 .mat（~50 个通道，单精度），既能进 git，也能直接用
`scipy.io.loadmat` 或 MATLAB 的 `load` 读。

用法
----
    python extract_signals.py <结果目录> [prefix1 prefix2 ...]

不给 prefix 时，扫描目录下所有有同名 .vs 的 .mat 前缀。

输出
----
    <结果目录>/<prefix>_sig.mat
        t      : (n,)      时间 [s]，取自 CarSim 的 T_Record
        names  : (nch,)    通道名（cellstr）
        data   : (n, nch)  通道数据，single
        units  : (nch,)    单位（从 .vs 头部读，缺失时为空串）
        source : 生成该文件的 .vs 文件名

控制器侧的 logVec / Icmd 仍在 `<prefix>.mat` 里（字段 D / t / Icmd），
不在本文件里重复。

通道选择
--------
见 CHANNELS。改这里就能增减；缺失的通道会被跳过并在终端提示。
"""
import json
import os
import struct
import sys

import numpy as np
from scipy.io import savemat

# 画图常用通道。按主题分组，顺序即 data 的列顺序。
CHANNELS = (
    # 时间与轨迹
    ['T_Record', 'Xo', 'Yo']
    # 车身运动
    + ['Vx', 'Vy', 'AVz', 'Ax', 'Ay', 'Beta']
    + ['Roll', 'Pitch', 'Yaw', 'LTR']
    # 转向
    + ['Steer_SW', 'Steer_L1', 'Steer_R1', 'Throttle', 'VxTarget']
    # 轮胎力 / 滑移
    + ['Fx_L1', 'Fx_L2', 'Fx_R1', 'Fx_R2']
    + ['Fy_L1', 'Fy_L2', 'Fy_R1', 'Fy_R2']
    + ['Fz_L1', 'Fz_L2', 'Fz_R1', 'Fz_R2']
    + ['Alpha_L1', 'Alpha_L2', 'Alpha_R1', 'Alpha_R2']
    + ['Kappa_L1', 'Kappa_L2', 'Kappa_R1', 'Kappa_R2']
    # 减振器
    + ['Fd_L1', 'Fd_L2', 'Fd_R1', 'Fd_R2']
    + ['CmpRD_L1', 'CmpRD_L2', 'CmpRD_R1', 'CmpRD_R2']
    + ['CmpD_L1', 'CmpD_L2', 'CmpD_R1', 'CmpD_R2']
    + ['FsExt_L1', 'FsExt_L2', 'FsExt_R1', 'FsExt_R2']
)


def read_vs_header(vs_path):
    """返回 (names, units)。.vs 头是一个 JSON 对象，后面跟二进制 .vsb。"""
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
    chans = hdr['VsChannelGroup']['Channels']
    names = [c['Name Aliases'][0] for c in chans]
    units = [c.get('Units', '') for c in chans]
    return names, units


def read_vs_data(vs_path):
    names, units = read_vs_header(vs_path)
    buf = open(vs_path[:-3] + '.vsb', 'rb').read()
    nch = struct.unpack_from('<6i', buf, 0)[5]
    data = np.frombuffer(buf, dtype='<f4', offset=24)
    ns = data.size // nch
    return names, units, data[:nch * ns].reshape(ns, nch)


def extract(vs_path, out_mat, channels=CHANNELS):
    names, units, M = read_vs_data(vs_path)
    idx = {n: i for i, n in enumerate(names)}
    keep, missing = [], []
    for c in channels:
        (keep if c in idx else missing).append(c)
    if missing:
        print('  missing %d channel(s): %s' % (len(missing), ', '.join(missing)))
    data = np.asarray(M[:, [idx[c] for c in keep]], dtype=np.float32)
    savemat(out_mat, {
        't': M[:, idx['T_Record']].astype(np.float64),
        'names': np.array(keep, dtype=object),
        'units': np.array([units[idx[c]] for c in keep], dtype=object),
        'data': data,
        'source': os.path.basename(vs_path),
    }, do_compression=True, oned_as='row')
    return len(keep), data.shape


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    root = sys.argv[1]
    prefixes = sys.argv[2:]
    if not prefixes:
        prefixes = sorted(f[:-3] for f in os.listdir(root)
                          if f.endswith('.vs')
                          and os.path.isfile(os.path.join(root, f[:-3] + '.vsb')))
    total = 0
    for p in prefixes:
        vs = os.path.join(root, p + '.vs')
        if not os.path.isfile(vs):
            print('skip %s (no .vs)' % p)
            continue
        out = os.path.join(root, p + '_sig.mat')
        nch, shape = extract(vs, out)
        sz = os.path.getsize(out) / 1024.0
        print('%-28s %2d ch  %s  %6.0f KB' % (p, nch, shape, sz))
        total += sz
    print('total %.1f MB' % (total / 1024.0))


if __name__ == '__main__':
    main()
