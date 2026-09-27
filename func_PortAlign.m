function func_PortAlign(blk, side, y1, pitch)
%FUNC_PORTALIGN  调块高, 让它某一侧的端口正好落在 y1, y1+pitch, y1+2*pitch, ...
%   func_PortAlign(blk, 'in' |'out', y1, pitch)
%
%   用途: 两个块对接时(Demux 出口 -> Mux 入口), 只要两边用同一个 (y1,pitch)
%   排一遍, 之间的连线就全是水平直线, 图上不会出现一大把斜线。
%
%   ⚠️ Simulink 的端口间距**只能是 5 的整数倍**, 而且跟块高不成比例。
%   实测 (Mux):
%       N=11: h=400->35  h=600->55  h=800->70  h=1000->90
%       N=62: h=600-> 5  h=800->10  h=1000->15
%   所以**不要**去反解"端口 y = top + h*f(i)"那种线性公式 —— 第一版就是这么写的,
%   残差 55 px, 画出来全是斜线。这里改成二分找"刚好排得出目标间距的最小块高",
%   再整体平移。端口相对块顶的偏移只跟块高有关, 所以平移是精确的。
%
%   pitch 不是 5 的倍数会直接报错; 目标间距跨过去了(比如从 5 直接跳到 15)也报错,
%   不会悄悄给一个差不多的值。

assert(mod(pitch,5) == 0, 'func_PortAlign:Pitch5', ...
       '%s: 端口间距必须是 5 的倍数, 给的是 %g', blk, pitch);
p0 = get_param(blk,'Position');
n  = numel(local_porty(blk, side));
if n < 2
    p = get_param(blk,'Position');
    h = p(4) - p(2);
    t = round(y1 - h/2);
    set_param(blk,'Position',[p(1) t p(3) t+h]);
    return
end

lo = 1;  hi = max(64, 4*pitch*n);
while local_pitch(blk, p0, side, hi, n) < pitch
    hi = hi * 2;
    assert(hi < 1e5, 'func_PortAlign:NoFit', '%s 排不出 %g 的端口间距', blk, pitch);
end
while hi - lo > 1                       % 找能达到 pitch 的最小块高
    mid = floor((lo+hi)/2);
    if local_pitch(blk, p0, side, mid, n) < pitch, lo = mid; else, hi = mid; end
end
got = local_pitch(blk, p0, side, hi, n);
assert(got == pitch, 'func_PortAlign:PitchMiss', ...
       '%s 的端口间距跨过了 %g(只能取到 %g), 换一个行距', blk, pitch, got);

y  = local_porty(blk, side);
dy = round(y1 - y(1));
p  = get_param(blk,'Position');
set_param(blk,'Position',[p(1) p(2)+dy p(3) p(4)+dy]);
end

% -------------------------------------------------------------------------
function p = local_pitch(blk, p0, side, h, n)
set_param(blk,'Position',[p0(1) 0 p0(3) h]);
y = local_porty(blk, side);
p = (y(end) - y(1)) / (n - 1);
end

function y = local_porty(blk, side)
ph = get_param(blk,'PortHandles');
if strcmp(side,'in'), hs = ph.Inport; else, hs = ph.Outport; end
y = zeros(numel(hs),1);
for k = 1:numel(hs)
    q = get_param(hs(k),'Position');
    y(k) = q(2);
end
end
