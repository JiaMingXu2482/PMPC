function rho = func_ZengRho(Vx, mu, delta_f, beta, r, g, I0, dI)
%FUNC_ZENGRHO  Zeng 2025 稳定指标与权重调度因子 rho。
%#codegen
if nargin < 7, I0 = 0.7; end
if nargin < 8, dI = 0.3; end
%  Zeng, Y. et al., "Integrated Motion Control of Interaction Safety,
%  Stability, and Path-Tracking for Front-and-Rear-Independently-Driven
%  Electric Vehicles," IEEE Trans. Transportation Electrification, 2025.
%
%  Eq.(12) 稳定域: beta_saddle_l <= beta <= beta_saddle_r, |r| <= mu*g/Vx
%  Eq.(13) 指标:   I = 1 - 2*sign((hi-x)(x-lo))*min(|hi-x|,|x-lo|)/(hi-lo)
%                  I_ind = max(I_beta, I_r)
%  Eq.(14) 调度:   rho = 0                              , I_ind <= 0.7
%                  rho = 0.5*(1-cos(pi*(I_ind-0.7)/0.3)), 0.7 < I_ind < 1
%                  rho = 1                              , I_ind >= 1
%
%  只用于 ZENG 控制器的横摆松弛权重：w_r <- w_r * rho。
%  侧偏松弛不调度，始终严格。

persistent DB
if isempty(DB)
    DB = coder.load('ZengSaddleDB_SUV.mat');   % mu_grid, V_grid_kmh, delta_grid_deg, betaL, betaR
end

Vx = max(abs(Vx), 1);                          % 避免低速除零
bL = local_interp3(DB, DB.betaL, mu, Vx*3.6, rad2deg(delta_f));
bR = local_interp3(DB, DB.betaR, mu, Vx*3.6, rad2deg(delta_f));

r_max = mu*g/Vx;
I_r = local_idx(r, -r_max, r_max);

if (bR - bL) <= 1e-8                           % 无稳定域时视为失稳
    I_beta = Inf;
else
    I_beta = local_idx(beta, bL, bR);
end

I_ind = max(I_beta, I_r);
if I_ind <= I0
    rho = 0;
elseif I_ind >= I0 + dI
    rho = 1;
else
    rho = 0.5*(1 - cos(pi*(I_ind - I0)/dI));
end
end

function I = local_idx(x, lo, hi)
%#codegen
% Eq.(13): 区间内 I in [0,1]（边界为 1，中心为 0）；越界时 I > 1。
w = hi - lo;
if w <= 1e-12
    I = Inf;
    return
end
d1 = hi - x;
d2 = x - lo;
I = 1 - 2*sign(d1*d2)*min(abs(d1), abs(d2))/w;
end

function v = local_interp3(DB, A, mu, Vkmh, ddeg)
%#codegen
% 在 (mu, V, delta) 上做三线性插值，超界时钳到边界节点。
[i1,i2,t1] = local_ax(DB.mu_grid, mu);
[j1,j2,t2] = local_ax(DB.V_grid_kmh, Vkmh);
[k1,k2,t3] = local_ax(DB.delta_grid_deg, ddeg);
v = 0;
for a = 0:1
    for b = 0:1
        for c = 0:1
            ii = i1*(1-a) + i2*a;
            wa = (1-t1)*(1-a) + t1*a;
            jj = j1*(1-b) + j2*b;
            wb = (1-t2)*(1-b) + t2*b;
            kk = k1*(1-c) + k2*c;
            wc = (1-t3)*(1-c) + t3*c;
            v = v + wa*wb*wc*A(ii,jj,kk);
        end
    end
end
end

function [i1,i2,t] = local_ax(gr, x)
%#codegen
n = numel(gr);
if x <= gr(1)
    i1 = 1; i2 = 1; t = 0; return
end
if x >= gr(n)
    i1 = n; i2 = n; t = 0; return
end
idx = find(gr <= x, 1, 'last');
if isempty(idx)
    i1 = 1;
else
    i1 = idx(1);
end
i2 = min(i1+1, n);
if i2 == i1
    t = 0;
else
    t = (x - gr(i1))/(gr(i2) - gr(i1));
end
end
