function run_id()
%RUN_ID 用 V22 E0 正模型重新做时滞辨识（阶跃训练 + 连续验证）。
% 与旧台架的差别：被辨识对象是 Controller 里的 V22 E0（3x 力），不是 MR_NLCSNN 的 v19 A。
here = fileparts(mfilename('fullpath')); addpath(here);
root = fileparts(fileparts(here));
addpath(fullfile(root,'nlcsnn'),'-begin');
run(fullfile(root,'init_valve2.m'));

WARM = 1.0;
cases = { 'comp_up', 200, 0.4, 1.2; 'comp_dn', 200, 1.2, 0.4;
          'reb_up', -200, 0.4, 1.2; 'reb_dn', -200, 1.2, 0.4 };
modes = {'hold','curdyn'};
fid = fopen(fullfile(here,'id_result.txt'),'w');
P = @(varargin) fprintf(fid, varargin{:});
for mi = 1:numel(modes)
    mode = modes{mi};
    P('=== 口径 %s ===%s', mode, newline());
    P('%-9s %8s %8s %9s %9s %9s %9s %9s%s','case','tau[ms]','L[ms]','RMSE_tr','RMSE_r50','RMSE_sm','maxAbs','A[N]',newline());
    for ci = 1:size(cases,1)
        nm = cases{ci,1}; v = cases{ci,2}; I0 = cases{ci,3}; I1 = cases{ci,4};
        % 训练：阶跃
        [t, I, t0] = gen_prof('step', I0, I1, WARM);
        S = bench_sim(v, I, t, mode);
        g = static_map(v, I0, I1, mode);
        F0 = mean(S.F(t>=t0-0.02 & t<t0));
        [tau,L] = fit_tauL(t, I, S.F, g, F0, t0);
        sel = t>=t0-0.02 & t<=t0+0.30;
        Fp = fopdt_sim(t(sel), I(sel), g, tau, L, F0);
        Fw = S.F(sel); A = abs(mean(S.F(t>t0+0.40)) - F0);
        e_tr = rms(Fw - Fp);
        % 验证
        e50 = NaN; esm = NaN; mx = NaN;
        [t2, I2, t02] = gen_prof('ramp50', I0, I1, WARM);
        V2 = bench_sim(v, I2, t2, mode);
        s2 = t2>=t02-0.02 & t2<=t02+0.30;
        F2 = fopdt_sim(t2(s2), I2(s2), g, tau, L, mean(V2.F(t2>=t02-0.02 & t2<t02)));
        e50 = rms(V2.F(s2) - F2);
        [t3, I3, t03] = gen_prof('smooth', I0, I1, WARM);
        V3 = bench_sim(v, I3, t3, mode);
        s3 = t3>=t03-0.02 & t3<=t03+0.42;
        F3 = fopdt_sim(t3(s3), I3(s3), g, tau, L, mean(V3.F(t3>=t03-0.02 & t3<t03)));
        esm = rms(V3.F(s3) - F3); mx = max(abs(V3.F(s3)-F3));
        P('%-9s %8.2f %8d %9.1f %9.1f %9.1f %9.1f %9.1f%s', nm, tau*1e3, L, e_tr, e50, esm, mx, A, newline());
    end
    P('%s', newline());
end
fclose(fid);
type(fullfile(here,'id_result.txt'));
end

function g = static_map(v, I0, I1, mode)
% 用准静态慢扫（0.5 A/s）取 g(I)，只用 V22 E0。
Ig = linspace(min(I0,I1), max(I0,I1), 41);
Tup = 3.0; dt = 1e-3;
t = (0:dt:Tup).'; I = I0 + (I1-I0)*t/Tup;
S = bench_sim(v, I, t, mode);
% 滞后修正：去掉前 10% 的启动
k = t>Tup*0.1;
F = interp1(I(k), S.F(k), Ig, 'linear','extrap');
g = [Ig(:).'; F(:).'];
end

function [tau,L,best] = fit_tauL(t, I, Ftrue, g, F0, t0)
sel = t>=t0-0.10 & t<=t0+0.30;
tw=t(sel); Iw=I(sel); Fw=Ftrue(sel);
tauG = [0, logspace(-3, log10(0.2), 24)];
best = inf; tau=NaN; L=NaN;
for Lg = 0:1:50
    for a = tauG
        Fp = fopdt_sim(tw, Iw, g, a, Lg, F0);
        c = rms(Fw - Fp);
        if c < best, best=c; tau=a; L=Lg; end
    end
end
end

function e = rms(x)
e = sqrt(mean(x(:).^2));
end
