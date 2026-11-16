function probe_delay()
%PROBE_DELAY 用 MAIN 自己的 nlcsnn 实现测量 pmpc_mil.slx 减振器链路的时延特性。
% 不使用 MR_NLCSNN 的任何数据或函数。
%
% pmpc_mil.slx 的 Stateflow 只调用：inv_current_impl_10ms / fwd_mf_impl /
% force_bounds_four / pmpc_preview / lon_arbitrate。**没有 curr_dyn_impl**。
% 因此模型内的 MATLAB 侧时间效应只有：10 ms 电流保持 + fwd_mf_impl 的 1 拍 I_prev。
% 本脚本对比两种口径：
%   A 模型现接法      : I_cmd(10 ms ZOH) -> fwd_mf_impl
%   B 若接入 curr_dyn : I_cmd(10 ms ZOH) -> curr_dyn_impl -> fwd_mf_impl

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'nlcsnn'),'-begin');
run(fullfile(root,'init_valve2.m'));

dt = 1e-3; T = 0.60; t = (0:dt:T).'; t0 = 0.15;
cases = { 'comp_up',  200, 0.4, 1.2; 'comp_dn',  200, 1.2, 0.4;
          'reb_up',  -200, 0.4, 1.2; 'reb_dn',  -200, 1.2, 0.4 };
fprintf('%-8s %8s %9s %9s %9s %9s %9s %9s\n', ...
    'case','v[mm/s]','t10[ms]','t90[ms]','tau[ms]','F0[N]','F1[N]','口径');
out = struct();
for k = 1:size(cases,1)
    nm = cases{k,1}; v = cases{k,2}; I0 = cases{k,3}; I1 = cases{k,4};
    Icmd = I0*ones(size(t)); Icmd(t>=t0) = I1;
    for mode = 1:2
        clear inv_current_impl_10ms curr_dyn_impl fwd_mf_impl
        F = zeros(size(t)); Ia = I0;
        for i = 1:numel(t)
            Ihold = zoh10(t(i), Icmd, t);          % 与 inv 的 10 ms 更新一致
            if mode == 2
                Ia = curr_dyn_impl(t(i), Ihold, 1);
            else
                Ia = Ihold;
            end
            F(i) = fwd_mf_impl(t(i), 30, 0, v, Ia, 1);
        end
        F0 = mean(F(t>t0-0.05 & t<t0)); F1 = mean(F(t>t0+0.35)); d = F1-F0;
        [t10,t90,tau] = steps(F, F0, d, t, t0);
        out.(sprintf('%s_m%d',nm,mode)) = struct('t',t,'F',F,'tau_ms',tau,'t10',t10,'t90',t90);
        fprintf('%-8s %8d %9s %9s %9s %9.1f %9.1f %9s\n', nm, v, f(t10), f(t90), f(tau), ...
            F0, F1, ternary(mode==1,'A 现接法','B 加curr_dyn'));
    end
end
save(fullfile(fileparts(mfilename('fullpath')),'probe_result.mat'),'-struct','out');
end

function Ih = zoh10(ti, Icmd, t)
k = floor(ti/0.01 + 1e-9)*0.01;
[~,j] = min(abs(t - k));
Ih = Icmd(j);
end

function [t10,t90,tau] = steps(F, F0, d, t, t0)
t10 = NaN; t90 = NaN; tau = NaN;
if abs(d) < 20, return; end
th10 = F0 + 0.1*d; th90 = F0 + 0.9*d;
if d > 0
    i10 = find(t>=t0 & F>=th10, 1); i90 = find(t>=t0 & F>=th90, 1);
else
    i10 = find(t>=t0 & F<=th10, 1); i90 = find(t>=t0 & F<=th90, 1);
end
if isempty(i10) || isempty(i90), return; end
t10 = (t(i10)-t0)*1e3; t90 = (t(i90)-t0)*1e3;
tau = (t90-t10)/2.19722;
end

function s = f(x)
if isnan(x), s='  —'; else, s=num2str(x,'%.1f'); end
end
function s = ternary(c,a,b)
if c, s=a; else, s=b; end
end
