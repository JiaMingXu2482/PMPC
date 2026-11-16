function r = bench_sim(v, Icmd, t, mode)
%BENCH_SIM 用 Controller/MAIN/nlcsnn 的 V22 E0 正模型跑一条电流指令。
%   mode = 'hold' : I_cmd 10 ms ZOH -> fwd_mf_impl（模型现接法）
%   mode = 'curdyn': 再加 curr_dyn_impl（一阶电流环）
%   不使用任何 Simulink 模型，也不使用 MR_NLCSNN 的文件。
dt = median(diff(t));  %#ok<NASGU>
clear fwd_mf_impl curr_dyn_impl
F = zeros(size(t)); Ia = Icmd(1);
for i = 1:numel(t)
    Ih = zoh10(t(i), Icmd, t);
    if strcmpi(mode,'curdyn')
        Ia = curr_dyn_impl(t(i), Ih, 1);
    else
        Ia = Ih;
    end
    F(i) = fwd_mf_impl(t(i), 30, 0, v, Ia, 1);
end
r.t = t(:); r.F = F(:); r.Icmd = Icmd(:); r.v = v; r.mode = mode;
end

function Ih = zoh10(ti, Icmd, t)
k = floor(ti/0.01 + 1e-9)*0.01;
[~,j] = min(abs(t - k));
Ih = Icmd(j);
end
