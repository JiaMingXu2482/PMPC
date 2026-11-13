classdef cdc_fwd
%CDC_FWD  V22_E0 正模型的公开入口，供离线实验调用。
%   内部实现与 nlcsnn/force_bounds_impl_10ms.m 的局部函数逐行相同，
%   只是把 fwd_step_gen / trial_mean_force 暴露出来。

methods (Static)

    function [Fn, h_new, prev_I_new] = step(netF, x_norm, v_norm, a_norm, ...
            T_norm, I_norm, h, prev_I)
        % 一个 1 ms 步；观察器只看上一拍电流
        obs = [x_norm; v_norm; a_norm; prev_I; T_norm];
        nfl = cdc_fwd.build_nfl(obs);
        h_new = cdc_fwd.gru(netF, nfl, h);
        h2 = h_new(2, :)';
        feat = [h2; nfl];
        z = max(netF.trunk_W * feat + netF.trunk_b, 0);
        Fn = netF.head_W * z + netF.head_b;
        prev_I_new = I_norm;
    end

    function Fmean = trialMean(netF, x_norm, v_norm, a_norm, T_norm, ...
            cand_norm, h0, prev0, n)
        % 候选电流保持 n 步，返回 n 步平均力（rig 符号，N）
        h = h0; prev_I = prev0; acc = 0;
        for s = 1:n
            [Fn, h, prev_I] = cdc_fwd.step(netF, x_norm, v_norm, a_norm, ...
                T_norm, cand_norm, h, prev_I);
            acc = acc + Fn;
        end
        Fmean = (acc / n) * netF.norm.f_scale;
    end

    function [h, prev_I] = converge(netF, x_norm, v_norm, a_norm, T_norm, ...
            I_norm, h0, prev0, nIter)
        % 把 GRU 迭代到不动点（同一输入重复推进）
        h = h0; prev_I = prev0;
        for s = 1:nIter
            [~, h, prev_I] = cdc_fwd.step(netF, x_norm, v_norm, a_norm, ...
                T_norm, I_norm, h, prev_I);
        end
    end

    function [tr, hEnd] = trace(netF, x_norm, v_norm, a_norm, T_norm, ...
            cand_norm, h0, prev0, n)
        % 逐步力轨迹（rig 符号，N）
        h = h0; prev_I = prev0; tr = zeros(n, 1);
        for s = 1:n
            [Fn, h, prev_I] = cdc_fwd.step(netF, x_norm, v_norm, a_norm, ...
                T_norm, cand_norm, h, prev_I);
            tr(s) = Fn;
        end
        tr = tr * netF.norm.f_scale;
        hEnd = h;
    end

    function nfl = build_nfl(obs)
        x = obs(1); v = obs(2); a = obs(3); i = obs(4); t = obs(5);
        av = abs(v);
        vp = max(v, 0);
        vn = min(v, 0);
        sgn_v = tanh(v / 0.1);
        slow = exp(-av / 0.1);
        nfl = [obs; ...
               v; x*v; v*i; v*t; ...
               tanh(2.0*vp); tanh(vp/0.1); i*tanh(vp/0.1); ...
               vn*av; vn^3; a*vn; i*vn*av; ...
               sgn_v; i*sgn_v; x^2; x^3; a*i; ...
               abs(a); a*v; abs(a)*sgn_v; i*abs(a); ...
               slow; x*slow; abs(x)*slow; ...
               i*x*slow; i*abs(x)*slow; sgn_v*abs(x)];
    end

    function h_new = gru(net, inp, h)
        hh0 = h(1, :)';
        r0 = cdc_fwd.sig(net.W_ir0 * inp + net.b_ir0 + net.W_hr0 * hh0 + net.b_hr0);
        z0 = cdc_fwd.sig(net.W_iz0 * inp + net.b_iz0 + net.W_hz0 * hh0 + net.b_hz0);
        n0 = tanh(net.W_in0 * inp + net.b_in0 + r0 .* (net.W_hn0 * hh0 + net.b_hn0));
        h0_new = (1 - z0) .* n0 + z0 .* hh0;
        hh1 = h(2, :)';
        r1 = cdc_fwd.sig(net.W_ir1 * h0_new + net.b_ir1 + net.W_hr1 * hh1 + net.b_hr1);
        z1 = cdc_fwd.sig(net.W_iz1 * h0_new + net.b_iz1 + net.W_hz1 * hh1 + net.b_hz1);
        n1 = tanh(net.W_in1 * h0_new + net.b_in1 + r1 .* (net.W_hn1 * hh1 + net.b_hn1));
        h1_new = (1 - z1) .* n1 + z1 .* hh1;
        h_new = [h0_new'; h1_new'];
    end

    function y = sig(x)
        y = 1 ./ (1 + exp(-x));
    end
end
end
