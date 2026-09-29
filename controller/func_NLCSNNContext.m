function [ctx, state] = func_NLCSNNContext(net, cmpD, cmpRD, state, cfg)
%FUNC_NLCSNNCONTEXT Prepare four-corner NLCSNN kinematics and force bounds.
% Corner order is [L1; R1; L2; R2]. Input CmpD/CmpRD use CarSim's
% compression-positive convention; force bounds use CarSim polarity.

%#codegen

cmpD = reshape(cmpD, 4, 1);
cmpRD = reshape(cmpRD, 4, 1);

for j = 1:4
    if ~isfinite(cmpD(j))
        cmpD(j) = 0;
    end
    if ~isfinite(cmpRD(j))
        if state.initialized && isfinite(state.v_prev(j))
            cmpRD(j) = state.v_prev(j);
        else
            cmpRD(j) = 0;
        end
    end
end

x = cfg.x_ref - cmpD;
v = -cmpRD;
if state.initialized
    a_raw = -(cmpRD - state.v_prev) / cfg.dt;
else
    a_raw = zeros(4,1);
end
for j = 1:4
    if ~isfinite(a_raw(j))
        a_raw(j) = 0;
    end
end

tau = max(cfg.tau_accel, eps);
alpha = 1 - exp(-cfg.dt / tau);
a = (1-alpha) * state.a_filt + alpha * a_raw;
for j = 1:4
    if ~isfinite(a(j))
        a(j) = 0;
    end
end

F_lo = zeros(4,1);
F_hi = zeros(4,1);
for j = 1:4
    F0 = -nlcsnn_predict_force(net, x(j), v(j), a(j), 0, ...
        cfg.temp, state.h(:,j));
    Fmax = -nlcsnn_predict_force(net, x(j), v(j), a(j), ...
        cfg.i_max, cfg.temp, state.h(:,j));
    F_lo(j) = min(F0, Fmax);
    F_hi(j) = max(F0, Fmax);
end

ctx = struct('x', x, 'v', v, 'a', a, 'F_lo', F_lo, ...
    'F_hi', F_hi, 'F_center', 0.5*(F_lo+F_hi));
state.v_prev = cmpRD;
state.a_filt = a;
state.initialized = true;
end
