function [F_actual, i_cmd, state] = func_NLCSNNApply(net, ctx, F_des, state, cfg)
%FUNC_NLCSNNAPPLY Invert desired forces and advance four NLCSNN dampers.
% F_des and F_actual use CarSim force polarity. NLCSNN polarity conversion
% occurs only at this adapter boundary.

%#codegen

F_des = reshape(F_des, 4, 1);
F_actual = zeros(4,1);
i_cmd = zeros(4,1);

for j = 1:4
    i_prev = min(max(state.i_prev(j), 0), cfg.i_max);
    F_request_nl = -F_des(j);
    [ij, ~] = nlcsnn_force_to_current(net, ctx.x(j), ctx.v(j), ...
        ctx.a(j), F_request_nl, cfg.temp, state.h(:,j), ...
        cfg.i_max, i_prev);
    if ~isfinite(ij)
        ij = i_prev;
    end
    ij = min(max(ij, 0), cfg.i_max);
    dcurr = (ij - i_prev) / cfg.dt;
    [F_nl, h_new] = nlcsnn_damper_step(net, ctx.x(j), ctx.v(j), ...
        ctx.a(j), ij, dcurr, cfg.temp, cfg.dt, state.h(:,j));
    if ~isfinite(F_nl) || any(~isfinite(h_new))
        F_nl = nlcsnn_predict_force(net, ctx.x(j), ctx.v(j), ...
            ctx.a(j), i_prev, cfg.temp, state.h(:,j));
        h_new = state.h(:,j);
        ij = i_prev;
    end
    F_actual(j) = -F_nl;
    i_cmd(j) = ij;
    state.h(:,j) = h_new;
    state.i_prev(j) = ij;
end
end
