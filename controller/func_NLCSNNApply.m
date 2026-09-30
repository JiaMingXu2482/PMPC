function [i_cmd, F_pred, i_prev_new] = ...
    func_NLCSNNApply(net, ctx, F_des, i_prev, cfg)
%FUNC_NLCSNNAPPLY  100 Hz controller side: desired force -> current command.
%
%   Inverse model ONLY. The old version also advanced the damper forward
%   model here; that now lives in the 1 kHz plant (func_NLCSNNPlant4),
%   which owns the hidden state. This function never touches h.
%
%   F_des uses CarSim force polarity, corner order [L1;R1;L2;R2].
%   NLCSNN polarity conversion happens only at this adapter boundary.
%
%   F_pred is the controller's PREDICTION of damper force at the tick state
%   with the new current, for logging (sys 5:8). The force actually applied
%   to CarSim is F_plant from the 1 kHz plant -- do not wire F_pred to CarSim.
%
%   i_prev is the previous tick's command (kept in
%   InitialParams.prevstate.nlcsnn.i_prev); it seeds the inverse solver's
%   nearest-branch selection.

%#codegen

F_des  = reshape(F_des, 4, 1);
i_prev = reshape(i_prev, 4, 1);

i_cmd      = zeros(4,1);
F_pred     = zeros(4,1);
i_prev_new = zeros(4,1);

for j = 1:4
    ip = min(max(i_prev(j), 0), cfg.i_max);
    if ~isfinite(ip)
        ip = 0;
    end
    F_request_nl = -F_des(j);   % CarSim -> NLCSNN polarity
    [ij, ~] = nlcsnn_force_to_current(net, ctx.x(j), ctx.v(j), ...
        ctx.a(j), F_request_nl, cfg.temp, ctx.h(:,j), ...
        cfg.i_max, ip);
    if ~isfinite(ij)
        ij = ip;
    end
    ij = min(max(ij, 0), cfg.i_max);
    % Predicted achieved force at the tick state with the new current.
    Fp = -nlcsnn_predict_force(net, ctx.x(j), ctx.v(j), ctx.a(j), ...
                               ij, cfg.temp, ctx.h(:,j));
    if ~isfinite(Fp)
        Fp = 0;
    end
    i_cmd(j)      = ij;
    F_pred(j)     = Fp;
    i_prev_new(j) = ij;
end
end
