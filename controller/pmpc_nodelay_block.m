function [sys, i_cmd, long_diag] = pmpc_nodelay_block(u, h_plant, x_plant, v_plant, a_plant, PMPC_NODELAY_P) %#codegen
%PMPC_NODELAY_BLOCK  PMPC ablation: six-state predictor, unchanged NLCSNN plant.
if numel(u) ~= 54
    error('pmpc_nodelay_block:InvalidInputSize', ...
        'Expected 54 CarSim export channels, received %d.', numel(u));
end
if PMPC_NODELAY_P.Pm.MPCParameters.ControllerVariant ~= 4 || ...
        PMPC_NODELAY_P.Pm.MPCParameters.Nx ~= 6
    error('pmpc_nodelay_block:WrongVariant', ...
        'pmpc_nodelay_block requires six-state PMPC-noDelay.');
end
uc = u(:);

persistent St
if isempty(St)
    St = PMPC_NODELAY_P.S0;
end
[sys, St, i_cmd] = pmpc_step(uc, PMPC_NODELAY_P.Pm, St, ...
    h_plant, x_plant, v_plant, a_plant);
long_diag = [St.LongCoord.diag; St.LongCoord.Fx_request; ...
    St.LongCoord.Fx_achieved; St.LongCoord.margin_prev; ...
    St.LongCoord.speed_mismatch];
end
