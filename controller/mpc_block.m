function [sys, i_cmd] = mpc_block(u, h_plant, x_plant, v_plant, a_plant, MPC_P) %#codegen
%MPC_BLOCK  Fixed six-state MPC entry point for mpc_mil.

if MPC_P.Pm.MPCParameters.ControllerVariant ~= 1
    error('mpc_block:WrongVariant', 'mpc_block requires ControllerVariant=1.');
end
if numel(u) ~= 54
    error('mpc_block:InvalidInputSize', 'Expected 54 CarSim export channels, received %d.', numel(u));
end

persistent St
if isempty(St)
    St = MPC_P.S0;
end
[sys, St, i_cmd] = pmpc_step(u(:), MPC_P.Pm, St, ...
    h_plant, x_plant, v_plant, a_plant);
end
