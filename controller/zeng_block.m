function [sys, i_cmd] = zeng_block(u, h_plant, x_plant, v_plant, a_plant, ZENG_P) %#codegen
%ZENG_BLOCK  Fixed seven-state ZENG entry point for zeng_mil.

if ZENG_P.Pm.MPCParameters.ControllerVariant ~= 2
    error('zeng_block:WrongVariant', 'zeng_block requires ControllerVariant=2.');
end
if numel(u) ~= 54
    error('zeng_block:InvalidInputSize', 'Expected 54 CarSim export channels, received %d.', numel(u));
end

persistent St
if isempty(St)
    St = ZENG_P.S0;
end
[sys, St, i_cmd] = pmpc_step(u(:), ZENG_P.Pm, St, ...
    h_plant, x_plant, v_plant, a_plant);
end
