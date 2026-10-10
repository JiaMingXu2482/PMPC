function [F,F_raw] = nlcsnn_v17_force_at_current(net,x,v,a,curr,temp,h)
%NLCSNN_V17_FORCE_AT_CURRENT Evaluate the deployed one-step force map.
% This trial includes the true current-memory buffer and GRU response.
% Its new state is intentionally discarded by the 100 Hz inverse.
%#codegen
[F_raw,~] = nlcsnn_v17_step_flat(net,x,v,a,curr,temp,0.001,h);
F = nlcsnn_deployed_force(net,F_raw,v);
end
