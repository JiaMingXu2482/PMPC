function [F,F_raw] = nlcsnn_v14_force_at_current(net, x, v, a, curr, temp, h)
%NLCSNN_V14_FORCE_AT_CURRENT Evaluate the full v14 model on a frozen state.
% The returned next state is discarded so inverse trials never advance the
% 1 kHz plant. Current lag and GRU response are both included.
%#codegen
n = net.norm_cfg;
u = single([(x-n.x_ref)/n.x_scale; v/n.v_scale; a/n.a_scale; ...
    curr/n.i_scale; 0; (temp-n.t_ref)/n.t_scale]);
[fn, ~] = nlcsnn_v14_step(net, u, h);
F_raw = double(fn)*n.f_scale;
F = nlcsnn_deployed_force(net,F_raw,v);
end
