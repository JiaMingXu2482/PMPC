function [f_norm, h_next, i_filtered] = nlcsnn_v14_step(net, u, h)
%NLCSNN_V14_STEP  One 1 kHz v14 step, including GRU and finite current lag.
% State: two 64-unit GRU layers, then previous K-1 effective-current inputs.
%#codegen
u = single(u(:));
h = single(h(:));
lag_length = double(net.current_lag_length);
assert(numel(h) == double(net.state_length));
nfl = nlcsnn_v14_nfl(u, net.nfl_sigma_v);
[h1, h2] = nlcsnn_v14_gru(nfl, h(1:64), h(65:128), net.weights);
i_effective = single(1) - min(max(abs(u(4)), single(0)), single(1));
i_filtered = net.current_lag_kernel(1) * i_effective;
for j = 1:lag_length-1
    i_filtered = i_filtered + net.current_lag_kernel(j+1)*h(128+j);
end
f_norm = nlcsnn_v14_heads(net, h2, nfl, u(2), i_effective, i_filtered);
h_next = zeros(size(h), 'single');
h_next(1:64) = h1;
h_next(65:128) = h2;
h_next(129) = i_effective;
h_next(130:end) = h(129:end-1);
end
