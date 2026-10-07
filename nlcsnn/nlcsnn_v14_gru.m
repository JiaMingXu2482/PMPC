function [h1_new, h2_new] = nlcsnn_v14_gru(nfl, h1, h2, w)
%NLCSNN_V14_GRU  PyTorch GRU gate order r,z,n, evaluated at this sample.
%#codegen
h1_new = layer(nfl, h1, w.gru_weight_ih_l0, w.gru_weight_hh_l0, ...
    w.gru_bias_ih_l0, w.gru_bias_hh_l0);
h2_new = layer(h1_new, h2, w.gru_weight_ih_l1, w.gru_weight_hh_l1, ...
    w.gru_bias_ih_l1, w.gru_bias_hh_l1);
end

function h_new = layer(x, h, W_i, W_h, b_i, b_h)
hidden = numel(h);
gi = W_i*x + b_i;
gh = W_h*h + b_h;
r = sigmoid(gi(1:hidden) + gh(1:hidden));
z = sigmoid(gi(hidden+1:2*hidden) + gh(hidden+1:2*hidden));
n = tanh(gi(2*hidden+1:3*hidden) + ...
    r.*gh(2*hidden+1:3*hidden));
h_new = (1-z).*n + z.*h;
end

function y = sigmoid(x)
y = 1 ./ (1 + exp(-x));
end
