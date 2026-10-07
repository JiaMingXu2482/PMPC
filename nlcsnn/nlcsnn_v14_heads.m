function f = nlcsnn_v14_heads(net, h2, nfl, v_norm, i_effective, i_filtered)
%NLCSNN_V14_HEADS  v14 viscous, friction, bias and monotone-current heads.
%#codegen
w = net.weights;
v_out = v_norm*net.v_out_scale;
features = [h2; nfl; abs(v_out)];
bias_features = [h2; nfl];
c_slow = mlp3(features, w.c_head_0_weight, w.c_head_0_bias, ...
    w.c_head_3_weight, w.c_head_3_bias, w.c_head_5_weight, w.c_head_5_bias);
s_slow = mlp3(features, w.s_head_0_weight, w.s_head_0_bias, ...
    w.s_head_3_weight, w.s_head_3_bias, w.s_head_5_weight, w.s_head_5_bias);
b = mlp3(bias_features, w.b_head_0_weight, w.b_head_0_bias, ...
    w.b_head_3_weight, w.b_head_3_bias, w.b_head_5_weight, w.b_head_5_bias);
i_features = [i_effective; i_filtered];
c_fast = w.c_fast_2_weight*max(w.c_fast_0_weight*i_features + ...
    w.c_fast_0_bias, single(0)) + w.c_fast_2_bias;
s_fast = w.s_fast_2_weight*max(w.s_fast_0_weight*i_features + ...
    w.s_fast_0_bias, single(0)) + w.s_fast_2_bias;
c = softplus(c_slow + c_fast) + net.c_eps + net.c_min;
s = softplus(s_slow + s_fast) + net.c_eps + net.s_min/numel(w.sigma);
Gc = monotone(i_effective, w.G_c_mlp_0_w_raw, w.G_c_mlp_0_bias, ...
    w.G_c_mlp_2_w_raw, w.G_c_mlp_2_bias, ...
    w.G_c_mlp_4_w_raw, w.G_c_mlp_4_bias, net.g_skip);
Gs = monotone(i_effective, w.G_s_mlp_0_w_raw, w.G_s_mlp_0_bias, ...
    w.G_s_mlp_2_w_raw, w.G_s_mlp_2_bias, ...
    w.G_s_mlp_4_w_raw, w.G_s_mlp_4_bias, net.g_skip);
friction = sum(s .* tanh(v_out ./ w.sigma(:))) * Gs;
f = c*v_out*Gc + friction + b;
end

function y = mlp3(x, W0, b0, W1, b1, W2, b2)
y = W2*max(W1*max(W0*x + b0, single(0)) + b1, single(0)) + b2;
end

function y = monotone(i, R0, b0, R1, b1, R2, b2, skip)
W0 = softplus(R0);
W1 = softplus(R1);
W2 = softplus(R2);
f0 = W2*tanh(W1*tanh(b0) + b1) + b2;
f1 = W2*tanh(W1*tanh(W0*single(1) + b0) + b1) + b2;
fi = W2*tanh(W1*tanh(W0*i + b0) + b1) + b2;
denom = (f1 - f0) + skip + single(1e-8);
y = max(((fi - f0) + skip*i) / denom, single(0));
end

function y = softplus(x)
y = max(x, single(0)) + log1p(exp(-abs(x)));
end
