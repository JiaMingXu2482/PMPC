function F = fwd_mf_impl(t, Temp_C, cmpD, cmpRD, I_cmd, cid)
%FWD_MF_IMPL  V22 E0 forward damper model, ONE corner, for MATLAB Function block.
%   %#codegen compatible. Build MEX per block for speed.
%
%   F = fwd_mf_impl(t, Temp_C, cmpD, cmpRD, I_cmd, cid)
%
%   Model: CDC_Model_Release v1.0.0, CDC_FWD_V22_E0, 130 epochs, best ep127,
%   SHA256 dbfb844ff117161cc78c634e307c5215ff88abf769880691e4d72433179351b4.
%   Architecture: NFL 31 -> GRU 2x8 -> concat 39 -> Linear(39,16) + ReLU
%   -> Linear(16,1), 2073 parameters. NO G, NO tau, NO c/s/b head.
%   This is NOT v19 A (21041 parameters, c/s/b + monotonic G + CurrentLag).
%   Do not mix v19 weights into this file: weights are in fwd_weights_m.m.
%
%   Observer (exactly as in training / CDC release):
%     motion/temperature at this sample, current only through the PREVIOUS
%     applied sample: obs = [x; v; a; I_prev; T], I_prev(0) = 0,
%     I_prev(t) = clip(|I(t-1)|/1.6, 0, 1). The current command given to
%     this block does NOT affect the force at the same 1 ms step; it
%     affects the force from the next 1 ms step onward, through I_prev.
%     In the closed-loop diagram, feed I_act (after the current-dynamics
%     block) into I_cmd here, not the raw inverse output.
%
%   Drop a MATLAB Function block per corner (4 blocks). Wire 6 scalar
%   inputs / 1 scalar output. For speed: right-click -> C/C++ Code ->
%   Build This Subsystem. The cid selects the state slot so corners
%   never cross-contaminate even in interpreted mode.
%
%   Inputs (all scalars, same interface and signs as v17_fwd_mf / v19A_fwd_mf):
%     t       simulation time [s]
%     Temp_C  oil temperature [degC] (e.g. 30)
%     cmpD    damper displacement [mm], CarSim sign (compression +)
%     cmpRD   damper velocity [mm/s], CarSim sign
%     I_cmd   applied current [A], clamped to [0,1.6] inside
%             (in closed loop this is I_act, after current dynamics)
%     cid     corner id: 1=L1, 2=L2, 3=R1, 4=R2 (constant per block)
%
%   Output:
%     F       damper force [N], CarSim sign (compression +)
%
%   Acceleration: this block has no accel input, so a_rig is derived from
%   v_rig by finite difference + 1st-order lag (tau 0.002 s, NOT the
%   0.02 s used in v19A_fwd_mf_impl). Reason, measured 2026-10-31 on
%   all 16 validation files (PyTorch forward, dataset accel vs derived
%   accel, valid mask): tau 0.02 gives pooled force diff MAE 6.71 N,
%   P95 30.7 N, and on 6.62 Hz files MAE diff 57-79 N (phase lag
%   40 degrees at 6.62 Hz); tau 0.002 gives pooled MAE 2.26 N,
%   P95 9.3 N; pure diff (tau 0) gives pooled MAE 1.07 N, P95 4.8 N.
%   tau 0.002 is a compromise: it keeps the phase lag at 6.62 Hz to
%   4.8 degrees while still smoothing a one-sample velocity spike
%   (alpha = 0.39 per 1 ms step). If your CarSim velocity is clean,
%   set TAU_ACCEL = 0 (pure diff) for the closest match to training.
%   Training used dataset accel, so even tau 0.002 leaves a small
%   residual difference vs Python validation, mainly at ST step edges.
%
%   Reset between runs: clear fwd_mf_impl   (resets all 4 corners)
%
%   Files required in the same directory as this file:
%     fwd_weights_m.m  (embedded weights, no .mat needed, codegen compatible)

%#codegen

persistent net st_h st_prev t_prevs v_prevs a_filts

NC = 4;
TAU_ACCEL = 0.002;

if isempty(net)
    net = fwd_weights_m();
    st_h = zeros(2, 8, NC);
    st_prev = zeros(1, NC);
    t_prevs = NaN(1, NC);
    v_prevs = zeros(1, NC);
    a_filts = zeros(1, NC);
end

cid = max(1, min(NC, round(cid)));
I_cmd = min(max(I_cmd, 0), 1.6);
I_norm = I_cmd / 1.6;

t_prev = t_prevs(cid);
first_step = isnan(t_prev);
if first_step
    dt = 0.001;
else
    dt = t - t_prev;
    if ~(dt > 0) || ~isfinite(dt)
        dt = 0.001;
    end
end
t_prevs(cid) = t;

n = net.norm;
x_rig = n.x_ref - cmpD;
v_rig = -cmpRD;
if first_step
    a_rig = 0;
else
    a_raw = (v_rig - v_prevs(cid)) / dt;
    if ~isfinite(a_raw), a_raw = 0; end
    alpha_a = 1 - exp(-dt / TAU_ACCEL);
    a_rig = (1 - alpha_a) * a_filts(cid) + alpha_a * a_raw;
    if ~isfinite(a_rig), a_rig = 0; end
end
v_prevs(cid) = v_rig;
a_filts(cid) = a_rig;

x_norm = (x_rig - n.x_ref) / n.x_scale;
v_norm = v_rig / n.v_scale;
a_norm = a_rig / n.a_scale;
T_norm = (Temp_C - n.t_ref) / n.t_scale;

h = st_h(:, :, cid);
prev_I = st_prev(cid);

if first_step
    for kw = 1:200
        [~, h, prev_I] = fwd_step_v22(net, x_norm, v_norm, a_norm, ...
                                      T_norm, I_norm, h, prev_I);
    end
end

n_sub = max(1, round(dt / 0.001));
Fn = 0;
for k = 1:n_sub
    [Fn, h, prev_I] = fwd_step_v22(net, x_norm, v_norm, a_norm, ...
                                   T_norm, I_norm, h, prev_I);
end

st_h(:, :, cid) = h;
st_prev(cid) = prev_I;

F_model = Fn * n.f_scale;
F = -F_model;   % rig -> CarSim sign
end


function [Fn, h_new, prev_I_new] = fwd_step_v22(net, x_norm, v_norm, ...
    a_norm, T_norm, I_norm, h, prev_I)
%#codegen
% One 1 ms model step. Observer sees PREVIOUS applied current only.
obs = [x_norm; v_norm; a_norm; prev_I; T_norm];
nfl = build_nfl_v22(obs);
h_new = gru_step_v22(net, nfl, h);
h2 = h_new(2, :)';
feat = [h2; nfl];
z = max(net.trunk_W * feat + net.trunk_b, 0);
Fn = net.head_W * z + net.head_b;
prev_I_new = I_norm;
end


function nfl = build_nfl_v22(obs)
%#codegen
% obs = [x; v; a; i; T], all normalised, i already I[t-1].
% Order is exactly CDC_Model_Release model_def.py nfl_v22 (31 features).
x = obs(1); v = obs(2); a = obs(3); i = obs(4); t = obs(5);
av = abs(v);
vp = max(v, 0);
vn = min(v, 0);
sgn_v = tanh(v / 0.1);
slow = exp(-av / 0.1);
nfl = [obs; ...
       v; x*v; v*i; v*t; ...
       tanh(2.0*vp); tanh(vp/0.1); i*tanh(vp/0.1); ...
       vn*av; vn^3; a*vn; i*vn*av; ...
       sgn_v; i*sgn_v; x^2; x^3; a*i; ...
       abs(a); a*v; abs(a)*sgn_v; i*abs(a); ...
       slow; x*slow; abs(x)*slow; ...
       i*x*slow; i*abs(x)*slow; sgn_v*abs(x)];
end


function h_new = gru_step_v22(net, nfl, h)
%#codegen
% PyTorch GRU, two layers, one time step. Biases kept split exactly as
% bias_ih / bias_hh in the checkpoint export (do NOT sum them: the
% new-gate bias_hh sits inside the reset-gate multiplication).
hh0 = h(1, :)';
r0 = sigmoid(net.W_ir0 * nfl + net.b_ir0 + net.W_hr0 * hh0 + net.b_hr0);
z0 = sigmoid(net.W_iz0 * nfl + net.b_iz0 + net.W_hz0 * hh0 + net.b_hz0);
n0 = tanh(net.W_in0 * nfl + net.b_in0 + r0 .* (net.W_hn0 * hh0 + net.b_hn0));
h0_new = (1 - z0) .* n0 + z0 .* hh0;
hh1 = h(2, :)';
r1 = sigmoid(net.W_ir1 * h0_new + net.b_ir1 + net.W_hr1 * hh1 + net.b_hr1);
z1 = sigmoid(net.W_iz1 * h0_new + net.b_iz1 + net.W_hz1 * hh1 + net.b_hz1);
n1 = tanh(net.W_in1 * h0_new + net.b_in1 + r1 .* (net.W_hn1 * hh1 + net.b_hn1));
h1_new = (1 - z1) .* n1 + z1 .* hh1;
h_new = [h0_new'; h1_new'];
end


function y = sigmoid(x)
%#codegen
y = 1 ./ (1 + exp(-x));
end
