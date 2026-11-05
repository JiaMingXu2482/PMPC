function [I_cmd, Flo, Fhi, F_clipped, F_ach] = ...
    inv_mf_impl_10ms(Fd_N, t, Temp_C, cmpD, cmpRD, cid)
%INV_MF_IMPL_10MS  V25.2 residual inverse damper model, ONE corner.
%   %#codegen compatible. Build MEX per block for speed.
%
%   [I_cmd, Flo, Fhi, F_clipped, F_ach] = inv_mf(Fd_N, t, Temp_C, cmpD, cmpRD, cid)
%
%   Model: V25.2 RI0 CMD Proxy, best.pth epoch 179 (NRMSE-selected),
%   SHA256 4a0567de121ec10b3666e2d2bb7bb786a34317abf363e89523198b1cebc29577.
%   Architecture: motion 20 + condition 11 = 31 -> GRU 2x8 -> concat 39
%   -> Linear(39,16) + ReLU -> Linear(16,1), 2073 parameters.
%   Head is LINEAR and outputs a current increment in Amps:
%     I_raw[k] = I_pred[k-1] + delta[k],  I_cmd[k] = clip(I_raw[k], 0, 1.6).
%   This is NOT v17 inverse: there is NO 9-point grid, NO bisection,
%   NO monotonic G, NO CurrentLag buffer, and NO low-speed hold.
%   Do not apply v17 low-speed hold here: V25.2 was trained without it,
%   and the Python free-running validation used no hold.
%
%   Timing (exactly as in training, v252_data.py):
%     Decision k is a 100 Hz step. Base[k] = I_pred[k-1] (previous
%     predicted command, NOT measured current). Label in training was
%     clip(|I_meas[k+1]|), i.e. the next measured sample, under the
%     assumption I_act[k] ~= I_cmd[k-1]. In deployment there is no
%     measured-current feedback: slots 21 and 23 (ip and v*ip) are built
%     from I_pred[k-1]/1.6 only. F_des[k] is the desired force at step k.
%     The output I_cmd[k] is the command for the NEXT control step:
%     I_act_est[k+1] = I_cmd[k]. In the closed-loop diagram, feed I_cmd
%     through the current-dynamics block first, then into fwd_mf, and
%     compare F_pred at k+1 with F_des at k, not at the same tick.
%
%   Drop a MATLAB Function block per corner (4 blocks). Call the block
%   at 1 kHz; the solver runs only on 10 ms ticks (floor(t/0.01)),
%   outputs are held between ticks. For speed: right-click each block
%   -> C/C++ Code -> Build This Subsystem. The cid selects the state
%   slot so corners never cross-contaminate even in interpreted mode.
%
%   Inputs (all scalars):
%     Fd_N    desired damper force [N], CarSim sign (compression +)
%     t       simulation time [s] (for dt and 10 ms tick)
%     Temp_C  oil temperature [degC] (e.g. 30)
%     cmpD    damper displacement [mm], CarSim sign (compression +)
%     cmpRD   damper velocity [mm/s], CarSim sign
%     cid     corner id: 1=L1, 2=L2, 3=R1, 4=R2 (constant per block)
%
%   Outputs (all scalars):
%     I_cmd      current command [A], [0,1.6], slew-limited 160 A/s
%                (= 1.6 A per 10 ms tick, i.e. full range, so the slew
%                limit never binds beyond the [0,1.6] clip in practice)
%     Flo, Fhi   achievable force envelope [N], CarSim sign, Flo <= Fhi
%                (trial values, see below, NOT the plant F_pred)
%     F_clipped  Fd_N after envelope clipping [N], CarSim sign
%     F_ach      trial force at I_cmd [N], CarSim sign (trial, see below)
%
%   Flo / Fhi / F_ach are computed with a V22 E0 forward TRIAL inside
%   this block, because V25.2 itself has no force map. At each 10 ms
%   tick, starting from this block's own trial forward GRU state
%   (advanced at 1 kHz with the held current, separate from the plant
%   fwd_mf state), a constant candidate current (0 A, 1.6 A, or I_cmd)
%   is held for 10 ms with motion held at the tick values, and the
%   mean trial force over those 10 steps is reported. This is an
%   estimate: future motion over the next 10 ms is unknown at the tick,
%   and the trial history follows I_cmd directly, not I_act through
%   current dynamics. In the closed-loop diagram, use F_pred from
%   fwd_mf for the F_des comparison, not F_ach from this block.
%
%   Reset between runs: clear inv_mf_impl_10ms   (resets all 4 corners)
%
%   Files required in the same directory as this file:
%     inv_weights_m.m  (V25.2 best, embedded weights, no .mat needed)
%     fwd_weights_m.m  (V22 E0, for the trial envelope only)
%   To use ep120 instead of best: replace inv_weights_m.m with the
%   contents of inv_weights_ep120_m.m (same function name), or edit
%   the net = inv_weights_m(); line below to inv_weights_ep120_m().

%#codegen

persistent net netF st_h_inv st_h_trial st_prev_trial t_prevs I_prevs
persistent v_prevs a_filts last_ticks I_holds Flo_holds Fhi_holds
persistent Fcl_holds Fach_holds

MAX_I = 1.6;
SLEW = 160;      % A/s, = 1.6 A per 10 ms tick (full range, see header)
NC = 4;
TAU_ACCEL = 0.002; % a from v diff + 1st-order lag (matches fwd model)

if isempty(net)
    net = inv_weights_m();
    netF = fwd_weights_m();
    st_h_inv = zeros(2, 8, NC);
    st_h_trial = zeros(2, 8, NC);
    st_prev_trial = zeros(1, NC);
    t_prevs = NaN(1, NC);
    I_prevs = zeros(1, NC);
    v_prevs = zeros(1, NC);
    a_filts = zeros(1, NC);
    last_ticks = -ones(1, NC);
    I_holds = zeros(1, NC);
    Flo_holds = zeros(1, NC);
    Fhi_holds = zeros(1, NC);
    Fcl_holds = zeros(1, NC);
    Fach_holds = zeros(1, NC);
end

cid = max(1, min(NC, round(cid)));

t_prev = t_prevs(cid);
I_prev = I_prevs(cid);
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
max_dI = SLEW * 0.01;

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
F_des_rig = -Fd_N;

x_norm = (x_rig - n.x_ref) / n.x_scale;
v_norm = v_rig / n.v_scale;
a_norm = a_rig / n.a_scale;
T_norm = (Temp_C - n.t_ref) / n.t_scale;
fce = F_des_rig / n.f_scale;
ip = I_prev / MAX_I;

if first_step
    % Warmup inverse GRU (200 steps, first sample, base = 0) and trial
    % forward GRU (200 steps, first sample, current = 0). Zero state is
    % out of distribution in training (first 1000 samples excluded),
    % so this avoids a t=0 spike, same intent as v17/v19 warmup.
    feat_w = [build_motion_feats(x_norm, v_norm, a_norm); ...
              build_cond_inv(fce, ip, x_norm, v_norm, a_norm)];
    h_w = zeros(2, 8);
    for kw = 1:200
        h_w = gru_step_gen(net, feat_w, h_w);
    end
    st_h_inv(:, :, cid) = h_w;
    h_tw = zeros(2, 8);
    prev_tw = 0;
    for kw = 1:200
        [~, h_tw, prev_tw] = fwd_step_gen(netF, x_norm, v_norm, ...
            a_norm, T_norm, 0, h_tw, prev_tw);
    end
    st_h_trial(:, :, cid) = h_tw;
    st_prev_trial(cid) = prev_tw;
end

% ---- 10 ms tick: solver @100 Hz, hold between ticks ----
tick = floor(t / 0.01 + 1e-9);
do_solve = (tick ~= last_ticks(cid));
last_ticks(cid) = tick;
% Trial state BEFORE this 1 ms advance is the history state for the
% envelope window starting at this tick (matches the Python envelope
% definition: hidden state before the window, previous current at
% window-1). Save it before advancing, use it for the envelope below.
h_trial_before = st_h_trial(:, :, cid);
prev_trial_before = st_prev_trial(cid);

% ---- trial forward state advance @1kHz, using held current ----
I_held = I_holds(cid);
I_held_norm = I_held / MAX_I;
[~, h_tr, prev_tr] = fwd_step_gen(netF, x_norm, v_norm, a_norm, ...
    T_norm, I_held_norm, st_h_trial(:, :, cid), st_prev_trial(cid));
st_h_trial(:, :, cid) = h_tr;
st_prev_trial(cid) = prev_tr;

if ~do_solve
    I_cmd = I_holds(cid);
    Flo = Flo_holds(cid);
    Fhi = Fhi_holds(cid);
    F_clipped = Fcl_holds(cid);
    F_ach = Fach_holds(cid);
    return;
end

% ---- V25.2 single-step solver (@10 ms) ----
feat = [build_motion_feats(x_norm, v_norm, a_norm); ...
        build_cond_inv(fce, ip, x_norm, v_norm, a_norm)];
h_new = gru_step_gen(net, feat, st_h_inv(:, :, cid));
st_h_inv(:, :, cid) = h_new;
h2 = h_new(2, :)';
z = max(net.trunk_W * [h2; feat] + net.trunk_b, 0);
delta = net.head_W * z + net.head_b;  % Amps, linear head, unbounded
I_raw = I_prev + delta;
I_cmd = min(max(I_raw, I_prev - max_dI), I_prev + max_dI);
I_cmd = min(max(I_cmd, 0), MAX_I);
I_prevs(cid) = I_cmd;

% ---- forward trial envelope @10 ms (see header) ----
% Use the trial history state saved BEFORE this call's 1 ms advance
% (h_trial_before / prev_trial_before), i.e. the state before the
% 10 ms window, exactly as in the Python envelope definition.
F0_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, T_norm, ...
    0, h_trial_before, prev_trial_before);
F16_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, T_norm, ...
    1.0, h_trial_before, prev_trial_before);
Fcmd_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, T_norm, ...
    I_cmd / MAX_I, h_trial_before, prev_trial_before);
Flo_rig = min(F0_rig, F16_rig);
Fhi_rig = max(F0_rig, F16_rig);
Fd_clip_rig = min(max(F_des_rig, Flo_rig), Fhi_rig);

Flo = min(-Flo_rig, -Fhi_rig);
Fhi = max(-Flo_rig, -Fhi_rig);
F_clipped = -Fd_clip_rig;
F_ach = -Fcmd_rig;

I_holds(cid) = I_cmd;
Flo_holds(cid) = Flo;
Fhi_holds(cid) = Fhi;
Fcl_holds(cid) = F_clipped;
Fach_holds(cid) = F_ach;
end


function feat = build_motion_feats(x, v, a)
%#codegen
% 20 motion features, exactly v23_data.py motion_feats order.
av = abs(v);
vp = max(v, 0);
vn = min(v, 0);
sgn_v = tanh(v / 0.1);
slow = exp(-av / 0.1);
feat = [x; v; a; v; x*v; ...
        tanh(2.0*vp); tanh(vp/0.1); ...
        vn*av; vn^3; a*vn; ...
        sgn_v; x^2; x^3; ...
        abs(a); a*v; abs(a)*sgn_v; ...
        slow; x*slow; abs(x)*slow; ...
        sgn_v*abs(x)];
end


function feat = build_cond_inv(f, ip, x, v, a)
%#codegen
% 11 condition features, exactly v23_data.py cond_inv order.
% f = desired force (normalised, rig sign), ip = past current / 1.6.
av = abs(v);
vp = max(v, 0);
vn = min(v, 0);
sgn_v = tanh(v / 0.1);
slow = exp(-av / 0.1);
feat = [f; ip; v*f; v*ip; ...
        f*tanh(vp/0.1); f*vn*av; f*sgn_v; ...
        a*f; f*abs(a); ...
        f*x*slow; f*abs(x)*slow];
end


function nfl = build_nfl_v22(obs)
%#codegen
% obs = [x; v; a; i; T], normalised, i already I[t-1]. 31 features,
% exactly CDC_Model_Release model_def.py nfl_v22 order.
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


function h_new = gru_step_gen(net, inp, h)
%#codegen
% PyTorch GRU, two layers, one time step. Works for both inv and fwd
% weights (same 31->8, 2-layer topology). Biases kept split.
hh0 = h(1, :)';
r0 = sigmoid(net.W_ir0 * inp + net.b_ir0 + net.W_hr0 * hh0 + net.b_hr0);
z0 = sigmoid(net.W_iz0 * inp + net.b_iz0 + net.W_hz0 * hh0 + net.b_hz0);
n0 = tanh(net.W_in0 * inp + net.b_in0 + r0 .* (net.W_hn0 * hh0 + net.b_hn0));
h0_new = (1 - z0) .* n0 + z0 .* hh0;
hh1 = h(2, :)';
r1 = sigmoid(net.W_ir1 * h0_new + net.b_ir1 + net.W_hr1 * hh1 + net.b_hr1);
z1 = sigmoid(net.W_iz1 * h0_new + net.b_iz1 + net.W_hz1 * hh1 + net.b_hz1);
n1 = tanh(net.W_in1 * h0_new + net.b_in1 + r1 .* (net.W_hn1 * hh1 + net.b_hn1));
h1_new = (1 - z1) .* n1 + z1 .* hh1;
h_new = [h0_new'; h1_new'];
end


function [Fn, h_new, prev_I_new] = fwd_step_gen(netF, x_norm, v_norm, ...
    a_norm, T_norm, I_norm, h, prev_I)
%#codegen
% One 1 ms V22 E0 step. Observer sees PREVIOUS applied current only.
obs = [x_norm; v_norm; a_norm; prev_I; T_norm];
nfl = build_nfl_v22(obs);
h_new = gru_step_gen(netF, nfl, h);
h2 = h_new(2, :)';
feat = [h2; nfl];
z = max(netF.trunk_W * feat + netF.trunk_b, 0);
Fn = netF.head_W * z + netF.head_b;
prev_I_new = I_norm;
end


function Fmean_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, ...
    T_norm, cand_norm, h0, prev0)
%#codegen
% 10 ms trial: candidate current held, motion held at tick values,
% starting from trial history state h0 / prev0. Returns mean force
% over 10 steps, in Newtons, rig sign.
h = h0;
prev_I = prev0;
acc = 0;
for s = 1:10
    [Fn, h, prev_I] = fwd_step_gen(netF, x_norm, v_norm, a_norm, ...
        T_norm, cand_norm, h, prev_I);
    acc = acc + Fn;
end
Fmean_rig = (acc / 10) * netF.norm.f_scale;
end


function y = sigmoid(x)
%#codegen
y = 1 ./ (1 + exp(-x));
end
