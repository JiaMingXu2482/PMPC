function I_cmd = inv_current_impl_10ms(Fd_N, t, Temp_C, cmpD, cmpRD, cid, F_at0, F_at16)
%INV_CURRENT_IMPL_10MS 原 inv_mf_impl_10ms 的电流计算部分。
%#codegen
% 每角调用一次/1 ms，电流仍每10 ms更新，间隔内保持。
% 只用 inv_weights_m；不计算边界、不读取真实减振器内部状态。
% Fd_N [N]、cmpD [mm]、cmpRD [mm/s] 均沿用 CarSim 符号。
% cid: 1=L1, 2=L2, 3=R1, 4=R2；Temp_C 保留原接口。
% 输出 I_cmd [A]。送执行器的命令不加本次拆分用的历史寄存。
% 可选F_at0/F_at16：当前速度下两端电流对应的平均力，来自边界块，CarSim符号。
% 接入后：目标先裁剪供逆NN使用；超界则选对应端点电流，不对NN外推。
% 保留6参旧调用；8参调用才启用端点饱和，不能把排序后的Flo/Fhi冒充端点身份。

persistent net st_h_inv t_prevs I_prevs
persistent v_prevs a_filts last_ticks I_holds
MAX_I = 1.6;
SLEW = 160;
NC = 4;
TAU_ACCEL = 0.002;

if isempty(net)
    net = inv_weights_m();
    st_h_inv = zeros(2, 8, NC);
    t_prevs = NaN(1, NC);
    I_prevs = zeros(1, NC);
    v_prevs = zeros(1, NC);
    a_filts = zeros(1, NC);
    last_ticks = -ones(1, NC);
    I_holds = zeros(1, NC);
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
% 记录饱和端点，保持原逆NN历史推进及10 ms电流更新节拍。
force_target=Fd_N; endpoint=-1;
if nargin>=8 && isfinite(F_at0) && isfinite(F_at16)
    lo=min(F_at0,F_at16); hi=max(F_at0,F_at16);
    force_target=min(max(Fd_N,lo),hi);
    if Fd_N<=lo || Fd_N>=hi
        if abs(F_at16-F_at0)<=1e-6
            endpoint=I_prev; % 两端力不可区分时不因饱和在0/1.6 A间跳变。
        elseif abs(Fd_N-F_at0)<=abs(Fd_N-F_at16)
            endpoint=0;
        else
            endpoint=MAX_I;
        end
    end
end
F_des_rig = -force_target;
x_norm = (x_rig - n.x_ref) / n.x_scale;
v_norm = v_rig / n.v_scale;
a_norm = a_rig / n.a_scale;
fce = F_des_rig / n.f_scale;
ip = I_prev / MAX_I;

if first_step
    % 原来的200步逆网络预热；保留初始 Fdes 和 ip 定义。
    feat_w = [build_motion_feats(x_norm, v_norm, a_norm); ...
              build_cond_inv(fce, ip, x_norm, v_norm, a_norm)];
    h_w = zeros(2, 8);
    for kw = 1:200
        h_w = gru_step_gen(net, feat_w, h_w);
    end
    st_h_inv(:, :, cid) = h_w;
end

tick = floor(t / 0.01 + 1e-9);
do_solve = (tick ~= last_ticks(cid));
last_ticks(cid) = tick;
if ~do_solve
    I_cmd = I_holds(cid);
    return;
end

feat = [build_motion_feats(x_norm, v_norm, a_norm); ...
        build_cond_inv(fce, ip, x_norm, v_norm, a_norm)];
h_new = gru_step_gen(net, feat, st_h_inv(:, :, cid));
st_h_inv(:, :, cid) = h_new;
h2 = h_new(2, :)';
z = max(net.trunk_W * [h2; feat] + net.trunk_b, 0);
delta = net.head_W * z + net.head_b;
I_raw = I_prev + delta;
if endpoint>=0, I_raw=endpoint; end % 将实际下发的饱和命令存入原I_prev/I_hold
I_cmd = min(max(I_raw, I_prev - max_dI), I_prev + max_dI);
I_cmd = min(max(I_cmd, 0), MAX_I);
I_prevs(cid) = I_cmd;
I_holds(cid) = I_cmd;
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

function y = sigmoid(x)
%#codegen
y = 1 ./ (1 + exp(-x));
end
