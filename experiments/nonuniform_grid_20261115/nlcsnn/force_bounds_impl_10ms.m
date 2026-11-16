function [Flo,Fhi,F_at0,F_at16] = force_bounds_impl_10ms( ...
    t,Temp_C,cmpD,cmpRD,I_hist,cid,cmpRD_preview,alpha,keep)
%FORCE_BOUNDS_IMPL_10MS 复用原边界NN，不使用查表、不改变权重。
%#codegen
% 每角1 kHz调用，历史仅使用当前实测cmpD/cmpRD和上一1 ms命令I_hist推进。
% 每10 ms更新：Flo/Fhi为(Np+1)x1，第1项当前界，其余为预测速度条件下的界。
% cmpRD_preview为Np个压缩速度[mm/s]，来自100 Hz pmpc_preview对应角的一行。
% 所有候选由同一历史快照独立试算10步；不把预测运动写入持久历史。
% 固定当前位移、加速度和温度，替换试算速度；不是完整未来运动/历史轨迹预测。
% F_at0/F_at16是当前速度下0/1.6 A对应的10 ms平均力[N]，送逆模型饱和判断。
% alpha=0使用当前界；keep=1不收缩。四角cid=[L1,L2,R1,R2]。
% 仅替换原边界块的函数调用；同一角不要同时调用本函数两次。

% 兼容原6参数调用：不传预测速度时只输出当前标量Flo/Fhi。
% 新调用需为所有角使用相同固定Np；修改Np后重新初始化模型。
if nargin<7, cmpRD_preview=zeros(0,1); end
if nargin<8, alpha=0; end
if nargin<9, keep=1; end

persistent netF st_h_trial st_prev_trial t_prevs
persistent v_prevs a_filts last_ticks Flo_holds Fhi_holds F0_holds F16_holds
MAX_I = 1.6;
NC = 4;
TAU_ACCEL = 0.002;
NP=coder.const(numel(cmpRD_preview));

if isempty(netF)
    netF = fwd_weights_m();
    st_h_trial = zeros(2, 8, NC);
    st_prev_trial = zeros(1, NC);
    t_prevs = NaN(1, NC);
    v_prevs = zeros(1, NC);
    a_filts = zeros(1, NC);
    last_ticks = -ones(1, NC);
    Flo_holds = zeros(NP+1, NC);
    Fhi_holds = zeros(NP+1, NC);
    F0_holds = zeros(1,NC);
    F16_holds = zeros(1,NC);
end
cid = max(1, min(NC, round(cid)));
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

% 当前正/逆权重文件的归一化数值相同，边界不再加载逆网络。
n = netF.norm;
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

if first_step
    % 原来的200步正模型预热，电流为0；不依赖初始 Fdes。
    h_tw = zeros(2, 8);
    prev_tw = 0;
    for kw = 1:200
        [~, h_tw, prev_tw] = fwd_step_gen(netF, x_norm, v_norm, ...
            a_norm, T_norm, 0, h_tw, prev_tw);
    end
    st_h_trial(:, :, cid) = h_tw;
    st_prev_trial(cid) = prev_tw;
end

tick = floor(t / 0.01 + 1e-9);
do_solve = (tick ~= last_ticks(cid));
last_ticks(cid) = tick;
% 与原代码相同：边界试算使用本次历史推进之前的快照。
h_trial_before = st_h_trial(:, :, cid);
prev_trial_before = st_prev_trial(cid);

% 原 I_holds(cid) 从外部历史寄存传入，仍只推进一次真实试算历史。
I_held_norm = I_hist / MAX_I;
[~, h_tr, prev_tr] = fwd_step_gen(netF, x_norm, v_norm, a_norm, ...
    T_norm, I_held_norm, st_h_trial(:, :, cid), st_prev_trial(cid));
st_h_trial(:, :, cid) = h_tr;
st_prev_trial(cid) = prev_tr;
if ~do_solve
    Flo = Flo_holds(:,cid);
    Fhi = Fhi_holds(:,cid);
    F_at0 = F0_holds(cid);
    F_at16 = F16_holds(cid);
    return;
end

% 两个候选各用同一快照的副本，试算不会改变持久历史。
F0_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, T_norm, ...
    0, h_trial_before, prev_trial_before);
F16_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, T_norm, ...
    1.0, h_trial_before, prev_trial_before);
F_at0=-F0_rig; F_at16=-F16_rig;
lo=min(F_at0,F_at16); hi=max(F_at0,F_at16);
Flo=zeros(NP+1,1); Fhi=zeros(NP+1,1);
Flo(1)=lo; Fhi(1)=hi;
alpha=min(max(alpha,0),1); keep=min(max(keep,0),1);
for j=1:NP
    l=lo; h=hi;
    if alpha>0 && isfinite(cmpRD_preview(j)) && cmpRD_preview(j)~=cmpRD
        vp=-cmpRD_preview(j)/n.v_scale;
        fp0=-trial_mean_force(netF,x_norm,vp,a_norm,T_norm, ...
            0,h_trial_before,prev_trial_before);
        fp16=-trial_mean_force(netF,x_norm,vp,a_norm,T_norm, ...
            1,h_trial_before,prev_trial_before);
        if isfinite(fp0) && isfinite(fp16)
            l=min(fp0,fp16); h=max(fp0,fp16);
            if cmpRD_preview(j)*cmpRD<0
                % 过零时取当前/预测区间交集；交集为空则退回当前界。
                % 滞回允许零速非零力，因此不把0 N当作总能达到的力。
                l=max(l,lo); h=min(h,hi);
                if l>h, l=lo; h=hi; end
            end
        end
    end
    l=(1-alpha)*lo+alpha*l; h=(1-alpha)*hi+alpha*h;
    mid=.5*(l+h); half=.5*(h-l)*keep;
    Flo(j+1)=mid-half; Fhi(j+1)=mid+half;
end
Flo_holds(:,cid)=Flo; Fhi_holds(:,cid)=Fhi;
F0_holds(cid)=F_at0; F16_holds(cid)=F_at16;
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
