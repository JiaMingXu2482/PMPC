function [Flo,Fhi,F_at0,F_at16,dbg,trace] = force_bounds_test( ...
    t,Temp_C,cmpD,cmpRD,I_hist,cid,cmpRD_preview,alpha,keep,ntrial)
%FORCE_BOUNDS_TEST  与 nlcsnn/force_bounds_impl_10ms.m 逐行相同，
%   唯一差别：试算步数由第 10 个入参 ntrial 控制（原版硬编码 10）。
%   仅用于离线数值实验，不接 Simulink。
%#codegen
if nargin<7, cmpRD_preview=zeros(0,1); end
if nargin<8, alpha=0; end
if nargin<9, keep=1; end
if nargin<10, ntrial=10; end

persistent netF st_h_trial st_prev_trial t_prevs
persistent v_prevs a_filts last_ticks Flo_holds Fhi_holds F0_holds F16_holds
MAX_I = 1.6;
NC = 4;
TAU_ACCEL = 0.002;
NP = numel(cmpRD_preview);

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
h_trial_before = st_h_trial(:, :, cid);
prev_trial_before = st_prev_trial(cid);

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
    if nargout > 4
        dbg = struct('solve', false, 'h0', h_trial_before, ...
            'prev0', prev_trial_before, 'x_norm', x_norm, 'v_norm', v_norm, ...
            'a_norm', a_norm, 'T_norm', T_norm, 'cmpRD', cmpRD);
    end
    return;
end

if nargout > 4
    dbg = struct('solve', true, 'h0', h_trial_before, ...
        'prev0', prev_trial_before, 'x_norm', x_norm, 'v_norm', v_norm, ...
        'a_norm', a_norm, 'T_norm', T_norm, 'cmpRD', cmpRD);
end
if nargout > 5
    trace = trial_trace(netF, x_norm, v_norm, a_norm, T_norm, ...
        h_trial_before, prev_trial_before, 10);
end

F0_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, T_norm, ...
    0, h_trial_before, prev_trial_before, ntrial);
F16_rig = trial_mean_force(netF, x_norm, v_norm, a_norm, T_norm, ...
    1.0, h_trial_before, prev_trial_before, ntrial);
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
            0,h_trial_before,prev_trial_before,ntrial);
        fp16=-trial_mean_force(netF,x_norm,vp,a_norm,T_norm, ...
            1,h_trial_before,prev_trial_before,ntrial);
        if isfinite(fp0) && isfinite(fp16)
            l=min(fp0,fp16); h=max(fp0,fp16);
            if cmpRD_preview(j)*cmpRD<0
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
    T_norm, cand_norm, h0, prev0, ntrial)
%#codegen
h = h0;
prev_I = prev0;
acc = 0;
for s = 1:ntrial
    [Fn, h, prev_I] = fwd_step_gen(netF, x_norm, v_norm, a_norm, ...
        T_norm, cand_norm, h, prev_I);
    acc = acc + Fn;
end
Fmean_rig = (acc / ntrial) * netF.norm.f_scale;
end

function tr = trial_trace(netF, x_norm, v_norm, a_norm, T_norm, h0, prev0, n)
% 10 步逐步力，两列：[0 A, 1.6 A]，rig 符号，单位 N
%#codegen
tr = zeros(n, 2);
for c = 1:2
    cand = double(c - 1);
    h = h0; prev_I = prev0; acc = zeros(n, 1);
    for s = 1:n
        [Fn, h, prev_I] = fwd_step_gen(netF, x_norm, v_norm, a_norm, ...
            T_norm, cand, h, prev_I);
        acc(s) = Fn;
    end
    tr(:, c) = acc * netF.norm.f_scale;
end
end

function y = sigmoid(x)
%#codegen
y = 1 ./ (1 + exp(-x));
end
