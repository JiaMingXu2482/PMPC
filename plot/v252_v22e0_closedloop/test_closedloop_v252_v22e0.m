%TEST_CLOSEDLOOP_V252_V22E0  Closed-loop test for the diagram in the task.
%   Run in MATLAB after cd to this folder:
%     test_closedloop_v252_v22e0
%   No Simulink needed. Single corner (cid = 1), fixed 1 ms step.
%
%   Chain (exactly as in the diagram):
%     F_des -> inv_mf (V25.2, 10 ms solver) -> I_cmd
%           -> curr_dyn_mf (rise 10 ms / fall 19 ms, 10-90%)
%           -> I_act -> fwd_mf (V22 E0, 1 kHz) -> F_pred
%           -> compare F_pred with F_des
%
%   CORRECTION (2026-10-31, after the first version of this script):
%   The first version set F_des = 300*sin(2*pi*1.66*t), i.e. in phase
%   with displacement. That is NOT achievable by a damper: damper
%   force is (to first order) in phase with velocity, not displacement.
%   In the Python reproduction of that version, corr(F_pred, cmpRD)
%   = 0.989, corr(F_pred, cmpD) = 0.017, while corr(F_des, cmpRD)
%   = -0.028, F_pred range -525 to +781 N vs F_des -300 to +300 N,
%   I_cmd only 0 to 0.380 A, MAE 443 N. That plot (F_pred not tracking
%   F_des, current stuck low) is exactly what an unachievable,
%   displacement-phase F_des produces, not a weight or wiring bug:
%   at high velocity even 0 A already gives ~500-800 N, so the inverse
%   saturates at 0 A and still cannot get down to 300 N.
%
%   This corrected version generates F_des from the forward model
%   itself, so F_des is achievable by construction:
%     Step 1 (open loop): I_ref = 0.6 + 0.4*sin(2*pi*0.5*t) A
%       -> curr_dyn -> I_act_ref -> fwd_mf -> F_ref
%     Step 2 (closed loop, all states cleared first):
%       F_des = F_ref, same motion, same Temp
%       -> inv_mf -> curr_dyn -> fwd_mf -> F_pred
%   Any remaining F_pred vs F_des error in Step 2 is then purely
%   inverse + dynamics + free-running error, not a passivity violation.
%   Python reproduction of this version: F_pred vs F_ref MAE 39 N
%   (vs 443 N for the old F_des), F_pred range -472 to +779 N vs
%   F_ref -494 to +779 N. I_cmd vs I_ref MAE is still ~0.30 A
%   (inverse compresses the current range), but force error is
%   much smaller because force is less sensitive to current at
%   low velocity. Do NOT generalise this 39 N to ST steps:
%   the Python free-running validation on real files gave 133 N
%   pooled, with ST/SH failures in the 300-1600 N range.
%
%   Motion (same in both steps, synthetic, not a validation file):
%     cmpD  = 10*sin(2*pi*1.66*t) mm, CarSim sign (compression +)
%     cmpRD = derivative of cmpD, Temp = 30 degC
%
%   Reset: this script clears fwd_mf_impl, inv_mf_impl_10ms and
%   curr_dyn_impl before EACH step, so Step 2 starts from clean state
%   and does not inherit Step 1's GRU history.

dt = 0.001;
Tend = 3;
t = (0:dt:Tend)';
N = numel(t);
Temp_C = 30;
cid = 1;

cmpD = 10*sin(2*pi*1.66*t);
cmpRD = 10*2*pi*1.66*cos(2*pi*1.66*t);

% ---- Step 1: reference force from a reference current ----
clear fwd_mf_impl inv_mf_impl_10ms curr_dyn_impl
I_ref = 0.6 + 0.4*sin(2*pi*0.5*t);
I_act_ref = zeros(N, 1);
F_ref = zeros(N, 1);
for k = 1:N
    I_act_ref(k) = curr_dyn_mf(t(k), I_ref(k), cid);
    F_ref(k) = fwd_mf(t(k), Temp_C, cmpD(k), cmpRD(k), I_act_ref(k), cid);
end
Fd = F_ref;  % achievable by construction

% ---- Step 2: closed loop with F_des = F_ref ----
clear fwd_mf_impl inv_mf_impl_10ms curr_dyn_impl
I_cmd = zeros(N, 1);
I_act = zeros(N, 1);
F_pred = zeros(N, 1);
Flo = zeros(N, 1);
Fhi = zeros(N, 1);
F_ach = zeros(N, 1);
for k = 1:N
    [I_cmd(k), Flo(k), Fhi(k), ~, F_ach(k)] = ...
        inv_mf(Fd(k), t(k), Temp_C, cmpD(k), cmpRD(k), cid);
    I_act(k) = curr_dyn_mf(t(k), I_cmd(k), cid);
    F_pred(k) = fwd_mf(t(k), Temp_C, cmpD(k), cmpRD(k), I_act(k), cid);
end

% Evaluation: ignore first 1 s (warmup / guard, same intent as Python
% validation, which masked the first 1000 samples).
m = t >= 1;
e_same = F_pred(m) - Fd(m);
idx = find(m);
idx_s = idx(11:end);
idx_d = idx(1:end-10);
e_shift = F_pred(idx_s) - Fd(idx_d);

fprintf('Closed-loop test, achievable F_des = F_ref(I_ref)\n');
fprintf('  F_ref range %.0f to %.0f N\n', min(F_ref(m)), max(F_ref(m)));
fprintf('  F_pred vs F_des, same tick:  MAE %.1f N, RMSE %.1f N\n', ...
    mean(abs(e_same)), sqrt(mean(e_same.^2)));
fprintf('  F_pred(k+1) vs F_des(k):      MAE %.1f N, RMSE %.1f N\n', ...
    mean(abs(e_shift)), sqrt(mean(e_shift.^2)));
fprintf('  I_cmd vs I_ref: MAE %.3f A, I_cmd range %.3f to %.3f A\n', ...
    mean(abs(I_cmd(m) - I_ref(m))), min(I_cmd(m)), max(I_cmd(m)));
fprintf('  I_act: mean %.3f A\n', mean(I_act(m)));
fprintf('  F_ach (trial, NOT plant F_pred): MAE vs F_des %.1f N\n', ...
    mean(abs(F_ach(m) - Fd(m))));
if any(~isfinite(F_pred)) || any(~isfinite(I_cmd))
    fprintf('  WARNING: non-finite output detected - check wiring / reset.\n');
else
    fprintf('  No non-finite outputs. Plots below.\n');
end

figure('Name', 'V25.2+V22E0 closed loop (achievable F_des)');
subplot(2, 1, 1);
plot(t, Fd, 'k', t, F_pred, 'b', t, F_ach, 'r--');
legend('F\_des (=F\_ref)', 'F\_pred (plant)', 'F\_ach (trial)');
ylabel('Force (N)'); title('Closed loop: force');
grid on;
subplot(2, 1, 2);
plot(t, I_ref, 'k--', t, I_cmd, 'b', t, I_act, 'r');
legend('I\_ref', 'I\_cmd (V25.2)', 'I\_act (curr dyn)');
ylabel('Current (A)'); xlabel('Time (s)'); title('Closed loop: current');
grid on;
