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
%   Motion is a synthetic sine (not a validation file):
%     cmpD  = 10*sin(2*pi*1.66*t) mm, CarSim sign (compression +)
%     cmpRD = derivative of cmpD, Temp = 30 degC
%   Desired force is a synthetic sine, NOT a measured force:
%     F_des = 300*sin(2*pi*1.66*t) N, CarSim sign
%   This is a smoke test (no NaN, outputs in range, plots look sane),
%   NOT a validation: do not compare its MAE with the Python validation
%   numbers, which used real validation files and real F_des.
%
%   EXPECTED RESULT (read this before running):
%   The closed-loop MAE here will be LARGE (hundreds of N on some
%   segments). That is not a bug in this script. The Python
%   free-running validation (2026-10-31, V25_2_CASCADE_VALIDATION)
%   already showed V25.2 in free-running mode gives force MAE
%   133 N (best) / 131 N (ep120) vs 33 N for the forward model alone,
%   with ST/SH failures in the 300-1600 N range. This MATLAB chain
%   implements the same free-running mode, so it inherits the same
%   failure. If you see F_pred tracking F_des closely, suspect a
%   state-reset or wiring error (e.g. measured current leaking into
%   the inverse base), not a fix.
%
%   Reset: this script clears fwd_mf_impl, inv_mf_impl_10ms and
%   curr_dyn_impl at the start, so a second run starts from clean state.

clear fwd_mf_impl inv_mf_impl_10ms curr_dyn_impl

dt = 0.001;
Tend = 3;
t = (0:dt:Tend)';
N = numel(t);
Temp_C = 30;
cid = 1;

cmpD = 10*sin(2*pi*1.66*t);
cmpRD = 10*2*pi*1.66*cos(2*pi*1.66*t);
Fd = 300*sin(2*pi*1.66*t);

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
% Shifted comparison: F_pred at k+1 vs F_des at k (see curr_dyn_impl
% header: V25.2 output is a command for the next step, not the same tick).
e_shift = F_pred(m);
Fd_shift = Fd(m);
% Align by 10 samples (10 ms): F_pred(11:end) vs Fd(1:end-10), within mask.
idx = find(m);
idx_s = idx(11:end);
idx_d = idx(1:end-10);
e_shift = F_pred(idx_s) - Fd(idx_d);

fprintf('Closed-loop test (synthetic sine, NOT validation data)\n');
fprintf('  F_pred vs F_des, same tick:  MAE %.1f N, RMSE %.1f N\n', ...
    mean(abs(e_same)), sqrt(mean(e_same.^2)));
fprintf('  F_pred(k+1) vs F_des(k):      MAE %.1f N, RMSE %.1f N\n', ...
    mean(abs(e_shift)), sqrt(mean(e_shift.^2)));
fprintf('  I_cmd: mean %.3f A, frac at 1.6 A %.1f%%, frac at 0 A %.1f%%\n', ...
    mean(I_cmd(m)), 100*mean(I_cmd(m) == 1.6), 100*mean(I_cmd(m) == 0));
fprintf('  I_act: mean %.3f A\n', mean(I_act(m)));
fprintf('  F_ach (trial, NOT plant F_pred): MAE vs F_des %.1f N\n', ...
    mean(abs(F_ach(m) - Fd(m))));
if any(~isfinite(F_pred)) || any(~isfinite(I_cmd))
    fprintf('  WARNING: non-finite output detected - check wiring / reset.\n');
else
    fprintf('  No non-finite outputs. Plots below.\n');
end

figure('Name', 'V25.2+V22E0 closed loop (synthetic)');
subplot(2, 1, 1);
plot(t, Fd, 'k', t, F_pred, 'b', t, F_ach, 'r--');
legend('F\_des', 'F\_pred (plant)', 'F\_ach (trial)');
ylabel('Force (N)'); title('Closed loop: force');
grid on;
subplot(2, 1, 2);
plot(t, I_cmd, 'b', t, I_act, 'r');
legend('I\_cmd (V25.2)', 'I\_act (curr dyn)');
ylabel('Current (A)'); xlabel('Time (s)'); title('Closed loop: current');
grid on;
