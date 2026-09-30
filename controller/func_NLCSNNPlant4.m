function [F_plant, h_new, a_filt_new, v_prev_new, init_new, ...
          x_plant, v_plant, a_plant] = ...
    func_NLCSNNPlant4(net, cmpD, cmpRD, i_cmd, h, a_filt, v_prev, ...
                      init_done, cfg)
%FUNC_NLCSNNPLANT4  NLCSNN damper plant, four corners, 1 ms step.
%   %#codegen compatible (no file I/O, no persistent state).
%
%   This is the damper PLANT for MIL simulation. It runs at the CarSim
%   math-model rate (1000 Hz) and replaces func_MRDamper + the first-order
%   actuator lag. The current command i_cmd comes from the 100 Hz
%   controller and is held constant (ZOH) between controller ticks. The
%   hidden state h is the TRUE damper state, integrated at the training
%   step (1 ms) with true 1 ms inputs -- no zero-order-hold approximation
%   on x/v/a (the old 10 ms macro step had up to ~7.9% RMS force error on
%   10 mm/5 Hz inputs).
%
%   Simulink wiring: ONE MATLAB Function block with Sample time 0.001,
%   plus four Unit Delays outside the block (all Sample time 0.001):
%     h         8x4     hidden state,            init zeros(8,4)
%     a_filt    4x1     filtered accel [mm/s^2], init zeros(4,1)
%     v_prev    4x1     prev velocity [mm/s],    init zeros(4,1)
%     init_done scalar   0/1 first-step guard,    init 0
%   Feed each delay output into h/a_filt/v_prev/init_done and wire the
%   *_new outputs back into the delays. See SLX_CHECKLIST.md.
%
%   Corner order convention:
%     cmpD/cmpRD in CarSim export order [L1;L2;R1;R2] (straight from CarSim,
%       no reorder block needed in the .slx).
%     i_cmd/h/a_filt/v_prev in controller order [L1;R1;L2;R2]
%       (straight from pmpc_block, matches the old ctx ordering).
%     F_plant returned in CarSim order [L1;L2;R1;R2] (straight to CarSim).
%     h_new/a_filt_new/v_prev_new/x_plant/v_plant/a_plant in controller
%       order (x_plant etc. are sampled 1 kHz -> 100 Hz into pmpc_block).
%
%   Inputs (physical units):
%     net       struct from nlcsnn_damper_init (weights + normalization)
%     cmpD      4x1 compression displacement [mm], CarSim sign (compression +)
%     cmpRD     4x1 compression velocity [mm/s], CarSim sign (compression +)
%     i_cmd     4x1 current command [A], controller order, ZOH from 100 Hz
%     h         8x4 hidden state, controller order
%     a_filt    4x1 filtered accel [mm/s^2], controller order, rebound +
%     v_prev    4x1 previous velocity [mm/s], controller order, rebound +
%     init_done scalar 0/1, 1 after the first plant step
%     cfg       struct with fields:
%                 x_ref     rig position reference [mm] (281.0645)
%                 temp      damper temperature [degC]
%                 i_max     max current [A] (1.6)
%                 dt_plant  plant step [s] (0.001)
%                 tau_accel accel filter time constant [s] (0.02)
%
%   Outputs:
%     F_plant      4x1 damper force [N], CarSim sign (tension +), CarSim order
%     h_new        8x4 updated hidden state, controller order
%     a_filt_new  4x1 updated filtered accel, controller order
%     v_prev_new  4x1 updated previous velocity, controller order
%     init_new     scalar 1 (latches the first-step guard)
%     x_plant      4x1 rig position [mm], controller order
%     v_plant      4x1 velocity [mm/s] rebound +, controller order
%     a_plant      4x1 filtered accel [mm/s^2] rebound +, controller order
%
%   Polarity: F_CarSim = -F_NLCSNN (verified 2026-09-29: 94.8% sign
%   agreement vs the legacy table on 824 samples; current direction
%   verified against raw rig CSVs: 0 A hardest -> 1.6 A softest).

%#codegen

dt = cfg.dt_plant;   % 0.001 s

cmpD  = reshape(cmpD, 4, 1);
cmpRD = reshape(cmpRD, 4, 1);

% ---- corner reorder: CarSim [L1;L2;R1;R2] -> internal [L1;R1;L2;R2] ----
% NOTE: only cmpD/cmpRD need reordering. h/a_filt/v_prev/i_cmd already
% arrive in controller order [L1;R1;L2;R2], which IS the internal order.
o = [1 3 2 4];
cmpD_i  = cmpD(o);
cmpRD_i = cmpRD(o);
h_i     = h;
a_i     = a_filt;
vp_i    = v_prev;

% ---- non-finite guards (same policy as the old 10 ms context) ----
for j = 1:4
    if ~isfinite(cmpD_i(j))
        cmpD_i(j) = 0;
    end
    if ~isfinite(cmpRD_i(j))
        if init_done > 0.5 && isfinite(vp_i(j))
            cmpRD_i(j) = -vp_i(j);   % hold last velocity (CarSim sign)
        else
            cmpRD_i(j) = 0;
        end
    end
end

% ---- kinematics: CarSim (compression +) -> NLCSNN (rebound +) ----
x = cfg.x_ref - cmpD_i;   % mm, rig absolute position
v = -cmpRD_i;             % mm/s

% ---- acceleration: 1 ms finite difference + first-order lag ----
if init_done > 0.5
    a_raw = (v - vp_i) / dt;
else
    a_raw = zeros(4,1);   % no differentiation spike on the first step
end
for j = 1:4
    if ~isfinite(a_raw(j))
        a_raw(j) = 0;
    end
end
tau = max(cfg.tau_accel, eps);
alpha = 1 - exp(-dt / tau);
a_new = (1 - alpha) * a_i + alpha * a_raw;
for j = 1:4
    if ~isfinite(a_new(j))
        a_new(j) = 0;
    end
end

% ---- per-corner forward step at 1 ms ----
F_int = zeros(4,1);
h_int = zeros(8,4);
for j = 1:4
    ij = min(max(i_cmd(j), 0), cfg.i_max);
    if ~isfinite(ij)
        ij = 0;
    end
    % dcurr is a dead input (build_nfl does not use di/dt); pass 0.
    [F_nl, hj] = nlcsnn_damper_step(net, x(j), v(j), a_new(j), ...
                                    ij, 0, cfg.temp, dt, h_i(:,j));
    if ~isfinite(F_nl) || any(~isfinite(hj))
        % fail-safe: freeze state, report force from the frozen state
        hj = h_i(:,j);
        F_nl = nlcsnn_predict_force(net, x(j), v(j), a_new(j), ...
                                    ij, cfg.temp, hj);
        if ~isfinite(F_nl)
            F_nl = 0;
        end
    end
    F_int(j) = -F_nl;      % NLCSNN -> CarSim polarity
    h_int(:,j) = hj;
end

% ---- reorder back to external conventions ----
F_plant = zeros(4,1);
F_plant(o) = F_int;       % -> CarSim order [L1;L2;R1;R2]
h_new = h_int;            % controller order [L1;R1;L2;R2]
a_filt_new = a_new;
v_prev_new = v;
init_new = 1;
x_plant = x;
v_plant = v;
a_plant = a_new;
end
