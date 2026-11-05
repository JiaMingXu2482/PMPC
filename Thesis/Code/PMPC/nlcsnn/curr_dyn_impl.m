function I_act = curr_dyn_impl(t, I_cmd, cid)
%CURR_DYN_IMPL  Current-dynamics simulation block, ONE corner.
%   %#codegen compatible.
%
%   I_act = curr_dyn_impl(t, I_cmd, cid)
%
%   This block is NOT part of V25.2 or V22 E0. It is the middle block
%   in the user's closed-loop diagram: inverse output I_cmd -> this
%   block -> actual current estimate I_act -> forward model.
%
%   Model: first-order lag with direction-dependent time constant,
%     dI_act/dt = (I_cmd - I_act) / tau,  tau = tau_rise if I_cmd > I_act
%     (current increasing), tau = tau_fall if I_cmd < I_act (decreasing).
%   tau is derived from the measured 10%-90% step times reported on
%   2026-10-31 (all 33 ST files, 1 kHz, median):
%     rise (increasing)  t10-90 = 10.0 ms  -> tau_rise = 10.0 / ln(9) = 4.55 ms
%     fall (decreasing)  t10-90 = 19.0 ms  -> tau_fall = 19.0 / ln(9) = 8.65 ms
%   Exact discrete update per call: I_act <- I_act + (1-exp(-dt/tau))
%   * (I_cmd - I_act), with dt from t (clamped to [0, 0.05] s for safety).
%   This reproduces the measured 10-90% times for a step, by construction.
%   It does NOT reproduce overshoot (ST files show overshoot to ~1.92 A
%   before clipping to 1.6 A in the proxy label), nor the driver slope
%   asymmetry beyond the two taus. It is a simulation approximation,
%   not a measured driver model.
%
%   WARNING - double-counting: V25.2 training already assumed
%   I_act[k] ~= I_cmd[k-1] (one 10 ms step). If you put this block
%   between V25.2 and V22 E0 in closed loop, the total lag from I_cmd
%   to force is (V25.2 one-step definition) + (this block, ~10/19 ms)
%   + (V22 E0's own 1 ms I_prev shift). That is the diagram as drawn,
%   and it is the correct physical chain IF V25.2's output is read as
%   "command issued at tick k" (not "actual current at tick k").
%   Do NOT also shift I_cmd by one step manually on top of this block:
%   that would be a second, spurious 10 ms delay. Compare F_pred at
%   step k+1 with F_des at step k in closed-loop evaluation, not
%   F_pred and F_des at the same tick.
%
%   Inputs (all scalars):
%     t      simulation time [s]
%     I_cmd  current command [A], clamped to [0,1.6] inside
%     cid    corner id: 1=L1, 2=L2, 3=R1, 4=R2 (constant per block)
%
%   Output:
%     I_act  estimated actual current [A], [0,1.6] (state is clamped
%            to [0,1.6] after each update; I_cmd outside that range
%            is clamped first, so overshoot above 1.6 A is not modelled)
%
%   Reset between runs: clear curr_dyn_impl   (resets all 4 corners)

%#codegen

persistent st_I t_prevs

NC = 4;
TAU_RISE = 0.010 / log(9);
TAU_FALL = 0.019 / log(9);

if isempty(st_I)
    st_I = zeros(1, NC);
    t_prevs = NaN(1, NC);
end

cid = max(1, min(NC, round(cid)));
I_cmd = min(max(I_cmd, 0), 1.6);

t_prev = t_prevs(cid);
if isnan(t_prev)
    dt = 0.001;
else
    dt = t - t_prev;
    if ~isfinite(dt) || dt < 0
        dt = 0.001;
    elseif dt > 0.05
        dt = 0.05;
    end
end
t_prevs(cid) = t;

I_act_prev = st_I(cid);
if I_cmd > I_act_prev
    tau = TAU_RISE;
else
    tau = TAU_FALL;
end
alpha = 1 - exp(-dt / tau);
I_act = I_act_prev + alpha * (I_cmd - I_act_prev);
I_act = min(max(I_act, 0), 1.6);
st_I(cid) = I_act;
end
