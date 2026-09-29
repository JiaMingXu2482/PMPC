function [i_cmd, F_ach] = nlcsnn_force_to_current(net, x, v, a, F_des, temp, h, i_max, i_prev)
%NLCSNN_FORCE_TO_CURRENT  Inverse CDC damper model: desired force -> current.
%   %#codegen compatible (no file I/O, no persistent state).
%
%   [i_cmd, F_ach] = nlcsnn_force_to_current(net, x, v, a, F_des, temp, h, i_max, i_prev)
%
%   The PMPC controller outputs a desired damping force, but the physical
%   CDC damper takes current. This solves for the current i in [0, i_max]
%   whose NLCSNN-predicted force equals F_des.
%
%   Algorithm (robust to non-monotonic F-i curves):
%     1. Coarse-scan the current grid (9 pts over [0, i_max]), evaluate F.
%     2. Clip F_des to the achievable [min F, max F] (semi-active passivity:
%        an "active" force request saturates at the closest feasible value,
%        i.e. clipped-optimal behaviour).
%     3. Find grid intervals bracketing F_des; if several, take the one
%        closest to i_prev (continuity, avoids current jumps).
%     4. Local bisection (20 iters) inside that interval.
%   If current has (almost) no authority (v ~ 0), hold i_prev.
%
%   Inputs (physical units, scalars):
%     net    struct from nlcsnn_damper_init
%     x      damper displacement [mm]
%     v      damper velocity     [mm/s]
%     a      damper acceleration [mm/s^2]
%     F_des  desired damper force [N] (from PMPC / QP allocation)
%     temp   damper temperature  [degC]
%     h      hidden state, 8x1 (at START of step, same h as forward call)
%     i_max  current limit       [A] (your CDC hardware)
%     i_prev current applied at previous step [A] (Unit Delay on i_cmd)
%
%   Outputs:
%     i_cmd  current command [A], feed into nlcsnn_damper_step
%     F_ach  force the model predicts at i_cmd [N] (diagnostics)
%
%   Per-corner chain per step:
%     [i_cmd, ~] = nlcsnn_force_to_current(net,x,v,a,F_des,temp,h,i_max,i_prev)
%     dcurr      = (i_cmd - i_prev)/dt
%     [F, h_new] = nlcsnn_damper_step(net,x,v,a,i_cmd,dcurr,temp,dt,h)

%#codegen

% ---- 1. coarse current scan ----
Nc = 9;
i_grid = linspace(0, i_max, Nc);
F_grid = zeros(1, Nc);
for j = 1:Nc
    F_grid(j) = nlcsnn_predict_force(net, x, v, a, i_grid(j), temp, h);
end

Flo = min(F_grid);
Fhi = max(F_grid);
if (Fhi - Flo) < 1.0
    % Current has (almost) no authority here, e.g. v ~ 0: hold last current
    % to avoid chattering between arbitrary solutions.
    i_cmd = i_prev;
    F_ach = nlcsnn_predict_force(net, x, v, a, i_prev, temp, h);
    return;
end

% ---- 2. clip to achievable range ----
Fd = min(max(F_des, Flo), Fhi);

% ---- 3. find bracketing intervals ----
s = F_grid - Fd;
cross = zeros(1, Nc-1);
ncross = 0;
for j = 1:Nc-1
    if s(j) * s(j+1) <= 0
        ncross = ncross + 1;
        cross(ncross) = j;
    end
end

if ncross == 0
    % Safety fallback (should not happen after clipping): nearest grid point.
    [~, jk] = min(abs(s));
    i_cmd = i_grid(jk);
    F_ach = F_grid(jk);
    return;
end

% Several crossings (non-monotonic curve): pick the interval closest to
% i_prev for continuity.
best = cross(1);
bestd = abs(0.5 * (i_grid(best) + i_grid(best+1)) - i_prev);
for c = 2:ncross
    j = cross(c);
    d = abs(0.5 * (i_grid(j) + i_grid(j+1)) - i_prev);
    if d < bestd
        bestd = d;
        best = j;
    end
end

% ---- 4. local bisection inside the bracket ----
lo = i_grid(best);
hi = i_grid(best+1);
increasing = F_grid(best+1) > F_grid(best);
for k = 1:20
    mid = 0.5 * (lo + hi);
    Fm = nlcsnn_predict_force(net, x, v, a, mid, temp, h);
    if (increasing && Fm < Fd) || (~increasing && Fm > Fd)
        lo = mid;
    else
        hi = mid;
    end
end
i_cmd = 0.5 * (lo + hi);
F_ach = nlcsnn_predict_force(net, x, v, a, i_cmd, temp, h);
end
