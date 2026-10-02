function [plan,next] = func_PMPCSpeedCoordinator( ...
    MPCParameters,VehiclePara,Constraints,W,Proj,VehStateMeasured, ...
    ParaHAT,margin_prev,prev,previewEndStation)
%FUNC_PMPCSPEEDCOORDINATOR Road-feasibility speed plan for PMPC only.
% All speed values are m/s internally; Fx_dem is a positive brake force.
% diag = [speed_cap; available_decel; road_margin; preview_length;
%         peak_curvature; reason; unreachable; fault].
if nargin < 10, previewEndStation = inf; end

Np = MPCParameters.Np;
vMeasured = VehStateMeasured.x_dot;
targetKmh = VehStateMeasured.VxTarget;
vFallback = vMeasured;
if ~isfinite(vFallback), vFallback = max(prev.Vset_prev/3.6,0); end
if ~isfinite(targetKmh), targetKmh = max(prev.Vset_prev,0); end
vFallback = max(vFallback,0);
targetKmh = max(targetKmh,0);
plan = struct('Vx_pred',vFallback*ones(Np,1), ...
    'Vset_pid',targetKmh,'Fx_dem',0,'diag',zeros(8,1),'valid',false);
next = prev;
plan.diag(1) = min(vFallback,targetKmh/3.6);

if ~Proj.valid || ~isfinite(Proj.s0) || ~isfinite(vMeasured) ...
        || ~isfinite(VehStateMeasured.VxTarget) || ~isfinite(margin_prev) ...
        || ~isfinite(VehiclePara.mu) || VehiclePara.mu <= 0 ...
        || ~isfinite(VehiclePara.m) || VehiclePara.m <= 0 ...
        || size(W,1) < 2 || size(W,2) < 7
    plan.diag(8) = 1;
    next.Fx_prev = 0;
    next.Vset_prev = targetKmh;
    next.trigger = false;
    next.release_ticks = 0;
    return;
end

mode = Constraints.LongCoordMode;
if mode == 0
    plan.valid = true;
    plan.diag(1) = min(vMeasured,targetKmh/3.6);
    next.Fx_prev = 0;
    next.Vset_prev = targetKmh;
    next.trigger = false;
    next.release_ticks = 0;
    return;
end

% Brake authority is based on current wheel loads and torque limits. A
% separate reserve leaves friction and brake torque available for yaw moment.
mu = VehiclePara.mu;
fWheel = [ParaHAT.Fz_l1;ParaHAT.Fz_r1;ParaHAT.Fz_l2;ParaHAT.Fz_r2];
if any(~isfinite(fWheel)) || ~isfinite(VehiclePara.rt) || VehiclePara.rt <= 0
    plan.diag(8) = 1;
    next.Fx_prev = 0;
    next.Vset_prev = targetKmh;
    next.trigger = false;
    next.release_ticks = 0;
    return;
end
Fmax = 0;
for j = 1:4
    Fmax = Fmax + min(max(fWheel(j),0)*mu*Constraints.Long_mu_reserve, ...
        Constraints.Tb_max/VehiclePara.rt);
end
aBrake = min(Constraints.Long_a_max, ...
    Constraints.Long_brake_reserve*Fmax/VehiclePara.m);
aBrake = max(aBrake,0);
plan.diag(2) = aBrake;

roadMargin = margin_prev;
measuredMargin = margin_prev;
if isfield(Constraints,'Roadwidth') && isfield(Constraints,'env_Wv') ...
        && isfield(Constraints,'env_es')
    measuredMargin = (Constraints.Roadwidth-Constraints.env_Wv)/2 ...
        - Constraints.env_es - abs(Proj.PrjP.ey);
    roadMargin = min(roadMargin,measuredMargin);
end
plan.diag(3) = roadMargin;

% The spatial horizon covers at least the physical stopping distance; the
% delayed actuation distance shifts its start before backward propagation.
v = max(vMeasured,0);
vTarget = targetKmh/3.6;
previewLength = max(40,v*v/(2*max(aBrake,0.25)) ...
    + v*Constraints.Long_delay + 10);
previewLength = min(previewLength,200);
% Current-mu planning must stop at a known friction boundary. Otherwise a
% future high-mu bend is falsely assessed using today's low-mu surface.
if isfinite(previewEndStation)
    previewLength = min(previewLength, ...
        max(previewEndStation-Proj.s0-v*Constraints.Long_delay-1e-3,0.1));
end
plan.diag(4) = previewLength;
nPreview = max(2,min(80,round(Constraints.Long_preview_nodes)));
ds = previewLength/(nPreview-1);
sStart = Proj.s0 + v*Constraints.Long_delay;
rawLimit = vTarget*ones(80,1);
active = false(80,1);
peakKappa = 0;
muEff = mu;
if isfield(VehiclePara,'mu_eff'), muEff = VehiclePara.mu_eff; end
aLatSafe = max(0,muEff*VehiclePara.g*Constraints.Long_mu_reserve);
for i = 1:nPreview
    station = sStart + (i-1)*ds;
    kappa = abs(local_interp(W(:,7),W(:,5),station));
    peakKappa = max(peakKappa,kappa);
    if kappa > 1e-9
        rawLimit(i) = min(vTarget,sqrt(aLatSafe/kappa));
    end
    % A sustained geometric speed cap must not vanish as soon as the car
    % slows to it. Compare with the scenario target, not measured speed.
    active(i) = rawLimit(i) < vTarget-Constraints.Long_release_band;
end
plan.diag(5) = peakKappa;

% A brief lane change can need high instantaneous lateral acceleration
% without being a sustained infeasible bend. Severe measured corridor loss
% shortens (but does not remove) persistence. Mode 2 omits this filter.
marginThreshold = Constraints.Long_margin_trigger;
if prev.trigger, marginThreshold = Constraints.Long_margin_recover; end
marginRisk = roadMargin < marginThreshold;
effectiveLimit = vTarget*ones(80,1);
runStart = 0;
for i = 1:nPreview+1
    isActive = false;
    if i <= nPreview, isActive = active(i); end
    if isActive && runStart == 0
        runStart = i;
    elseif ~isActive && runStart > 0
        runLength = (i-runStart)*ds;
        sustainNeeded = Constraints.Long_min_sustain_m;
        if marginRisk && measuredMargin < -0.25
            sustainNeeded = max(15,0.5*sustainNeeded);
        end
        if runLength >= sustainNeeded || mode == 2
            for j = runStart:i-1
                effectiveLimit(j) = rawLimit(j);
            end
        end
        runStart = 0;
    end
end

% Backward braking-distance pass: retain the highest speed at every node
% that can reach the next lower local cap under available deceleration.
cap = effectiveLimit;
for i = nPreview-1:-1:1
    cap(i) = min(cap(i),sqrt(max(0,cap(i+1)^2+2*aBrake*ds)));
end
speedCap = min(cap(1),vTarget);
plan.diag(1) = speedCap;
needBrake = speedCap < v-Constraints.Long_trigger_band;
reason = 0;
if needBrake
    reason = 1;
    if marginRisk, reason = 2; end
    if vTarget < v-Constraints.Long_release_band, reason = 3; end
end
plan.diag(6) = reason;

% A physically unreachable cap is still reported, but the force request
% never exceeds current brake authority. The next tick replans from actual Vx.
unreachable = 0;
if needBrake && aBrake < 0.05
    unreachable = 1;
end
for i = 1:nPreview
    if effectiveLimit(i) < v-Constraints.Long_release_band
        distance = max((i-1)*ds-v*Constraints.Long_delay,0);
        if v*v-effectiveLimit(i)^2 > 2*aBrake*distance+1e-6
            unreachable = 1;
        end
    end
end
plan.diag(7) = unreachable;

desiredForce = 0;
if needBrake
    aReq = max(0,(v-speedCap)/max(Constraints.Long_tau,0.01));
    desiredForce = VehiclePara.m*min(aReq,aBrake);
    next.trigger = true;
    next.release_ticks = 0;
elseif prev.trigger
    next.release_ticks = min(prev.release_ticks+1,1000);
    if next.release_ticks < 10
        desiredForce = prev.Fx_prev;
    else
        next.trigger = false;
    end
end
stepF = Constraints.Long_F_slew*MPCParameters.Ts_exec;
force = min(max(desiredForce,prev.Fx_prev-stepF),prev.Fx_prev+stepF);
force = min(max(force,0),VehiclePara.m*aBrake);
plan.Fx_dem = force;

% The PID target recovers slowly and never exceeds the scenario's target.
targetSpeedKmh = min(vTarget,speedCap)*3.6;
if targetSpeedKmh < prev.Vset_prev
    vset = max(targetSpeedKmh, ...
        prev.Vset_prev-3.6*Constraints.Long_VdownRate*MPCParameters.Ts_exec);
else
    vset = min(targetSpeedKmh, ...
        prev.Vset_prev+3.6*Constraints.Long_VupRate*MPCParameters.Ts_exec);
end
plan.Vset_pid = min(vset,targetKmh);

% Only deceleration that the allocator could execute is credited to the
% lateral prediction. An allocator failure disables that credit this tick.
aPred = force/VehiclePara.m*min(max(prev.achievedRatio,0),1);
if prev.allocationFailed, aPred = 0; end
t = 0;
for i = 1:Np
    dt = MPCParameters.Ts;
    if i == 1 && MPCParameters.first_Tc, dt = MPCParameters.Ts_exec; end
    t = t+dt;
    plan.Vx_pred(i) = max(0,v-aPred*t);
end

plan.valid = all(isfinite(plan.Vx_pred)) && isfinite(plan.Vset_pid) ...
    && isfinite(plan.Fx_dem) && all(isfinite(plan.diag));
if ~plan.valid
    plan.Vx_pred(:) = vFallback;
    plan.Vset_pid = targetKmh;
    plan.Fx_dem = 0;
    plan.diag(:) = 0;
    plan.diag(8) = 1;
end
next.Fx_prev = plan.Fx_dem;
next.Vset_prev = plan.Vset_pid;
end

function value = local_interp(stations,values,station)
n = numel(stations);
if station <= stations(1)
    value = values(1);
elseif station >= stations(n)
    value = values(n);
else
    lo = 1; hi = n;
    while hi-lo > 1
        mid = floor((lo+hi)/2);
        if stations(mid) <= station, lo = mid; else, hi = mid; end
    end
    value = values(lo)+(values(hi)-values(lo)) ...
        *(station-stations(lo))/(stations(hi)-stations(lo));
end
end
