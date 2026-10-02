function tests = test_pmpc_speed_coordinator
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testStraightKeepsTarget(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('straight');
[plan,next] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,20);
verifyTrue(testCase,plan.valid);
verifySize(testCase,plan.Vx_pred,[mpc.Np 1]);
verifySize(testCase,plan.diag,[8 1]);
verifyEqual(testCase,plan.Fx_dem,0,'AbsTol',1e-12);
verifyEqual(testCase,plan.Vset_pid,80,'AbsTol',1e-12);
verifyEqual(testCase,next.Fx_prev,0,'AbsTol',1e-12);
end

function testSustainedR69BrakesBeforeBend(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('jturn');
[plan,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,47);
verifyTrue(testCase,plan.valid);
verifyGreaterThan(testCase,plan.Fx_dem,0);
verifyLessThan(testCase,plan.diag(1),state.x_dot);
verifyGreaterThan(testCase,plan.diag(2),0); % reachable brake authority
verifyLessThanOrEqual(testCase,plan.Fx_dem,limits.Long_F_slew*mpc.Ts_exec+1e-9);
verifyLessThanOrEqual(testCase,plan.Vset_pid,state.VxTarget);
end

function testFrictionBoundaryStopsLowMuPreview(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = func_WayPoints(6,69,false);
vehicle.mu = 0.5;
vehicle.mu_eff = 0.855*vehicle.mu;
state.x_dot = 90/3.6;
state.VxTarget = 90;
prev.Vset_prev = 90;
straight = W(:,2) <= 280;
switchStation = interp1(W(straight,2),W(straight,7),230);
projection = struct('WPIndex',201,'PrjP', ...
    struct('ey',0,'epsi',0,'Velr',state.x_dot,'xr',200,'yr',0,'psir',0), ...
    's0',200,'valid',true);
[plan,~] = func_PMPCSpeedCoordinator(mpc,vehicle,limits,W, ...
    projection,state,loads,0.5,prev,switchStation);
verifyEqual(testCase,plan.Fx_dem,0,'AbsTol',1e-9);
verifyEqual(testCase,plan.Vset_pid,90,'AbsTol',1e-9);
end

function testLateralCapRetainsOriginalSafetyReserve(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
state.x_dot = 90/3.6;
state.VxTarget = 90;
prev.Vset_prev = 90;
W = road('jturn');
[plan,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,100);
expected = sqrt(vehicle.mu_eff*vehicle.g*limits.Long_mu_reserve*69);
verifyEqual(testCase,plan.diag(1),expected,'AbsTol',1e-6);
end

function testSustainedBendCapDoesNotDisappearBelowReleaseSpeed(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('jturn');
capKmh = 3.6*sqrt(vehicle.mu_eff*vehicle.g*limits.Long_mu_reserve*69);
prev.Vset_prev = capKmh;
for speedKmh = [72 73 73.8]
    state.x_dot = speedKmh/3.6;
    [plan,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,100);
    verifyEqual(testCase,plan.diag(1)*3.6,capKmh,'AbsTol',1e-6);
    verifyEqual(testCase,plan.Vset_pid,capKmh,'AbsTol',1e-6);
    verifyEqual(testCase,plan.Fx_dem,0,'AbsTol',1e-12);
end
end

function testShortDlcBendDoesNotBrakeWhenReachable(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('short_bend');
[plan,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,47);
verifyTrue(testCase,plan.valid);
verifyEqual(testCase,plan.Fx_dem,0,'AbsTol',1e-12);
verifyEqual(testCase,plan.diag(1),state.x_dot,'AbsTol',1e-12);
end

function testActualDlcAlternatingLobesDoNotBrake(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
vehicle.mu = 0.5;
vehicle.mu_eff = 0.855*vehicle.mu;
limits.Long_min_sustain_m = 30;
W = func_WayPoints(1,69,false);
for station = [0 50 65 80 110 130]
    [plan,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,-0.5,station);
    verifyTrue(testCase,plan.valid);
    verifyEqual(testCase,plan.Fx_dem,0,'AbsTol',1e-12);
end
end

function testLowerMuNeverRaisesSpeedCap(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('jturn');
[high,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,47);
vehicle.mu = 0.5;
vehicle.mu_eff = 0.855*vehicle.mu;
[low,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,47);
verifyLessThanOrEqual(testCase,low.diag(1),high.diag(1)+1e-9);
verifyLessThanOrEqual(testCase,low.diag(2),high.diag(2)+1e-9);
end

function testLowBrakeAuthorityFlagsUnreachable(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('jturn');
loads.Fz_l1 = 1; loads.Fz_r1 = 1;
loads.Fz_l2 = 1; loads.Fz_r2 = 1;
[plan,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,47);
verifyTrue(testCase,plan.valid);
verifyEqual(testCase,plan.diag(7),1);
verifyLessThanOrEqual(testCase,plan.Fx_dem,plan.diag(2)*vehicle.m+1e-9);
end

function testForceSlewAndRecovery(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
prev.Fx_prev = 5000;
prev.Vset_prev = 72;
prev.trigger = true;
W = road('straight');
[plan,next] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,20);
verifyGreaterThanOrEqual(testCase,plan.Fx_dem, ...
    prev.Fx_prev-limits.Long_F_slew*mpc.Ts_exec-1e-9);
verifyLessThanOrEqual(testCase,plan.Fx_dem,prev.Fx_prev+1e-9);
verifyGreaterThanOrEqual(testCase,plan.Vset_pid,prev.Vset_prev);
verifyLessThanOrEqual(testCase,plan.Vset_pid, ...
    prev.Vset_prev+3.6*limits.Long_VupRate*mpc.Ts_exec+1e-9);
verifyEqual(testCase,next.Fx_prev,plan.Fx_dem,'AbsTol',1e-12);
end

function testInvalidInputFallsBack(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('jturn');
projection = struct('WPIndex',1,'PrjP', ...
    struct('ey',0,'epsi',0,'Velr',state.x_dot,'xr',0,'yr',0,'psir',0), ...
    's0',47,'valid',false);
[plan,~] = func_PMPCSpeedCoordinator( ...
    mpc,vehicle,limits,W,projection,state,loads,0.5,prev);
verifyFalse(testCase,plan.valid);
verifyEqual(testCase,plan.Fx_dem,0,'AbsTol',1e-12);
verifyEqual(testCase,plan.Vset_pid,state.VxTarget,'AbsTol',1e-12);
verifyEqual(testCase,plan.Vx_pred,state.x_dot*ones(mpc.Np,1),'AbsTol',1e-12);
verifyEqual(testCase,plan.diag(8),1);
end

function testUnachievedBrakeNotCredited(testCase)
[mpc,vehicle,limits,state,loads,prev] = fixture();
W = road('jturn');
[fullCredit,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,47);
prev.achievedRatio = 0;
[noCredit,~] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,0.5,47);
verifyGreaterThan(testCase,fullCredit.Fx_dem,0);
verifyEqual(testCase,noCredit.Fx_dem,fullCredit.Fx_dem,'AbsTol',1e-9);
verifyEqual(testCase,noCredit.Vx_pred, ...
    state.x_dot*ones(mpc.Np,1),'AbsTol',1e-12);
verifyLessThan(testCase,fullCredit.Vx_pred(end),state.x_dot);
end

function [plan,next] = runPlanner(mpc,vehicle,limits,W,state,loads,prev,margin,s0)
projection = struct('WPIndex',max(1,round(s0)+1), ...
    'PrjP',struct('ey',0,'epsi',0,'Velr',state.x_dot, ...
    'xr',s0,'yr',0,'psir',0), 's0',s0,'valid',true);
[plan,next] = func_PMPCSpeedCoordinator( ...
    mpc,vehicle,limits,W,projection,state,loads,margin,prev);
end

function [mpc,vehicle,limits,state,loads,prev] = fixture()
mpc = struct('Np',20,'Ts',0.05,'Ts_exec',0.01,'first_Tc',false);
vehicle = struct('m',1860,'mu',0.85,'mu_eff',0.855*0.85,'g',9.81,'rt',0.347);
limits = struct('LongCoordMode',1,'Tb_max',3000,'Long_mu_reserve',0.85, ...
    'Long_brake_reserve',0.7,'Long_a_max',3,'Long_preview_nodes',80, ...
    'Long_min_sustain_m',30,'Long_delay',0.15,'Long_tau',0.35, ...
    'Long_F_slew',2e5,'Long_VdownRate',4,'Long_VupRate',1.5, ...
    'Long_margin_trigger',0.2,'Long_margin_recover',0.35, ...
    'Long_trigger_band',0.1,'Long_release_band',0.4);
state = struct('x_dot',80/3.6,'VxTarget',80);
loads = struct('Fz_l1',4800,'Fz_r1',4800,'Fz_l2',4300,'Fz_r2',4300);
prev = struct('Fx_prev',0,'Vset_prev',80,'trigger',false, ...
    'release_ticks',0,'allocationFailed',false,'achievedRatio',1);
end

function W = road(kind)
s = (0:200)';
W = zeros(numel(s),9);
W(:,1) = (1:numel(s))';
W(:,2) = s;
W(:,7) = s;
if strcmp(kind,'jturn')
    W(s>=60 & s<=160,5) = 1/69;
elseif strcmp(kind,'short_bend')
    W(s>=60 & s<=62,5) = 0.04;
end
end
