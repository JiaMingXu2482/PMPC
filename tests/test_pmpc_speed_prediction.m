function tests = test_pmpc_speed_prediction
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testConstantScheduleMatchesLegacyMatrices(testCase)
for nx = [6 7]
    [vehicle,mpc,state] = modelFixture(nx);
    mpc.first_Tc = true;
    legacy = func_DynamicalModel(vehicle,mpc,state,1);
    scheduled = func_DynamicalModel(vehicle,mpc,state,1,[], ...
        state.x_dot*ones(mpc.Np,1));
    verifyEqual(testCase,scheduled.A_aug,legacy.A_aug,'AbsTol',1e-12);
    verifyEqual(testCase,scheduled.B_aug,legacy.B_aug,'AbsTol',1e-12);
    verifyEqual(testCase,scheduled.D_aug,legacy.D_aug,'AbsTol',1e-12);
end
end

function testDecreasingSpeedChangesFutureNodesNotFirstState(testCase)
[vehicle,mpc,state] = modelFixture(6);
mpc.first_Tc = true;
speed = [20;18;16;14;12];
legacy = func_DynamicalModel(vehicle,mpc,state,1);
scheduled = func_DynamicalModel(vehicle,mpc,state,1,[],speed);
verifyEqual(testCase,scheduled.A_aug(:,:,1),legacy.A_aug(:,:,1),'AbsTol',1e-12);
verifyEqual(testCase,scheduled.A_aug(5,6,1),0.2,'AbsTol',1e-12);
verifyEqual(testCase,scheduled.A_aug(5,6,5),0.6,'AbsTol',1e-12);
verifyNotEqual(testCase,scheduled.A_aug(:,:,5),legacy.A_aug(:,:,5));
end

function testBothLtvBranchesHaveFixedFiniteDimensions(testCase)
for nx = [6 7]
    [vehicle,mpc,state] = modelFixture(nx);
    speed = [20;18;16;14;12];
    frozen = func_DynamicalModel(vehicle,mpc,state,1,[],speed);
    ltv = func_DynamicalModel(vehicle,mpc,state,1,ltvFixture(nx,mpc),speed);
    for model = {frozen,ltv}
        value = model{1};
        verifySize(testCase,value.A_aug,[nx+mpc.Nu,nx+mpc.Nu,mpc.Np]);
        verifySize(testCase,value.B_aug,[nx+mpc.Nu,mpc.Nu,mpc.Np]);
        verifySize(testCase,value.D_aug,[nx+mpc.Nu,3,mpc.Np]);
        verifyTrue(testCase,all(isfinite(value.A_aug(:))));
        verifyTrue(testCase,all(isfinite(value.B_aug(:))));
        verifyTrue(testCase,all(isfinite(value.D_aug(:))));
    end
    verifyEqual(testCase,ltv.A_aug(5,6,5),0.6,'AbsTol',1e-12);
end
end

function testCurvatureAndVelocityUseSameStations(testCase)
[vehicle,mpc,state] = modelFixture(6);
vehicle.L = vehicle.lf+vehicle.lr;
vehicle.CafHat = 90000; vehicle.CarHat = 95000;
vehicle.mu = 0.85;
s = (0:100)';
W = zeros(numel(s),9);
W(:,1) = (1:numel(s))'; W(:,2) = s;
W(:,4) = s.^2/2000; W(:,5) = s/1000; W(:,7) = s;
state.X = 20+vehicle.lf; state.Y = 0; state.Yaw = 0;
projection = func_PathProjection(vehicle,15,W,state);
verifyTrue(testCase,projection.valid);
speed = [20;18;16;14;12];
[~,~,kap] = func_RefTraj_LocalPlanning( ...
    mpc,vehicle,15,W,state,struct(),projection,speed);
model = func_DynamicalModel(vehicle,mpc,state,1,[],speed);
[~,~,gamma] = func_SystemFurture(mpc,model,kap,[0;0]);
verifyEqual(testCase,gamma(1:3:end),kap,'AbsTol',1e-12);
verifyEqual(testCase,reshape(model.D_aug(6,1,:),[],1),-0.05*speed,'AbsTol',1e-12);
verifyEqual(testCase,sum(0.05*speed.*kap),0.088,'AbsTol',1e-10);
end

function [vehicle,mpc,state] = modelFixture(nx)
vehicle = struct('m',1860,'ms',1500,'g',9.81,'lf',1.2466, ...
    'lr',1.5034,'Ix',894.4,'Iz',2800,'h_S2R',0.57,'Kt',45000, ...
    'CbarF',-90000,'CbarR',-95000);
mpc = struct('Np',5,'Nc',3,'Nx',nx,'Nu',3,'Ny',nx, ...
    'Ts',0.05,'Ts_exec',0.01,'first_Tc',false,'tau_d',0.015);
state = struct('x_dot',20,'delta_f',0.01);
end

function ltv = ltvFixture(nx,mpc)
tire = struct('fzlo',1,'fzhi',20000,'cq',[0 0 10], ...
    'mq',[0 0 0.8],'kcs',0,'C0',200000);
ltv = struct('TireF',tire,'TireR',tire,'delta_robot',0, ...
    'Fzf',9000,'Fzr',9000,'kap',zeros(mpc.Np,1), ...
    'x0',zeros(nx,1),'u0',zeros(mpc.Nu,1), ...
    'dU',zeros(mpc.Nu,mpc.Nc),'Ctan_floor_frac',0.1);
end
