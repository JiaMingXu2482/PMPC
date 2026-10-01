function tests = test_pmpc_speed_path
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testLegacyPathSameWithConstantSpeed(testCase)
[mpc,vehicle] = parameters();
for kind = 1:2
    [W,state,index,want] = pathCase(kind,vehicle);
    [ix,ref,kap,prj,node] = func_RefTraj_LocalPlanning( ...
        mpc,vehicle,index,W,state,struct());
    verifyEqual(testCase,ix,want.index);
    verifyEqual(testCase,[prj.ey,prj.epsi,prj.xr,prj.yr],want.projection,'AbsTol',1e-9);
    verifyEqual(testCase,[kap(1),kap(5),node(1),ref(1),ref(3)],want.preview,'AbsTol',1e-9);

    projection = func_PathProjection(vehicle,index,W,state);
    verifyTrue(testCase,projection.valid);
    [ix2,ref2,kap2,prj2,node2] = func_RefTraj_LocalPlanning( ...
        mpc,vehicle,index,W,state,struct(),projection,20*ones(mpc.Np,1));
    verifyEqual(testCase,ix2,ix);
    verifyEqual(testCase,ref2,ref,'AbsTol',1e-10);
    verifyEqual(testCase,kap2,kap,'AbsTol',1e-10);
    verifyEqual(testCase,node2,node,'AbsTol',1e-10);
    verifyEqual(testCase,[prj2.ey,prj2.epsi,prj2.xr,prj2.yr], ...
        [prj.ey,prj.epsi,prj.xr,prj.yr],'AbsTol',1e-10);
end
end

function testPlannedSpeedChangesArcSamples(testCase)
[mpc,vehicle] = parameters();
W = zeros(101,9);
s = (0:100)';
W(:,1) = (1:101)'; W(:,2) = s; W(:,3) = 1e-6*s.^3;
W(:,4) = 1.5e-4*s.^2; W(:,5) = s/10000; W(:,7) = s;
state = struct('x_dot',20,'Yaw',0,'X',21.2466,'Y',0);
projection = func_PathProjection(vehicle,15,W,state);
verifyTrue(testCase,projection.valid);
[~,~,~,~,fastNode] = func_RefTraj_LocalPlanning( ...
    mpc,vehicle,15,W,state,struct(),projection,20*ones(5,1));
[~,~,~,~,slowNode] = func_RefTraj_LocalPlanning( ...
    mpc,vehicle,15,W,state,struct(),projection,[20;15;10;5;5]);
verifyEqual(testCase,slowNode(1),fastNode(1),'AbsTol',1e-10);
verifyLessThan(testCase,slowNode(5),fastNode(5));
verifyEqual(testCase,fastNode(5)-slowNode(5),0.000225,'AbsTol',1e-6);
end

function testEndOfRoadIsFinite(testCase)
[mpc,vehicle] = parameters();
[W,state] = pathCase(2,vehicle);
state.X = W(end-1,2) + vehicle.lf*cos(W(end-1,4));
state.Y = W(end-1,3) + vehicle.lf*sin(W(end-1,4));
state.Yaw = W(end-1,4);
projection = func_PathProjection(vehicle,size(W,1)-3,W,state);
verifyTrue(testCase,projection.valid);
[~,ref,kap,~,node] = func_RefTraj_LocalPlanning( ...
    mpc,vehicle,size(W,1)-3,W,state,struct(),projection,20*ones(5,1));
verifyTrue(testCase,all(isfinite([ref;kap;node])));
verifyEqual(testCase,node(end),1/69,'AbsTol',1e-12);
end

function testNonMonotoneStationIsInvalid(testCase)
[~,vehicle] = parameters();
[W,state] = pathCase(2,vehicle);
W(20,7) = W(19,7);
projection = func_PathProjection(vehicle,10,W,state);
verifyFalse(testCase,projection.valid);
end

function [mpc,vehicle] = parameters()
mpc = struct('Np',5,'Ts',0.05,'Ts_exec',0.01,'first_Tc',false);
vehicle = struct('L',2.75,'m',1860,'lf',1.2466,'lr',1.5034, ...
    'mu',0.85,'g',9.81,'CafHat',90000,'CarHat',95000);
end

function [W,state,index,want] = pathCase(kind,vehicle)
if kind == 1
    data = load(fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        'data','WayPoints_Type1.mat'),'WayPoints_Collect');
    W = data.WayPoints_Collect;
    index = 140;
    psi = W(150,4);
    state = struct('x_dot',20,'Yaw',psi+0.01, ...
        'X',W(150,2)+vehicle.lf*cos(psi)+0.15, ...
        'Y',W(150,3)+vehicle.lf*sin(psi)-0.08);
    want.index = 150;
    want.projection = [-0.113140021160348,0.00632394547193532, ...
        74.6374249499666,0.619023982727784];
    want.preview = [0.0268319475345581,0.00944255502412902, ...
        0.0267166545741143,0.0864174982868997,0.416925];
else
    theta = (0:79)'*0.01;
    W = zeros(80,9);
    W(:,1) = (1:80)';
    W(:,2) = 69*sin(theta);
    W(:,3) = 69*(1-cos(theta));
    W(:,4) = theta;
    W(:,5) = 1/69;
    W(:,7) = 69*theta;
    index = 45;
    state = struct('x_dot',20,'Yaw',0.51, ...
        'X',69*sin(0.5)+vehicle.lf*cos(0.5)+0.1, ...
        'Y',69*(1-cos(0.5))+vehicle.lf*sin(0.5)-0.05);
    want.index = 51;
    want.projection = [-0.104298075600703,0.00905270180120077, ...
        33.1377092465126,8.47816736198223];
    want.preview = [0.0144927536231885,0.0144927536231884, ...
        0.0144927536231884,0.0468781563922983,0.289855072463768];
end
end
