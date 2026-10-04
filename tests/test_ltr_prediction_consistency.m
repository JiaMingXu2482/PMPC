function tests = test_ltr_prediction_consistency
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'controller'));
end

function testCurrentLTRUsesMeasuredDamperMoment(testCase)
[V,M,S,H,Y,x,AI,Ut] = fixture();
[~,ltr] = func_LTRDiagnosis(V,M,S,H,Y,x,1,AI,Ut,1000,1);
% 2/(2000*10*2) * (1000*0.1 + 300) = 0.0200.
verifyEqual(testCase,ltr,0.0200,'AbsTol',1e-12);
end

function testForecastIncludesAffineTireForceOffset(testCase)
[V,M,S,H,Y,x,AI,Ut] = fixture();
% Tire affine forces sum to 5000 N; with zero Vy/yaw rate the predicted
% a_y is 2.5 m/s^2. The seventh state is the actual delayed moment (200
% Nm), not the 1000 Nm command.
ayOffset = 5000/V.m;
[~,~,ltr] = func_LTRDiagnosis(V,M,S,H,Y,x,1,AI,Ut,1000,1,ayOffset);
verifyEqual(testCase,ltr,0.12125,'AbsTol',1e-12);
end

function [V,M,S,H,Y,x,AI,Ut] = fixture()
V = struct('m',2000,'ms',1500,'g',10,'tf',2,'Kt',1000, ...
    'h_TL',0.8,'h_S2R',0.5,'CbarF',-10000,'CbarR',-8000, ...
    'lf',1,'lr',2);
M = struct('Nu',3,'Nc',1,'Np',1,'Nx',7,'Ny',7);
S = struct('x_dot',10,'delta_f',0);
H = struct('Roll',0.1,'Md',300,'Fy_l1',0,'Fy_r1',0, ...
    'Fy_l2',0,'Fy_r2',0,'Fz_l1',4750,'Fz_l2',4750, ...
    'Fz_r1',5250,'Fz_r2',5250);
Y = [0;0;0.1;0;0;0;200];
x = zeros(3,1);
AI = eye(3);
Ut = [0;0;1000];
end
