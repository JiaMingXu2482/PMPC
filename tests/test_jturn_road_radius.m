function tests = test_jturn_road_radius
tests = functiontests(localfunctions);
end

function testActualCarSimRadiusOverridesDatasetLabel(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
par = writeRunAll(testCase, 80);

mv = func_ManeuverFromRun('JT80_R69_mu0.85_PMPC', par);

verifyEqual(testCase, mv.R, 80);
verifyEqual(testCase, mv.type, 5);
end

function testCombinedCourseUsesCarSimRadiusDespiteOldDatasetLabel(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
par = writeRunAll(testCase, 80, 280);

mv = func_ManeuverFromRun('COMB90_DLC05_JT_R70_mu0.5to0.85_PMPC', par);

verifyEqual(testCase, mv.type, 6);
verifyEqual(testCase, mv.R, 80);
verifyEqual(testCase, mv.wp, 'WayPoints_Type6_R80.mat');
W = func_WayPoints(mv.type, mv.R, false);
verifyEqual(testCase, max(W(:,2)), 360, 'AbsTol', 1e-8);
end

function testCombinedRoadUsesLongerStraightFromCarSim(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
par = writeRunAll(testCase, 80, 330);

mv = func_ManeuverFromRun('COMB90_DLC05_JT_R70_mu0.5to0.85_PMPC', par);

assertTrue(testCase,isfield(mv,'turn_x'));
verifyEqual(testCase,mv.turn_x,330);
W = func_WayPoints(mv.type,mv.R,false,mv.turn_x);
verifyEqual(testCase,max(W(:,2)),410,'AbsTol',1e-8);
end

function testMetricsUseActualRoadInsteadOfDatasetLabel(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
par = writeRunAll(testCase, 80);
D = struct('Dataset','JT80_R69_mu0.85_PMPC','RunAllPar',par, ...
    'N',2,'t',[0;0.05],'Xo',[140;140],'Yo',[80;90], ...
    'Xcg_TM',[140;140],'Ycg_TM',[80;90],'Yaw',[90;90], ...
    'AVz',[0;0],'Vx',[80;80],'LTR',[0;0],'Roll',[0;0], ...
    'Ay',[0;0],'Steer_SW',[0;0],'Throttle',[0;0]);
for name = {'My_Bk_L1','My_Bk_R1','My_Bk_L2','My_Bk_R2', ...
        'Fd_L1','Fd_R1','Fd_L2','Fd_R2'}
    D.(name{1}) = zeros(2,1);
end

M = func_Metrics(D);

verifyLessThan(testCase, M.ey_pk, 0.01);
verifyLessThan(testCase, M.ey_rms, 0.01);
end

function testCombinedMetricsUseActualRoadInsteadOfDatasetLabel(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
D = struct('Dataset','COMB90_DLC05_JT_R70_mu0.5to0.85_PMPC', ...
    'N',2,'t',[0;0.05],'Xo',[360;360], ...
    'Yo',[80;90],'Xcg_TM',[360;360],'Ycg_TM',[80;90], ...
    'Yaw',[90;90],'AVz',[0;0],'Vx',[80;80], ...
    'LTR',[0;0],'Roll',[0;0],'Ay',[0;0], ...
    'Steer_SW',[0;0],'Throttle',[0;0]);
for name = {'My_Bk_L1','My_Bk_R1','My_Bk_L2','My_Bk_R2', ...
        'Fd_L1','Fd_R1','Fd_L2','Fd_R2'}
    D.(name{1}) = zeros(2,1);
end

for turnX = [280 330]
    D.RunAllPar = writeRunAll(testCase,80,turnX);
    D.Xo(:) = turnX+80;
    D.Xcg_TM(:) = turnX+80;
    M = func_Metrics(D);
    verifyLessThan(testCase, M.ey_pk, 0.01);
    verifyLessThan(testCase, M.ey_rms, 0.01);
end
end

function par = writeRunAll(testCase, radius, straightLength)
if nargin < 3, straightLength = 60; end
folder = tempname;
mkdir(folder);
testCase.addTeardown(@() rmdir(folder,'s'));
par = fullfile(folder,'Run_all.par');
fid = fopen(par,'w');
assert(fid > 0);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'SEGMENT_LENGTH %.6f\nSEGMENT_RADIUS %.6f\nSEGMENT_LENGTH 100\n', ...
    straightLength,radius);
end
