function tests = test_ey_console_summary
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
testCase.TestData.root = root;
end

function testPrintsPeakAndRmsFromRealErd(testCase)
tag = fullfile(testCase.TestData.root, 'simulation_results', 'current', ...
    'erd_0927_base', 'MPC');
D = func_ReadERD(tag);

text = evalc('S = func_EyConsoleSummary(D);');
M = func_Metrics(D);

verifyEqual(testCase, S.ey_peak_m, M.ey_pk, 'AbsTol', 1e-12);
verifyEqual(testCase, S.ey_rms_m, M.ey_rms, 'AbsTol', 1e-12);
verifySubstring(testCase, text, 'e_y peak');
verifySubstring(testCase, text, 'e_y RMS');
verifySubstring(testCase, text, 'm');
end

function testCombinedRunPrintsSeparateDlcAndJTurnWithVelocityRms(testCase)
tag = fullfile(testCase.TestData.root, 'simulation_results', 'combined', 'Results', ...
    'Run_Combined90_R70_PMPC', 'LastRun');
D = func_ReadERD(tag);

output = evalc('S = func_EyConsoleSummary(D);');

verifyNumElements(testCase, S.segments, 2);
verifyEqual(testCase, fieldnames(S), {'segments'});
verifyEqual(testCase, fieldnames(S.segments), ...
    {'name';'ey_peak_m';'ey_rms_m';'vx_rms_kmh'});
verifyEqual(testCase, {S.segments.name}, {'DLC','J-turn'});
verifySubstring(testCase, output, 'DLC');
verifySubstring(testCase, output, 'J-turn');
verifySubstring(testCase, output, 'Vx RMS');
verifyNumElements(testCase, regexp(strtrim(output), '\r?\n', 'split'), 2);
% This fixture uses the generated X-Y road, whose straight-road station
% offset is 0.6415 m; a cone-based straight road has zero offset.
dlcEnd = 200.6415;
turnStart = 280.6415;
dlcVx = D.Vx(D.Station>=0 & D.Station<dlcEnd);
turnVx = D.Vx(D.Station>=turnStart);
verifyEqual(testCase, S.segments(1).vx_rms_kmh, ...
    sqrt(mean(dlcVx.^2)), 'AbsTol', 1e-10);
verifyEqual(testCase, S.segments(2).vx_rms_kmh, ...
    sqrt(mean(turnVx.^2)), 'AbsTol', 1e-10);
verifyGreaterThan(testCase, S.segments(1).ey_peak_m, 0);
verifyGreaterThan(testCase, S.segments(2).ey_peak_m, 0);
end

function testCombinedRunDoesNotReportJTurnWhenSimulationStopsAtDlc(testCase)
tag = fullfile(testCase.TestData.root, 'simulation_results', 'combined', 'Results', ...
    'Run_Combined90_R70_PMPC', 'LastRun');
D = func_ReadERD(tag);
ix = D.Station < 200;
fields = fieldnames(D);
for j = 1:numel(fields)
    v = D.(fields{j});
    if isnumeric(v) && isvector(v) && numel(v)==D.N
        D.(fields{j}) = v(ix);
    end
end
D.N = nnz(ix);

output = evalc('S = func_EyConsoleSummary(D);');

verifyNumElements(testCase, S.segments, 2);
verifyTrue(testCase, isnan(S.segments(2).ey_peak_m));
verifyTrue(testCase, isnan(S.segments(2).vx_rms_kmh));
verifySubstring(testCase, output, 'J-turn');
verifySubstring(testCase, output, 'not reached');
end

function testConeBasedStraightRoadUsesRoadStationForSegmentBoundaries(testCase)
tag = fullfile(testCase.TestData.root, 'simulation_results', 'combined', 'Results', ...
    'Run_Combined90_R70_PMPC', 'LastRun');
D = func_ReadERD(tag);
% Same vehicle response, but CarSim's straight-road centerline has no
% 0.6415 m DLC arc-length offset in its station coordinate.
D.Station = max(0,D.Station-0.6415);

evalc('S = func_EyConsoleSummary(D);');

dlcVx = D.Vx(D.Station>=0 & D.Station<200);
turnVx = D.Vx(D.Station>=280);
verifyEqual(testCase,S.segments(1).vx_rms_kmh, ...
    sqrt(mean(dlcVx.^2)),'AbsTol',1e-10);
verifyEqual(testCase,S.segments(2).vx_rms_kmh, ...
    sqrt(mean(turnVx.^2)),'AbsTol',1e-10);
end
