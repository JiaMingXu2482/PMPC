function tests = test_carsim_interface
tests = functiontests(localfunctions);
end

function testGeneratedInterfaceHas54Exports(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
simText = fileread(fullfile(root, 'simfile.sim'));
verifyNotEmpty(testCase, regexp(simText, '(?m)^PORTS_EXP\s+1,54\s*$', 'once'));
end

function testActiveExportDatasetAppendsCmpD(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
simText = fileread(fullfile(root, 'simfile.sim'));
workDir = token(simText, '(?m)^SET_MACRO \$\(WORK_DIR\)\$\s+(.+?)\s*$');
runName = token(simText, '(?m)^SET_MACRO \$\(ROOT_FILE_NAME\)\$\s+(\S+)\s*$');
runPar = fullfile(workDir, 'Results', runName, 'Run_all.par');
assumeTrue(testCase, exist(runPar, 'file') == 2, ...
    ['Active CarSim Run_all.par is unavailable: ' runPar]);

runText = fileread(runPar);
exports = regexp(runText, '(?m)^EXPORT\s+(\S+)\s*$', 'tokens');
exports = cellfun(@(c)c{1}, exports, 'UniformOutput', false);
verifyEqual(testCase, exports(:), expectedExports());
verifyNotEmpty(testCase, regexp(runText, '(?m)^PORTS_EXP\s+1,54\s*$', 'once'));

relDataset = token(runText, ...
    '(?m)^ENTER_PARSFILE\s+(IO_Channels\\O_Channels\\Export_\S+\.par)\s*$');
datasetPath = fullfile(workDir, relDataset);
verifyEqual(testCase, exist(datasetPath, 'file'), 2);
datasetText = fileread(datasetPath);
datasetExports = regexp(datasetText, '(?m)^EXPORT\s+(\S+)\s*$', 'tokens');
datasetExports = cellfun(@(c)c{1}, datasetExports, 'UniformOutput', false);
verifyEqual(testCase, datasetExports(:), expectedExports());
end

function value = token(text, expression)
match = regexp(text, expression, 'tokens', 'once');
assert(~isempty(match), 'test_carsim_interface:MissingToken', ...
    'Required CarSim token was not found.');
value = strtrim(match{1});
end

function names = expectedExports()
names = { ...
    'CmpRD_L1';'CmpRD_L2';'CmpRD_R1';'CmpRD_R2'; ...
    'Alpha_L1';'Alpha_L2';'Alpha_R1';'Alpha_R2'; ...
    'Fx_L1';'Fx_L2';'Fx_R1';'Fx_R2'; ...
    'Fy_L1';'Fy_L2';'Fy_R1';'Fy_R2'; ...
    'Fz_L1';'Fz_L2';'Fz_R1';'Fz_R2'; ...
    'Fd_L1';'Fd_L2';'Fd_R1';'Fd_R2'; ...
    'AVy_L1';'AVy_L2';'AVy_R1';'AVy_R2'; ...
    'My_Dr_L1';'My_Dr_L2';'My_Dr_R1';'My_Dr_R2'; ...
    'AVx';'AVz';'Ax';'Ay';'Beta';'Roll';'Yaw';'Xo';'Yo'; ...
    'Vx';'Vy';'AAx';'AAz';'Steer_L1';'Steer_R1';'Steer_SW'; ...
    'VxTarget';'LTR';'CmpD_L1';'CmpD_L2';'CmpD_R1';'CmpD_R2'};
end
