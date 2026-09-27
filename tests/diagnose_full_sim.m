solverDir = 'D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers\Matlab84+';
modelDir = fileparts(fileparts(mfilename('fullpath')));
cd(modelDir);
addpath(modelDir, solverDir);
startup_pmpc();
clear global;
load_system('Solver_SF');
load_system('cq3_2019');
set_param('cq3_2019', 'StopTime', '6.25');

try
    capturedOutput = evalc("simOut = sim('cq3_2019');"); %#ok<NASGU>
    fprintf('CODEX_SIM_STOP_TIME=%.6f\n', simOut.tout(end));
catch ME
    fprintf('IDENTIFIER=%s\n', ME.identifier);
    fprintf('%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
    for k = 1:numel(ME.cause)
        fprintf('CAUSE_%d_IDENTIFIER=%s\n', k, ME.cause{k}.identifier);
        fprintf('CAUSE_%d:\n%s\n', k, ...
            getReport(ME.cause{k}, 'extended', 'hyperlinks', 'off'));
    end
end

close_system('cq3_2019', 0);
close_system('Solver_SF', 0);
