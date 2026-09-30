function verify_run_ready()
% Clean-session check: does setup_pmpc alone leave base ready for sim?
% Mirrors what run_current_carsim does before sim() - no manual NLCSNN.
startup_pmpc();
ok = func_CarSimLib();
fprintf('func_CarSimLib -> %d  (Solver_SF = %s)\n', ok, which('Solver_SF'));
setup_pmpc();

w = evalin('base','whos'); nm = {w.name};
fprintf('base has NLCSNN : %d\n', any(strcmp(nm,'NLCSNN')));
fprintf('base has PMPC_P : %d\n', any(strcmp(nm,'PMPC_P')));
N = evalin('base','NLCSNN');
fprintf('NLCSNN.dt_plant          = %g\n', N.dt_plant);
fprintf('PMPC_P.Pm.NLCSNN.dt_plant= %g\n', evalin('base','PMPC_P.Pm.NLCSNN.dt_plant'));
fprintf('NLCSNN fields: %s\n', strjoin(fieldnames(N)', ', '));

load_system('pmpc_mil');
try
    set_param('pmpc_mil','SimulationCommand','update');
    fprintf('\n>>> COMPILE OK - 参数已解析, run_current_carsim 可以往下走\n');
catch ME
    fprintf('\n>>> COMPILE FAILED: %s\n%s\n', ME.identifier, ME.getMessage());
    c = ME.cause;
    while ~isempty(c)
        fprintf('    cause: %s | %s\n', c{1}.identifier, c{1}.message);
        c = c{1}.cause;
    end
end
close_system('pmpc_mil', 0);   % 0 = 不保存
end
