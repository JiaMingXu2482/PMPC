function outFile = func_SaveControllerLogs(simOut, resdir, model)
%FUNC_SAVECONTROLLERLOGS  Persist controller signals beside CarSim LastRun.

logsout = simOut.logsout;
assert(isa(logsout, 'Simulink.SimulationData.Dataset') && ...
    logsout.numElements >= 2, 'func_SaveControllerLogs:MissingSignals', ...
    '%s did not return the logged controller outputs.', model);
if strcmp(model, 'pmpc_mil') || strcmp(model, 'pmpc_nodelay_mil')
    assert(logsout.numElements >= 3, 'func_SaveControllerLogs:MissingPriority', ...
        '%s did not return the 12-channel priority diagnostics.', model);
end
outFile = fullfile(resdir, [model '_logs.mat']);
save(outFile, 'logsout', '-v7.3');
end
