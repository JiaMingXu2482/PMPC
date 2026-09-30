function P = mil_init_variant(variant, bundleName, blockName, modelName)
%MIL_INIT_VARIANT  Shared preparation for one fixed controller/model pair.

if ~isscalar(variant) || ~ismember(variant, [1 2 3])
    error('mil_init_variant:InvalidVariant', 'variant must be 1 (MPC), 2 (ZENG), or 3 (PMPC).');
end

startup_pmpc();
evalin('base', 'clear PMPC_MODE PMPC_ZENGRHO PMPC_P MPC_P ZENG_P NLCSNN');
assignin('base', 'PMPC_CONTROLLER_VARIANT', variant);
if evalin('base', 'exist(''PMPC_VERBOSE'', ''var'') ~= 1')
    assignin('base', 'PMPC_VERBOSE', 1);
end
evalin('base', ['clear ' blockName]);

P = setup_pmpc();
assignin('base', bundleName, P);
assignin('base', 'NLCSNN', P.Pm.NLCSNN);

fprintf('\n===== %s ready =====\n', upper(modelName));
fprintf('  controller variant: %d | Nx: %d | tau_d: %.3f s\n', ...
    P.Pm.MPCParameters.ControllerVariant, P.Pm.MPCParameters.Nx, P.Pm.MPCParameters.tau_d);
fprintf('  open/run: %s\n\n', modelName);
end
