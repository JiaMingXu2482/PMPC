function model = func_ControllerModelForTag(tag)
%FUNC_CONTROLLERMODELFORTAG Map a controller identifier to its Simulink model.

if isstring(tag) && isscalar(tag)
    tag = char(tag);
end
if ~ischar(tag)
    error('func_ControllerModelForTag:InvalidTag', ...
        'Controller tag must be a character vector or scalar string.');
end

switch upper(strtrim(tag))
    case 'MPC'
        model = 'mpc_mil';
    case 'ZENG'
        model = 'zeng_mil';
    case 'PMPC'
        model = 'pmpc_mil';
    otherwise
        error('func_ControllerModelForTag:InvalidTag', ...
            'Unsupported controller tag "%s". Use MPC, ZENG, or PMPC.', tag);
end
end
