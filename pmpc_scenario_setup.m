function pmpc_scenario_setup(path_type)
%PMPC_SCENARIO_SETUP 按工况配置模型（目标车速、StopTime、InitFcn）。
%   pmpc_scenario_setup(1)  % DLC，60 km/h，6.5 s
%   pmpc_scenario_setup(2)  % Slalom，70 km/h，10 s
%   pmpc_scenario_setup(3)  % J-turn，70 km/h，10 s
%
% 会保存模型并重新打开，使 MATLAB Function 块重新编译。

mdl = 'pmpc_mil';
root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end

switch path_type
    case 1
        target = 60; stopTime = 6.5; name = 'DLC';
    case 2
        target = 70; stopTime = 10; name = 'Slalom';
    case 3
        target = 70; stopTime = 10; name = 'J-turn';
    otherwise
        error('path_type 只能为 1(DLC)、2(Slalom)、3(J-turn)');
end

if ~bdIsLoaded(mdl)
    load_system(fullfile(root,[mdl '.slx']));
end

pre = sprintf(['root = ''%s'';\n\n' ...
    'addpath(fullfile(root,''pmpc_core''),''-begin'');\n' ...
    'addpath(fullfile(root,''nlcsnn''),''-begin'');\n\n' ...
    '[params,Reftraj,state0,meas0] = pmpc_init(%d);\n' ...
    '[PMPC_MeasBus,PMPC_StateBus,PMPC_PredictionBus] = ...\n' ...
    '    pmpc_bus(meas0,state0);'], root, path_type);
init = sprintf(['root = ''%s'';\n\n' ...
    'addpath(fullfile(root,''pmpc_core''),''-begin'');\n' ...
    'addpath(fullfile(root,''nlcsnn''),''-begin'');'], root);

set_param(mdl,'PreLoadFcn',pre);
set_param(mdl,'InitFcn',init);

allb = find_system(mdl,'FindAll','on','Type','block');
for i = 1:numel(allb)
    try
        if strcmp(get_param(allb(i),'BlockType'),'Constant') && ...
                contains(get_param(allb(i),'Name'),'v_target')
            set_param(allb(i),'Value',sprintf('%g',target));
        end
    catch
    end
end
set_param(mdl,'StopTime',sprintf('%g',stopTime));
save_system(mdl);

bdclose(mdl);
clear functions;
clear pmpc_init pmpc_bus pmpc_step pmpc_preview prepare_step ...
    build_prediction force_bounds_four inv_current_impl_10ms ...
    force_bounds_impl_10ms fwd_mf_impl curr_dyn_impl lon_arbitrate ...
    func_ZengRho;
open_system(mdl);

fprintf('%s 配置完成：v_target=%g km/h, StopTime=%g s, InitFcn=pmpc_init(%d)\n', ...
    name, target, stopTime, path_type);
end
