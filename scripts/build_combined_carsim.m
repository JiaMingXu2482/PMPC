function S = build_combined_carsim(model,R)
%BUILD_COMBINED_CARSIM Create an isolated 90 km/h DLC + J-turn CarSim run.
% The source CarSim datasets are read only. Generated input and outputs live
% under simulation_results/combined, so the selected CarSim Run is untouched.
if nargin < 1 || isempty(model), model = 'pmpc_mil'; end
if nargin < 2 || isempty(R), R = 70; end
assert(any(strcmp(model,{'mpc_mil','zeng_mil','pmpc_mil'})), ...
    'build_combined_carsim:InvalidModel','Unknown model %s.',model);
tag = upper(erase(model,'_mil'));
root = startup_pmpc();
C = func_CombinedCourse(R);
W = func_WayPoints(6,C.turn_radius,false);
stopS = floor(W(end,7)-20);
roadEndS = ceil(W(end,7)+20);
name = sprintf('COMB90_DLC05_JT_R%g_mu0.5to0.85_%s',C.turn_radius,tag);
runId = sprintf('Run_Combined90_R%g_%s',R,tag);
outRoot = fullfile(root,'simulation_results','combined');
runDir = fullfile(outRoot,'Results',runId);
if ~exist(runDir,'dir'), mkdir(runDir); end

sfTemplate = fileread(fullfile(root,'simfile.sim'));
dataDir = local_macro(sfTemplate,'WORK_DIR');
srcRun = local_find_run(dataDir,'DLC80_mu0.5_PMPC');
[~,srcId] = fileparts(srcRun);
srcPar = fullfile(dataDir,'Results',srcId,'Run_all.par');
assert(exist(srcPar,'file')==2,'build_combined_carsim:NotSent', ...
    'Send DLC80_mu0.5_PMPC to Simulink once before building the combined run.');
par = fileread(srcPar);
assert(contains(par,'MU_ROAD_CONSTANT 0.5'), ...
    'build_combined_carsim:UnexpectedRoad','DLC source is not mu=0.5.');
par = strrep(par,'DLC80_mu0.5_PMPC',name);
modelLine = regexp(par,'(?m)^SIMULINK_MODEL_FILE[ \t]+[^\r\n]+', ...
    'match','once');
assert(~isempty(modelLine),'build_combined_carsim:NoModel', ...
    'The source CarSim run does not declare a Simulink model.');
par = strrep(par,modelLine, ...
    ['SIMULINK_MODEL_FILE ' fullfile(root,[model '.slx'])]);
par = regexprep(par,'(?m)^SV_VXS[ \t]+[-+]?[0-9.]+[ \t\r]*$', ...
    'SV_VXS 90');
par = regexprep(par,'(?m)^TSTOP[ \t]+[-+]?[0-9.]+[ \t\r]*$', ...
    'TSTOP 50');
par = regexprep(par,'(?m)^SSTOP[ \t]+[-+]?[0-9.]+[ \t\r]*$', ...
    sprintf('SSTOP %d',stopS));
par = regexprep(par,'(?m)^SSTOP\(([0-9]+)\)[ \t]+[-+]?[0-9.]+[ \t\r]*$', ...
    sprintf('SSTOP($1) %d',roadEndS));
assert(isempty(regexp(par,'(?m)^SV_VXS[ \t]+80[ \t\r]*$','once')) ...
    && contains(par,'TSTOP 50') && contains(par,sprintf('SSTOP %d',stopS)), ...
    'build_combined_carsim:StaleSpeedOrStop', ...
    'Generated CarSim run still has old speed or stop settings.');

pathText = local_path_section(W);
pathCsv = fullfile(outRoot,sprintf('combined_path_r%g_xy_station.csv',R));
indices = unique([1:4:size(W,1),size(W,1)]);
writematrix(W(indices,[2 3 7]),pathCsv);
par = local_replace_section(par, ...
    'Roads\BuilderSegment\','path',pathText);
straight = W(:,2) <= C.turn_x;
switchS = interp1(W(straight,2),W(straight,7),C.mu_switch_x,'linear');
frictionText = local_friction_section(C,switchS,roadEndS);
par = local_replace_section(par,'Roads\Friction\','friction',frictionText);
parFile = fullfile(runDir,'Run_all.par');
local_write(parFile,par);

prefix = fullfile(runDir,'LastRun');
sf = local_replace_macro(sfTemplate,'ROOT_FILE_NAME',runId);
sf = local_replace_macro(sf,'WORK_DIR',[outRoot filesep]);
sf = local_replace_macro(sf,'OUTPUT_PATH',fullfile(outRoot,'Results'));
sf = local_replace_macro(sf,'OUTPUT_FILE_PREFIX',prefix);
simfile = fullfile(outRoot,sprintf('run_combined_r%g_%s.sim',R,lower(tag)));
local_write(simfile,sf);

S = struct('name',name,'simfile',simfile,'parfile',parFile, ...
    'pathCsv',pathCsv, ...
    'outputPrefix',prefix,'waypoints',W,'mu_switch_station',switchS, ...
    'mu_switch_x',C.mu_switch_x,'turn_x',C.turn_x, ...
    'model',model,'reportFile',fullfile(outRoot, ...
        sprintf('report_r%g_%s.mat',R,lower(tag))), ...
    'course',C);
fprintf('Combined run prepared: mu %.2f -> %.2f at X=%.1f m / station=%.2f m; J-turn begins X=%.1f m.\n', ...
    C.mu_dlc,C.mu_jturn,C.mu_switch_x,switchS,C.turn_x);
end

function section = local_path_section(W)
section = sprintf(['ENTER_PARSFILE Roads\\Center_XY\\PathXY_Combined.par\n' ...
    '#FullDataName Path: X-Y Coordinates`Combined DLC J-turn`PMPC\n' ...
    'SET_IPATH_FOR_ID 0\nDEFINE_XY_TABLES 1\n' ...
    'ITAB_XY = NTAB_XY\nXY_TABLE_ID = ITAB_XY\n' ...
    'SET_DESCRIPTION PATH_ID Combined DLC J-turn\n' ...
    'SET_DESCRIPTION XY_TABLE_ID Combined DLC J-turn\n' ...
    'IPATHSEG = 1\nPATH_ID_DM = PATH_ID\n' ...
    'SEGMENT_TYPE = 1\nXY_SEGMENT_ID = XY_TABLE_ID\n' ...
    'OPT_PATH_LOOP 0\nSPATH_START 0\nSEGMENT_XY_TABLE\n']);
indices = unique([1:4:size(W,1),size(W,1)]);
for j = indices
    section = [section sprintf('%.6f, %.6f, %.6f\n', ...
        W(j,2),W(j,3),W(j,7))]; %#ok<AGROW>
end
section = [section sprintf('ENDTABLE\nEXIT_PARSFILE Roads\\Center_XY\\PathXY_Combined.par')];
end

function section = local_friction_section(C,switchS,roadEndS)
section = sprintf(['ENTER_PARSFILE Roads\\Friction\\RdMu_Combined.par\n' ...
    '#FullDataName Road: Friction Map, S-L Grid`Combined 0.5 to 0.85`PMPC\n' ...
    'MU_ROAD_CARPET 2D_STEP\n' ...
    '0, -100, 0, 100\n' ...
    '-100, %.6f, %.6f, %.6f\n' ...
    '%.6f, %.6f, %.6f, %.6f\n' ...
    '%.6f, %.6f, %.6f, %.6f\n' ...
    'ENDTABLE\nEXIT_PARSFILE Roads\\Friction\\RdMu_Combined.par'], ...
    C.mu_dlc,C.mu_dlc,C.mu_dlc, ...
    switchS,C.mu_jturn,C.mu_jturn,C.mu_jturn, ...
    roadEndS,C.mu_jturn,C.mu_jturn,C.mu_jturn);
end

function out = local_replace_section(in,kind,label,replacement)
pattern = ['(?s)ENTER_PARSFILE ' regexptranslate('escape',kind) ...
    '[^\r\n]+.*?EXIT_PARSFILE ' regexptranslate('escape',kind) '[^\r\n]+'];
old = regexp(in,pattern,'match','once');
assert(~isempty(old),'build_combined_carsim:MissingSection', ...
    'Cannot find %s section in expanded CarSim input.',label);
out = strrep(in,old,replacement);
end

function value = local_macro(text,key)
v = regexp(text,['(?m)^SET_MACRO \$\(' key '\)\$[ \t]+([^\r\n]+)'], ...
    'tokens','once');
assert(~isempty(v),'build_combined_carsim:MissingMacro','%s',key);
value = strtrim(v{1});
end

function text = local_replace_macro(text,key,value)
old = regexp(text,['(?m)^SET_MACRO \$\(' key '\)\$[^\r\n]*'], ...
    'match','once');
assert(~isempty(old),'build_combined_carsim:MissingMacro','%s',key);
text = strrep(text,old,['SET_MACRO $(' key ')$ ' value]);
end

function path = local_find_run(dataDir,name)
files = dir(fullfile(dataDir,'Runs','Run_*.par'));
path = '';
for j = 1:numel(files)
    candidate = fullfile(files(j).folder,files(j).name);
    f = fopen(candidate,'r');
    if f<0, continue; end
    head = fread(f,500,'*char').'; fclose(f);
    token = regexp(head,'FullDataName CarSim Run Control`([^`]+)`', ...
        'tokens','once');
    if ~isempty(token) && strcmp(token{1},name)
        path = candidate; return;
    end
end
assert(~isempty(path),'build_combined_carsim:MissingDLC', ...
    'Could not find source CarSim Run %s.',name);
end

function local_write(path,value)
f = fopen(path,'w');
assert(f>=0,'build_combined_carsim:WriteFailed','%s',path);
fwrite(f,value,'char'); fclose(f);
end
