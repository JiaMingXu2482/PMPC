function run_log_case(caseName, speed, ctrl, opts)
%RUN_LOG_CASE 用日志模型跑工况，保存 1 kHz 全链路记录 + 100 Hz pmpc_log。
% 顺序严格照 run_dlc/run_slalom：先 load_system（触发 PreLoadFcn），
% 再 pmpc_init/assignin，避免回调把参数覆盖回默认 controller=1。
if nargin<4, opts=struct(); end
here = fileparts(mfilename('fullpath'));
MAIN = fileparts(fileparts(fileparts(here)));
old = cd(MAIN); cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
addpath(fullfile(MAIN,'pmpc_core'),'-begin');
addpath(fullfile(MAIN,'nlcsnn'),'-begin');

mdl = sprintf('PmpcLog_%s', caseName);
isSla = strcmpi(caseName,'sla80');
if ~isfield(opts,'stopTime'), opts.stopTime = ternary(isSla,8,15); end
if ~isfield(opts,'tau_d'),    opts.tau_d = 1e-4; end
if ~isfield(opts,'simfile')
    if isSla, opts.simfile = fullfile(MAIN,'simfile_slalom.sim');
    else,     opts.simfile = fullfile(MAIN,'simfile_dlc.sim'); end
end
man = ternary(isSla,2,1);

% ---------- 1) CarSim 展开 par：照 run_dlc / run_slalom ----------
dataset = cs_dataset_name(opts.simfile);
resdir  = fullfile('D:\Program Files\CarSim2019.0\CarSim2019.0_Data\Results', dataset);
parfile = fullfile(resdir,'Run_all.par');
assert(isfile(parfile),'缺 Run_all.par: %s', parfile);
copyfile(parfile, fullfile(here,sprintf('Run_all_before_%s.par',caseName)),'f');  % 改动前快照
lines = regexp(fileread(parfile),'\r?\n','split');
n=0;
for k=1:numel(lines)
    if strncmp(strtrim(lines{k}),'SV_VXS ',7), lines{k}=sprintf('SV_VXS %g',speed); n=n+1; end
end
assert(n>0,'par 里没有 SV_VXS');
if ~isSla     % DLC 才把 SV_XO 归零
    for k=1:numel(lines)
        if startsWith(strtrim(lines{k}),'SV_XO '), lines{k}='SV_XO 0'; end
    end
end
tstop=20; sstop=200;
for k=1:numel(lines)
    if startsWith(strtrim(lines{k}),'TSTOP '), lines{k}=sprintf('TSTOP %g',tstop);
    elseif startsWith(strtrim(lines{k}),'SSTOP '), lines{k}=sprintf('SSTOP %g',sstop); end
end
fid=fopen(parfile,'w'); fprintf(fid,'%s\n',lines{:}); fclose(fid);
copyfile(parfile, fullfile(here,sprintf('Run_all_used_%s.par',caseName)),'f');   % 实际使用

% ---------- 2) 先加载模型（让 PreLoadFcn 跑完）----------
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(here,[mdl '.slx']));
set_param([mdl '/CarSim'],'SIMFILE',opts.simfile);
set_param(mdl,'StopTime',sprintf('%g',opts.stopTime));

% ---------- 3) 再初始化并写入基础工作区 ----------
[P,R,s0,m0] = pmpc_init(man, ctrl);
P.M.tau_d = opts.tau_d; P.M.tau_d_mode = 1;
assignin('base','params',P);  assignin('base','Reftraj',R);
assignin('base','state0',s0); assignin('base','meas0',m0);
[mb,sb,pb] = pmpc_bus(m0,s0);
assignin('base','PMPC_MeasBus',mb); assignin('base','PMPC_StateBus',sb);
assignin('base','PMPC_PredictionBus',pb);
assignin('base','delta_d',0);                 % 原控制所需的驾驶员转角
assignin('base','V_target',speed);            % v_target 块的字面量引用

% v_target 常量块：按名字扫，assert 命中 1 个
cs = find_system(mdl,'SearchDepth',1,'BlockType','Constant');
hit = cs(contains(cs,'v_target'));
assert(numel(hit)==1, 'v_target 常量块命中 %d 个', numel(hit));
set_param(hit{1},'Value',num2str(speed));

% ---------- 4) 仿真前核对基础工作区与局部 P 一致 ----------
bc = evalin('base','params.M.controller'); bt = evalin('base','params.M.tau_d');
bn = evalin('base','params.M.Np');        bT = evalin('base','params.M.Ts');
fprintf('CHECK base: controller=%d tau_d=%g Np=%d Ts=%g | local: %d %g %d %g\n', ...
    bc,bt,bn,bT,P.M.controller,P.M.tau_d,P.M.Np,P.M.Ts);
assert(bc==ctrl, '基础工作区 controller=%d 与请求 %d 不一致', bc, ctrl);
assert(abs(bt-P.M.tau_d)<1e-12, 'tau_d 不一致');
assert(bn==P.M.Np && abs(bT-P.M.Ts)<1e-12, '网格参数不一致');

fprintf('RUN %s: ctrl=%d speed=%g man=%d stop=%g\n', caseName, ctrl, speed, man, opts.stopTime);
out = sim(mdl,'ReturnWorkspaceOutputs','on');

names = {'LogIcmd1','LogIcmd2','LogIcmd3','LogIcmd4','LogU', ...
         'LogIa1','LogIa2','LogIa3','LogIa4','LogFa1','LogFa2','LogFa3','LogFa4', ...
         'LogFhi','LogFlo','LogFat0','LogFat16'};
rec = struct();
for i=1:numel(names)
    v = out.get(names{i});
    rec.(names{i}) = struct('time',double(v.time(:)),'values',squeeze(double(v.signals.values)));
end
L=out.pmpc_log; D=squeeze(L.Data); if size(D,1)~=27, D=D.'; end
t=double(L.Time(:));
save(fullfile(here,sprintf('log_%s.mat',caseName)),'rec','D','t','P','opts','caseName','speed','ctrl');
fprintf('SAVED log_%s.mat  n1k=%d n100=%d\n', caseName, numel(rec.LogIa1.time), numel(t));
close_system(mdl,0);
end

function y = ternary(c,a,b)
if c, y=a; else, y=b; end
end
