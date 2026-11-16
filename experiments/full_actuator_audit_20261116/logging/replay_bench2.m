function replay_bench2(caseName)
%REPLAY_BENCH2 用记录的 1 ms I_cmd(4 路) 与 54 维 u 回放完整减振器链路，
% 逐点比较 Ia / Fa。对齐规则：Fa→Fa、Ia→Ia，不做任意移位。
here = fileparts(mfilename('fullpath'));
MAIN = fileparts(fileparts(fileparts(here)));
old = cd(MAIN); cleanupObj = onCleanup(@() cd(old)); %#ok<NASGU>
addpath(fullfile(MAIN,'nlcsnn'),'-begin');
addpath(fullfile(here),'-begin');

L = load(fullfile(here,sprintf('log_%s.mat',caseName)));
rec = L.rec;
tlog = rec.LogIcmd1.time;

% --- 初始条件与整车一致 ---
assignin('base','root',MAIN);
evalin('base', sprintf('run(''%s'')', fullfile(MAIN,'init_valve2.m')));
assignin('base','Iref_init',0.8);
assignin('base','D_init_avg',3*0.8/24);
assignin('base','D_init',(3*0.8+0.7)/(24+0.7));
assert(contains(which('fwd_mf_impl'), MAIN), '路径解析不在 MAIN');

% --- 构造回放模型（DamperBench 副本） ---
src = fullfile(fileparts(here),'DamperBench.slx');
dst = fullfile(here,'ReplayBench.slx');
copyfile(src,dst,'f');
if bdIsLoaded('ReplayBench'), close_system('ReplayBench',0); end
load_system(dst); m='ReplayBench';
dp = [m '/DamperPlant'];

% Ia tap
udNames = {'Unit Delay','Unit Delay1','Unit Delay2','Unit Delay3'};
for c=1:4
    h=get_param([dp '/' udNames{c}],'PortHandles');
    add_block('simulink/Sinks/To Workspace',[dp sprintf('/TWIa%d',c)], ...
        'VariableName',sprintf('rIa%d',c),'SaveFormat','StructureWithTime','SampleTime','-1');
    ph2=get_param([dp sprintf('/TWIa%d',c)],'PortHandles');
    add_line(dp,h.Outport(1),ph2.Inport(1),'autorouting','on');
end
% 输入 From Workspace（4 路电流 + 54 维 u）
for c=1:4
    add_block('simulink/Sources/From Workspace',[m sprintf('/FI%d',c)], ...
        'VariableName',sprintf('rIcmd%d',c),'SampleTime','0');
    add_line(m,sprintf('FI%d/1',c),sprintf('DamperPlant/%d',c),'autorouting','on');
end
add_block('simulink/Sources/From Workspace',[m '/FU'],'VariableName','rU','SampleTime','0');
add_line(m,'FU/1','DamperPlant/5','autorouting','on');
% Fa tap
for c=1:4
    add_block('simulink/Sinks/To Workspace',[m sprintf('/TWFa%d',c)], ...
        'VariableName',sprintf('rFa%d',c),'SaveFormat','StructureWithTime','SampleTime','-1');
    ph2=get_param([m sprintf('/TWFa%d',c)],'PortHandles');
    add_line(m,sprintf('DamperPlant/%d',c),sprintf('TWFa%d/1',c),'autorouting','on');
end
% 工作区数据
for c=1:4
    assignin('base',sprintf('rIcmd%d',c), timeseries(rec.(sprintf('LogIcmd%d',c)).values, tlog));
end
assignin('base','rU', timeseries(rec.LogU.values, tlog));
set_param(m,'StopTime',sprintf('%g',tlog(end)),'Solver','FixedStepDiscrete','FixedStep','0.001');
out = sim(m,'ReturnWorkspaceOutputs','on');

res = struct();
for c=1:4
    ia = out.get(sprintf('rIa%d',c)); fa = out.get(sprintf('rFa%d',c));
    res.ia{c} = struct('time',double(ia.time(:)),'values',squeeze(double(ia.signals.values)));
    res.fa{c} = struct('time',double(fa.time(:)),'values',squeeze(double(fa.signals.values)));
end
save(fullfile(here,sprintf('replay_%s.mat',caseName)),'res');
close_system(m,0);
fprintf('REPLAY_SAVED %s\n', caseName);
end
