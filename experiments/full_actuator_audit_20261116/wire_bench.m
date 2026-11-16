function wire_bench()
%WIRE_BENCH 给 DamperBench 接线：In1=电流指令阶跃，In2..In5=常数（运动/温度等），
% 输出 4 角力到 To Workspace。运行一次，用来确认端口含义与整链响应。
here = fileparts(mfilename('fullpath')); addpath(here);
if bdIsLoaded('DamperBench'), close_system('DamperBench',0); end
load_system(fullfile(here,'DamperBench.slx'));
bdclose('DamperBench'); copyfile(fullfile(here,'DamperBench.slx'), fullfile(here,'DamperRun.slx'),'f');
load_system(fullfile(here,'DamperRun.slx'));
m='DamperRun';
ph = get_param([m '/DamperPlant'],'PortHandles');
nIn = numel(ph.Inport); nOut = numel(ph.Outport);

% 输入：In1 阶跃 0.4->1.2 @0.2s；其余常数
vals = [0 200 0 0 40];          % 猜测：cmpD/cmpRD/.../温度；下面按端口数截断
add_block('simulink/Sources/Step',[m '/Istep'],'Time','0.2','Before','0.4','After','1.2', ...
    'SampleTime','0','Position',[20 20 60 40]);
add_line(m,'Istep/1','DamperPlant/1','autorouting','on');
for i=2:nIn
    nm = sprintf('C%d',i);
    add_block('simulink/Sources/Constant',[m '/' nm],'Value',num2str(vals(min(i,numel(vals)))), ...
        'SampleTime','0','Position',[20 20+40*i 60 40+40*i]);
    add_line(m,[nm '/1'],['DamperPlant/' num2str(i)],'autorouting','on');
end
% 输出记录
pos = [400 20 460 60];
add_block('simulink/Sinks/To Workspace',[m '/Log'],'VariableName','Fa','SaveFormat','Array', ...
    'SampleTime','-1','Position',pos);
add_line(m,'DamperPlant/1','Log/1','autorouting','on');
set_param(m,'StopTime','0.6','Solver','FixedStepDiscrete','FixedStep','0.001');
out = sim(m,'ReturnWorkspaceOutputs','on');
Fa = out.get('Fa');
if isstruct(Fa), Fa = Fa.signals.values; end
Fa = squeeze(Fa);
fprintf('Fa size %s\n', mat2str(size(Fa)));
if size(Fa,1) < size(Fa,2), Fa = Fa.'; end
t = out.tout;
c0 = mean(Fa(t>0.05 & t<0.19, :),1); c1 = mean(Fa(t>0.45, :),1);
fprintf('每角 前/后 均值: %s\n', mat2str([c0; c1], 6));
fprintf('响应幅值: %s\n', mat2str(abs(c1-c0),6));
save(fullfile(here,'bench_run.mat'),'t','Fa','c0','c1');
end
