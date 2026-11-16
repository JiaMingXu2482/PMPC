function wire_bench2()
%WIRE_BENCH2 按主审给定的真实端口语义接线并冒烟：
%  In1..In4 = I_cmd_L1..R2 [A]（标量，0..1.6）
%  In5      = u，CarSim 54 维：u(1:4)=cmpRD [mm/s]（L1,L2,R1,R2），u(51:54)=cmpD [mm]
%  温度与时钟在子系统内部（40 °C / Clock），不要外部喂。
here = fileparts(mfilename('fullpath')); addpath(here,'-begin');
MAIN = fileparts(fileparts(here));            % ...\Controller\MAIN
addpath(fullfile(MAIN,'nlcsnn'),'-begin');   % 先保证解析到 MAIN 的 nlcsnn
addpath(fullfile(MAIN,'pmpc_core'),'-begin');
old = cd(MAIN); cleanup = onCleanup(@() cd(old)); %#ok<NASGU>

% 模型可见工作区：真实的初始化（含 InitFcn 的覆盖）
assignin('base','root',MAIN);
evalin('base', sprintf('run(''%s'')', fullfile(MAIN,'init_valve2.m')));
assignin('base','Iref_init',0.8);
assignin('base','D_init_avg',3*0.8/24);
assignin('base','D_init',(3*0.8+0.7)/(24+0.7));
fprintf('which fwd_mf_impl      = %s\n', which('fwd_mf_impl'));
fprintf('which fwd_weights_m    = %s\n', which('fwd_weights_m'));
fprintf('D_init = %.6f  Kp=%s Ki=%s\n', evalin('base','D_init'), ...
    evalin('base','mat2str(Kp)'), evalin('base','mat2str(Ki)'));
assert(contains(which('fwd_mf_impl'), MAIN), '路径解析到了非 MAIN 的文件');

src='DamperBench.slx';
if bdIsLoaded('DamperRun'), close_system('DamperRun',0); end
copyfile(fullfile(here,src), fullfile(here,'DamperRun.slx'),'f');
load_system(fullfile(here,'DamperRun.slx'));
m='DamperRun';

for i=1:4
    nm=sprintf('Icmd%d',i);
    add_block('simulink/Sources/Step',[m '/' nm],'Time','0.2','Before','0.4','After','1.2', ...
        'SampleTime','0','Position',[20 40*i 60 40*i+25]);
    add_line(m,[nm '/1'],sprintf('DamperPlant/%d',i),'autorouting','on');
end
u = zeros(54,1); u(1:4) = 200; u(51:54) = 0;
add_block('simulink/Sources/Constant',[m '/U'],'Value','u54','SampleTime','0', ...
    'Position',[20 240 60 270]);
assignin('base','u54',u);
add_line(m,'U/1','DamperPlant/5','autorouting','on');

add_block('simulink/Sinks/To Workspace',[m '/LogF'],'VariableName','Fa1','SaveFormat','Array', ...
    'SampleTime','-1','Position',[420 60 480 90]);
add_line(m,'DamperPlant/1','LogF/1','autorouting','on');

set_param(m,'StopTime','0.6','Solver','FixedStepDiscrete','FixedStep','0.001', ...
    'SaveTime','on','SaveOutput','on','ReturnWorkspaceOutputs','on');
try
    out = sim(m,'ReturnWorkspaceOutputs','on');
    Fa = out.get('Fa1'); if isstruct(Fa), Fa = Fa.signals.values; end
    Fa = double(squeeze(Fa)); t = out.tout;
    c0 = mean(Fa(t>0.05 & t<0.19)); c1 = mean(Fa(t>0.45));
    fprintf('SMOKE ok: Fa 前 %.1f -> 后 %.1f N  (Δ=%.1f)\n', c0, c1, c1-c0);
    save(fullfile(here,'smoke.mat'),'t','Fa','c0','c1');
catch ME
    f = fopen(fullfile(here,'error_bench.txt'),'w','n','UTF-8');
    fprintf(f,'%s\n', getReport(ME,'extended','hyperlinks','off'));
    if isprop(ME,'cause') && ~isempty(ME.cause)
        for k=1:numel(ME.cause), fprintf(f,'CAUSE %d: %s\n',k,getReport(ME.cause{k},'extended','hyperlinks','off')); end
    end
    fclose(f);
    fprintf('SMOKE failed -> error_bench.txt\n');
end
close_system(m,0);
end
