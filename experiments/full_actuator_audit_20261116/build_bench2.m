function build_bench2()
%BUILD_BENCH2 复制模型 → 重命名非 ASCII 子系统 → 提取为独立台架模型。
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
old = cd(root); cleanupObj = onCleanup(@() cd(old)); %#ok<NASGU>
warning('off','all');

if bdIsLoaded('pmpc_mil'), close_system('pmpc_mil',0); end
copyfile(fullfile(root,'pmpc_mil.slx'), fullfile(here,'pm_work.slx'));
load_system(fullfile(here,'pm_work.slx'));
m = 'pm_work';

fir = find_system(m,'LookUnderMasks','all','FollowLinks','on','BlockType','DiscreteFir');
assert(~isempty(fir),'no FIR');
cur = get_param(fir{1},'Parent');
while ~strcmp(get_param(cur,'Parent'), m), cur = get_param(cur,'Parent'); end
parent = get_param(cur,'Parent');
set_param(cur,'Name','DamperPlant');
cur = [parent '/DamperPlant'];
fprintf('renamed -> %s\n', cur);

ph = get_param(cur,'PortHandles');
fprintf('inports=%d outports=%d\n', numel(ph.Inport), numel(ph.Outport));
for i=1:numel(ph.Inport)
    fprintf('  in %d: %s\n', i, get_param(ph.Inport(i),'Name'));
end
for i=1:numel(ph.Outport)
    fprintf('  out %d: %s\n', i, get_param(ph.Outport(i),'Name'));
end

if bdIsLoaded('DamperBench'), close_system('DamperBench',0); end
Simulink.BlockDiagram.copyContentsToNewModel(cur,'DamperBench');
save_system('DamperBench', fullfile(here,'DamperBench.slx'));
fprintf('saved DamperBench.slx\n');
save(fullfile(here,'bench_meta.mat'),'cur','ph');
end
