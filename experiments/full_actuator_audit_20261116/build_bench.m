function build_bench()
%BUILD_BENCH 用 MATLAB API 从 pmpc_mil.slx 复制减振器正模型子系统，构造可重放台架。
% 不修改生产 SLX；新模型与副本都在本任务目录。
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
old = cd(root);                     % 保证解析到 MAIN 的 pmpc_mil.slx
cleanupObj = onCleanup(@() cd(old));

m = 'pmpc_mil';
warning('off','Simulink:Engine:ShadowedModel');
if bdIsLoaded(m), close_system(m,0); end
load_system(fullfile(root,'pmpc_mil.slx'));

% 找到包含 DiscreteFir 的子系统（减振器模型）
blks = find_system(m,'LookUnderMasks','all','FollowLinks','on','BlockType','DiscreteFir');
assert(~isempty(blks), '找不到 DiscreteFir，模型结构已变');
sub = get_param(get_param(blks{1},'Parent'),'Name');
while ~strcmp(get_param(get_param(sub,'Parent'),'BlockType'),'SubSystem')==0
    break
end
% 向上找到顶层子系统
cur = get_param(blks{1},'Parent');
while ~strcmp(get_param(cur,'Parent'), m)
    cur = get_param(cur,'Parent');
end
damperName = strrep(cur,[m '/'],'');
fprintf('damper subsystem = %s\n', damperName);

% 端口
ph = get_param(cur,'PortHandles');
nIn = numel(ph.Inport); nOut = numel(ph.Outport);
fprintf('ports: in=%d out=%d\n', nIn, nOut);
for i=1:nIn
    pn = get_param(ph.Inport(i),'Name');
    fprintf('  in %d: %s\n', i, pn);
end

% 复制内容到新模型
newName = 'DamperBench';
if bdIsLoaded(newName), close_system(newName,0); end
newName = Simulink.BlockDiagram.copyContentsToNewModel(cur, newName);
save_system(newName, fullfile(here,[newName '.slx']));
fprintf('saved %s\n', fullfile(here,[newName '.slx']));
save(fullfile(here,'bench_meta.mat'),'damperName','nIn','nOut');
end
