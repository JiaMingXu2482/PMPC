function build_bench3()
%BUILD_BENCH3 用 add_block 把减振器子系统复制进新模型，并列出其一级块以识别端口。
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
old = cd(root); cleanupObj = onCleanup(@() cd(old)); %#ok<NASGU>
warning('off','all');

if bdIsLoaded('pm_work'), close_system('pm_work',0); end
copyfile(fullfile(root,'pmpc_mil.slx'), fullfile(here,'pm_work.slx'),'f');
load_system(fullfile(here,'pm_work.slx'));
m='pm_work';
fir = find_system(m,'LookUnderMasks','all','FollowLinks','on','BlockType','DiscreteFir');
cur = get_param(fir{1},'Parent');
while ~strcmp(get_param(cur,'Parent'), m), cur = get_param(cur,'Parent'); end
parent = get_param(cur,'Parent');
set_param(cur,'Name','DamperPlant');
cur = [parent '/DamperPlant'];

if bdIsLoaded('DamperBench'), close_system('DamperBench',0); end
new_system('DamperBench'); open_system('DamperBench');
add_block(cur,'DamperBench/DamperPlant');
save_system('DamperBench', fullfile(here,'DamperBench.slx'));

% 列出子系统一级块（含端口名）
fid=fopen(fullfile(here,'damper_blocks.txt'),'w');
b = find_system('DamperBench/DamperPlant','SearchDepth',1,'Type','Block');
for i=1:numel(b)
    bt=get_param(b{i},'BlockType');
    nm=get_param(b{i},'Name');
    extra='';
    if strcmp(bt,'Inport')||strcmp(bt,'Outport'), extra=[' -> ' get_param(b{i},'Port')]; end
    if strcmp(bt,'DiscreteFir'), extra=[' c=' mat2str(get_param(b{i},'Coefficients'))]; end
    if strcmp(bt,'DiscretePIDController'), extra=sprintf(' P=%s I=%s', get_param(b{i},'P'), get_param(b{i},'I')); end
    fprintf(fid,'[%s] %s%s\n', bt, nm, extra);
end
fclose(fid);
type(fullfile(here,'damper_blocks.txt'));
end
