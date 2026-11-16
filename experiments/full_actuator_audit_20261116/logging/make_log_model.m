function make_log_model(caseName)
%MAKE_LOG_MODEL 复制生产 pmpc_mil.slx 为日志模型并加观测 tap（只分支，不改逻辑）。
%  Ia 按精确块名 Unit Delay / Unit Delay1..3 对应 L1/L2/R1/R2（原 SID 981/999/1008/1017）。
here = fileparts(mfilename('fullpath'));
MAIN = fileparts(fileparts(fileparts(here)));
old = cd(MAIN); cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
warning('off','Simulink:Engine:ShadowedModel');

dst = fullfile(here, sprintf('PmpcLog_%s.slx', caseName));
copyfile(fullfile(MAIN,'pmpc_mil.slx'), dst, 'f');
mdl = sprintf('PmpcLog_%s', caseName);
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(dst);

fir = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','DiscreteFir');
dp = get_param(fir{1},'Parent');
while ~strcmp(get_param(dp,'Parent'), mdl), dp = get_param(dp,'Parent'); end
fprintf('damper subsystem = %s\n', dp);

tags = {}; nTap = 0;
ph = get_param(dp,'PortHandles');

% 1) I_cmd (In1..In4) 与 u (In5)
for i=1:4
    lh = get_param(ph.Inport(i),'Line');
    assert(lh>0, 'In%d 无连线', i);
    nTap = nTap + add_log(mdl, sprintf('LogIcmd%d',i), lh);
    tags{end+1} = sprintf('LogIcmd%d',i);
end
lh = get_param(ph.Inport(5),'Line'); assert(lh>0,'In5 无连线');
nTap = nTap + add_log(mdl, 'LogU', lh); tags{end+1}='LogU';

% 2) Ia：精确块名映射
udNames = {'Unit Delay','Unit Delay1','Unit Delay2','Unit Delay3'};
iaOk = 0;
for c=1:4
    blk = [dp '/' udNames{c}];
    assert(getSimulinkBlockHandle(blk)>0, '找不到块 %s', blk);
    h = get_param(blk,'PortHandles');
    lh = get_param(h.Outport(1),'Line');
    assert(lh>0, '%s 输出无连线', udNames{c});
    % 核对去向里的 Goto tag
    dn = get_param(lh,'DstBlockHandle'); tagStr = '';
    for k=1:numel(dn)
        if strcmp(get_param(dn(k),'BlockType'),'Goto')
            tagStr = [tagStr ' ' get_param(dn(k),'GotoTag')]; %#ok<AGROW>
        end
    end
    fprintf('  Ia%d <- %-14s SID=%s  Goto:%s\n', c, udNames{c}, get_param(blk,'SID'), tagStr);
    nTap = nTap + add_log(dp, sprintf('LogIa%d',c), lh);
    tags{end+1} = sprintf('LogIa%d',c); %#ok<AGROW>
    iaOk = iaOk + 1;
end
assert(iaOk==4, 'Ia tap 只有 %d 路', iaOk);

% 3) Fa：子系统 4 个输出
for i=1:4
    lh = get_param(ph.Outport(i),'Line');
    assert(lh>0,'Out%d 无连线', i);
    nTap = nTap + add_log(mdl, sprintf('LogFa%d',i), lh);
    tags{end+1} = sprintf('LogFa%d',i); %#ok<AGROW>
end

% 4) 边界子系统：MATLAB Function 具有 8 入 / 4 出（SID 939），
%    四个输出依次为 Fhi, Flo, F_at0, F_at16（各 4 维）。
bf = '';
allMF = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','SubSystem');
for k=1:numel(allMF)
    if ~contains(get_param(allMF{k},'Name'),'MATLAB Function'), continue; end
    phk = get_param(allMF{k},'PortHandles');
    if numel(phk.Inport)==8 && numel(phk.Outport)==4, bf = allMF{k}; break; end
end
assert(~isempty(bf), '找不到 8 入 4 出的边界 MATLAB Function');
bph = get_param(bf,'PortHandles');
bnames = {'LogFhi','LogFlo','LogFat0','LogFat16'};
for j=1:4
    lh = get_param(bph.Outport(j),'Line');
    assert(lh>0, '边界输出 %d 无连线', j);
    nTap = nTap + add_log(get_param(bf,'Parent'), bnames{j}, lh);
    tags{end+1} = bnames{j};
end
fprintf('boundary taps: %s\n', strjoin(bnames,', '));
fprintf('taps = %d (期望 17)\n', nTap);
assert(nTap>=17, 'tap 数量不足: %d', nTap);
save_system(mdl, dst);
save(fullfile(here,sprintf('taps_%s.mat',caseName)),'tags','dp','mdl');
close_system(mdl,0);
fprintf('saved %s\n', dst);
end

function n = add_log(parent, varName, lineHandle)
%ADD_LOG 在已有信号线上挂一个 To Workspace（分支），保存时间。
mdl = strtok(parent,'/');
blk = [parent '/TW_' varName];
if getSimulinkBlockHandle(blk) > 0, n = 0; return; end
add_block('simulink/Sinks/To Workspace', blk, 'VariableName', varName, ...
    'SaveFormat','StructureWithTime','SampleTime','-1','Position',[900 20 980 50]);
ph = get_param(blk,'PortHandles');
add_line(parent, get_param(lineHandle,'SrcPortHandle'), ph.Inport(1), 'autorouting','on');
n = 1;
end
