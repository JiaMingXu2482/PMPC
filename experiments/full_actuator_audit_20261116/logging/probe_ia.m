function probe_ia()
here = fileparts(mfilename('fullpath'));
if bdIsLoaded('PmpcLog_dlc75'), close_system('PmpcLog_dlc75',0); end
load_system(fullfile(here,'PmpcLog_dlc75.slx'));
m='PmpcLog_dlc75';
fir = find_system(m,'LookUnderMasks','all','FollowLinks','on','BlockType','DiscreteFir');
dp = get_param(fir{1},'Parent');
while ~strcmp(get_param(dp,'Parent'), m), dp = get_param(dp,'Parent'); end
uds = find_system(dp,'SearchDepth',1,'BlockType','UnitDelay');
fprintf('inside dp: %d UnitDelay\n', numel(uds));
for i=1:numel(uds)
    h=get_param(uds{i},'PortHandles');
    lh=get_param(h.Outport(1),'Line');
    if lh<=0, fprintf('  %s : no line\n', get_param(uds{i},'Name')); continue; end
    d=get_param(lh,'DstBlockHandle');
    names=arrayfun(@(x) sprintf('%s[%s]', get_param(x,'Name'), get_param(x,'BlockType')), d, 'uni', 0);
    fprintf('  %-16s -> %s\n', get_param(uds{i},'Name'), strjoin(names,', '));
end
close_system(m,0);
end
