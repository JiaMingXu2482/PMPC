function probe_bd()
here=fileparts(mfilename('fullpath'));
if bdIsLoaded('PmpcLog_dlc75'), close_system('PmpcLog_dlc75',0); end
load_system(fullfile(here,'PmpcLog_dlc75.slx'));
m='PmpcLog_dlc75';
subs = find_system(m,'SearchDepth',1,'BlockType','SubSystem');
for i=1:numel(subs)
    fprintf('TOP: %s\n', get_param(subs{i},'Name'));
    mfs = find_system(subs{i},'SearchDepth',1,'BlockType','SubSystem');
    for k=1:numel(mfs)
        if ~contains(get_param(mfs{k},'Name'),'MATLAB Function'), continue; end
        ph=get_param(mfs{k},'PortHandles');
        fprintf('   MF %-18s in=%d out=%d  SID=%s\n', get_param(mfs{k},'Name'), ...
            numel(ph.Inport), numel(ph.Outport), get_param(mfs{k},'SID'));
    end
end
close_system(m,0);
end
