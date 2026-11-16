function name = cs_dataset_name(simfile)
%CS_DATASET_NAME 从 CarSim 的 simfile 里取出 Run_<dataset> 目录名。
%
%   name = cs_dataset_name('simfile_dlc.sim')
%
% 每个 worktree / 每个 agent 指向各自独立的 dataset 副本，所以 run_*.m
% 不再写死 Run_ 目录名，一律从 simfile 推导。这样三个 agent 并发时不会
% 互相改写同一份 Run_all.par（SV_VXS / TSTOP / SSTOP / M_PL）。
%
% 返回空字符串表示 simfile 里没有 Run_* 名。

if ~isfile(simfile)
    name = '';
    return
end
txt  = fileread(simfile);
tok  = regexp(txt, 'Run_[A-Za-z0-9_\-]+', 'match', 'once');
if isempty(tok)
    name = '';
else
    % 去掉可能粘上的 .par / 尾部分隔符
    name = regexprep(tok, '\.par$', '');
end
end
