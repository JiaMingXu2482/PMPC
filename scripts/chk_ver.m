function chk_ver()
%CHK_VER  自检: 确认当前用的 .m 是不是最新修过的那份
%   在 2018a 上跑 sim 之前先跑这个, 比对着猜快得多。
fprintf('\n===== 文件来源 =====\n');
for n = {'setup_pmpc','pmpc_step','func_SolveMPCQP','func_RefTraj_LocalPlanning','func_QPA_DB'}
    fprintf('  %-28s %s\n', n{1}, which(n{1}));
end
fprintf('  当前目录 %s\n', pwd);

fprintf('\n===== 修复标记 =====\n');
chk = { 'setup_pmpc',                 'MPCParameters.Zg_Npv',   'Zg_Npv 搬进 MPCParameters'
        'pmpc_step',                  'MPCParameters.Zg_Npv',   'pmpc_step 用 MPCParameters.Zg_Npv'
        'pmpc_step',                  'min(V_cand(:))',         'V_cand 强制列向量'
        'func_RefTraj_LocalPlanning',  'min(interp_dists(:))',  'interp_dists 强制列向量'
        'func_SolveMPCQP',            'bsxfun(@times',          'bsxfun 代替隐式扩展'
        'func_SolveMPCQP',            'fprintf(2',              'warning 换成 fprintf'
        'func_QPKwik',                'local_warmstart',        'persistent 已隔离'
        'func_StateEstimation',       'ModelInput(50)',         '输入已重编号成 50 路'
        'func_PortMap',               'P.dropped',              'PortMap 已是 50 入' };
bad = 0;
for i = 1:size(chk,1)
    p = which(chk{i,1});
    if isempty(p), fprintf('  [缺失] %s\n', chk{i,1}); bad = bad + 1; continue; end
    t = fileread(p);
    if isempty(strfind(t, chk{i,2}))
        fprintf('  [旧版] %-30s 缺 "%s"\n', chk{i,3}, chk{i,2});  bad = bad + 1;
    else
        fprintf('  [ok]   %s\n', chk{i,3});
    end
end
fprintf('\n');
if bad > 0
    fprintf(2, '  !! %d 项不通过 —— 这些文件是旧的, 重新拷包再跑\n\n', bad);
else
    fprintf('  全部通过, 可以 sim\n\n');
end
end
