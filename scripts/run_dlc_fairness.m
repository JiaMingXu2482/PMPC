function R = run_dlc_fairness()
%RUN_DLC_FAIRNESS Compare MPC, ZENG and PMPC on DLC80 at mu=0.5.
% All controllers use the same 80 km/h maneuver and disable the
% ZENG-only longitudinal speed limit.

startup_pmpc();
root = func_ProjectRoot();
outdir = fullfile(root, 'results', 'dlc80_fairness');
if ~exist(outdir, 'dir'), mkdir(outdir); end
assert(func_CarSimRunning(), 'run_dlc_fairness:NoCarSim', ...
    '请保持 CarSim Browser 运行。');
func_CarSimLib();

cases = {'MPC'; 'ZENG'; 'PMPC'};
R = struct('controller', {}, 'ey_pk_m', {}, 'ey_rms_m', {});
assignin('base', 'PMPC_VERBOSE', 0);

for k = 1:numel(cases)
    tag = cases{k};
    info = run_ds(tag, '-nosim', '-ds', 'DLC80_mu0.5');
    oldLog = dir(fullfile(info.resdir, 'LastRun_log.txt'));
    if isempty(oldLog), logstamp = 0; else, logstamp = oldLog.datenum; end

    evalin('base', 'clear PMPC_MODE PMPC_ZENGRHO PMPC_P MPC_P ZENG_P');
    switch info.model
        case 'mpc_mil'
            mil_init_MPC;
            bundleName = 'MPC_P';
        case 'zeng_mil'
            mil_init_ZENG;
            bundleName = 'ZENG_P';
        case 'pmpc_mil'
            mil_init_PMPC;
            bundleName = 'PMPC_P';
    end
    P = evalin('base', bundleName);
    P.Constraints.ZengLong_on = 0;
    P.S0.Constraints.ZengLong_on = 0;
    assignin('base', bundleName, P);
    assert(P.S0.Constraints.ZengLong_on == 0, ...
           'run_dlc_fairness:WrongMode', '控制器模式设置不正确：%s', tag);

    load_system(info.model);
    set_param([info.model '/CarSim'], 'SIMFILE', 'simfile.sim');
    sim(info.model);
    close_system(info.model, 0);
    func_WaitERD(info.resdir, logstamp);
    src = fullfile(info.resdir, 'LastRun');
    D = func_ReadERD(src);
    assert(strcmp(D.Dataset, info.name), 'run_dlc_fairness:WrongDataset', ...
        '%s 的 ERD 数据集标签不正确：%s', tag, D.Dataset);
    assert(abs(D.Vx(1) - 80) < 0.05 && abs(D.VxTarget(1) - 80) < 0.05, ...
        'run_dlc_fairness:WrongInitialSpeed', ...
        '%s 实际初速度 %.3f km/h，目标 %.3f km/h；要求两者均为 80 km/h。', ...
        tag, D.Vx(1), D.VxTarget(1));

    M = func_Metrics(D);
    R(k).controller = tag;
    R(k).ey_pk_m = M.ey_pk;
    R(k).ey_rms_m = M.ey_rms;
    copyfile([src '.vs'], fullfile(outdir, [tag '.vs']));
    copyfile([src '.vsb'], fullfile(outdir, [tag '.vsb']));
    fprintf('%s: Vx(0)=%.3f km/h, e_y peak=%.6f m, RMS=%.6f m\n', ...
        tag, D.Vx(1), M.ey_pk, M.ey_rms);
end

save(fullfile(outdir, 'ey_metrics.mat'), 'R');
end
