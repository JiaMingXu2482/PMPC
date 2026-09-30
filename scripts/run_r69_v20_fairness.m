function R = run_r69_v20_fairness()
%RUN_R69_V20_FAIRNESS Compare MPC, ZENG and PMPC on R69 at 20 m/s.
% Each controller has a registered JT72 CarSim Run. All three share the
% R69/72 km/h procedure and disable the ZENG-only longitudinal speed limit.

startup_pmpc();
root = func_ProjectRoot();
outdir = fullfile(root, 'results', 'r69_v20_fairness');
if ~exist(outdir, 'dir'), mkdir(outdir); end
registry = jsondecode(fileread(fullfile(outdir, 'registered_runs.json')));

template = fileread(fullfile(root, 'simfile.sim'));
wd = regexp(template, 'WORK_DIR\)\$\s+([^\r\n]+)', 'tokens', 'once');
assert(~isempty(wd), 'run_r69_v20_fairness:BadSimfile', 'simfile.sim 缺少 WORK_DIR。');
assert(func_CarSimRunning(), 'run_r69_v20_fairness:NoCarSim', ...
    '请保持 CarSim Browser 运行。');
func_CarSimLib();
load_system('pmpc_mil');
closeModel = onCleanup(@() close_system('pmpc_mil', 0)); %#ok<NASGU>
clearSimfile = onCleanup(@() evalin('base', 'clear PMPC_SIMFILE')); %#ok<NASGU>

opt = struct('wp', fullfile(root, 'data', 'generated', 'WayPoints_Type5_R69.mat'), 'mu', 0.85);
cases = {'MPC',  2, 0; 'ZENG', 2, 1; 'PMPC', 1, 0};
R = struct('controller', {}, 'ey_pk_m', {}, 'ey_rms_m', {});
assignin('base', 'PMPC_VERBOSE', 0);

for k = 1:size(cases, 1)
    tag = cases{k,1};
    reg = registry.Runs(strcmp({registry.Runs.Tag}, tag));
    assert(numel(reg) == 1, 'run_r69_v20_fairness:BadRegistry', '注册记录缺少 %s。', tag);
    runName = ['Run_' reg.Guid];
    resdir = fullfile(strtrim(wd{1}), 'Results', runName);
    runpar = fullfile(resdir, 'Run_all.par');
    assert(exist(runpar, 'file') == 2, 'run_r69_v20_fairness:NoRun', ...
        '找不到 %s 的 CarSim 输入文件：%s', tag, runpar);
    par = fileread(runpar);
    assert(numel(regexp(par, '(?m)^SV_VXS\s+72\s*$', 'match')) == 1 && ...
           contains(par, 'DEFINE_PARAMETER PMPC_ZENGLONG = 0'), ...
           'run_r69_v20_fairness:WrongSpeed', '%s 的车速或纵向设置不正确。', tag);
    simfile = fullfile(outdir, ['r69_v20_' tag '.sim']);
    local_write_simfile(template, simfile, runName);
    assignin('base', 'PMPC_SIMFILE', simfile);
    set_param('pmpc_mil/CarSim', 'SIMFILE', fullfile('results', 'r69_v20_fairness', ['r69_v20_' tag '.sim']));

    evalin('base', 'clear PMPC_MODE PMPC_ZENGRHO');
    evalin('base', 'clear PMPC_P');
    evalin('base', 'setup_pmpc;');
    P = evalin('base', 'PMPC_P');
    P.Constraints.ZengLong_on = 0;
    P.S0.Constraints.ZengLong_on = 0;
    assignin('base', 'PMPC_P', P);
    assert(P.S0.Constraints.ControllerMode == cases{k,2} && ...
           P.S0.Constraints.ZengRho_on == cases{k,3} && ...
           P.S0.Constraints.ZengLong_on == 0, ...
           'run_r69_v20_fairness:WrongMode', '控制器模式设置不正确：%s', tag);

    sim('pmpc_mil');
    src = fullfile(resdir, 'LastRun');
    D = func_ReadERD(src);
    assert(strcmp(D.Dataset, reg.Name), 'run_r69_v20_fairness:WrongDataset', ...
        '%s 的 ERD 数据集标签不正确：%s', tag, D.Dataset);
    assert(abs(D.Vx(1) - 72) < 0.05 && abs(D.VxTarget(1) - 72) < 0.05, ...
        'run_r69_v20_fairness:WrongInitialSpeed', ...
        '%s 实际初速度 %.3f km/h，目标 %.3f km/h；要求两者均为 72 km/h。', ...
        tag, D.Vx(1), D.VxTarget(1));
    assert(D.t(end) >= 9.95, 'run_r69_v20_fairness:EarlyStop', ...
        '%s 提前在 %.3f s 停止。', tag, D.t(end));
    M = func_Metrics(D, opt);
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

function local_write_simfile(template, filename, runName)
old = regexp(template, '(?m)SET_MACRO \$\(ROOT_FILE_NAME\)\$\s+(\S+)', 'tokens', 'once');
assert(~isempty(old), 'run_r69_v20_fairness:BadSimfile', 'simfile.sim 缺少 ROOT_FILE_NAME。');
t = strrep(template, old{1}, runName);
fid = fopen(filename, 'w');
assert(fid >= 0, 'run_r69_v20_fairness:WriteFailed', '不能写入 %s', filename);
fwrite(fid, t, 'char');
fclose(fid);
end
