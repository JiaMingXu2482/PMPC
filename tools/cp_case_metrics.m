function T = cp_case_metrics(files)
%CP_CASE_METRICS 打印 CP_RESULTS.md 表里的那一行（每个算例一行）。
%
%   cp_case_metrics                                  % 默认主对照五个算例（固定 tau 口径，*_fix）
%   cp_case_metrics({f1,f2})                         % 指定
%   T = cp_case_metrics(...)                         % 返回 table
%
% 数据全部来自 .mat 的 D（27 x N）与 t，行序见 pmpc_core/pmpc_step.m 头注释：
%   D(1) ey [m]    D(2) epsi [rad]  D(5) LTR     D(6) Md_actual  D(7) Md_cmd
%   D(18) Mz_cmd   D(19) SW [deg]   D(20) Vx [m/s]  D(24) QP_OK
% Icmd 为四角 1 kHz 电流（N x 4）。
% tau_d 取 .mat 里的 tau_dtrace（run_*.m 用 tau_d_effective.m 事后复算）。
% Mz_avg / Md_avg 都是 mean(abs())。
%
% QP% 口径（2026-11-14 统一，三方必须一致）：
%   D(24) = 1 本拍主 QP 收敛；0 未收敛；-1 表示本拍没跑 QP（pmpc_step 在
%   无效测量/无参考路径等条件下提前返回，留 -1 当哨兵）。
%   所以分母只数 D(24) >= 0 的帧，**不把 -1 当失败**：
%       QP% = 100 * sum(D(24)>0.5) / sum(D(24)>=0)
%
% Vx 范围按 km/h 打印：三方对比时先看这一列，确认大家跑的是同一个车速
% （基线那支没有纵向速度环，曾出现过 DLC 掉到 0.6 km/h 的算例）。
%
% 默认清单是主对照用的固定 tau 版（tau_d_mode=1，*_fix）。时变口径（mode=2）的
% 五个算例与它们同目录、无 _fix 后缀，需要时显式传文件名，例如：
%   cp_case_metrics({'PMPC_Results/DLC_mu0.4_interp/cp_dlc_80.mat'})

if nargin < 1 || isempty(files)
    root = fileparts(fileparts(mfilename('fullpath')));
    d1 = fullfile(root,'PMPC_Results','DLC_mu0.4_interp');
    d2 = fullfile(root,'PMPC_Results','Slalom_mu0.85_interp');
    d3 = fullfile(root,'PMPC_Results','FishHook_80_mu0.9_cargo170_newplant');
    files = {fullfile(d1,'cp_dlc_70_fix.mat'), fullfile(d1,'cp_dlc_75_fix.mat'), ...
             fullfile(d1,'cp_dlc_80_fix.mat'), fullfile(d2,'cp_sla_80_fix.mat'), ...
             fullfile(d3,'cp_fh_80_fix.mat')};
end
if ischar(files) || isstring(files), files = {char(files)}; end

n = numel(files);
name = strings(n,1);
Tend = nan(n,1); eyM = Tend; eyR = Tend; epM = Tend; ltrM = Tend; swM = Tend;
mzA = Tend; mzM = Tend; mdA = Tend; mdM = Tend;
iLo = Tend; iHi = Tend; vLo = Tend; vHi = Tend; tdLo = Tend; tdHi = Tend;
qp = Tend; qpN = Tend; qpDrop = Tend;

for k = 1:n
    if ~isfile(files{k})
        name(k) = string(files{k}); continue;
    end
    [~,nm] = fileparts(files{k}); name(k) = string(nm);
    S = load(files{k});
    D = double(S.D); t = double(S.t(:));
    if size(D,1) ~= 27, D = D.'; end
    ey = D(1,:); epsi = D(2,:); ltr = D(5,:); mda = D(6,:);
    mz = D(18,:); sw = D(19,:); vx = D(20,:); q = D(24,:);
    I = S.Icmd; if isstruct(I) && isfield(I,'signals'), I = I.signals.values; end

    Tend(k) = t(end);
    eyM(k) = max(abs(ey));  eyR(k) = sqrt(mean(ey.^2));
    epM(k) = max(abs(rad2deg(epsi)));
    ltrM(k) = max(abs(ltr));
    swM(k) = max(abs(sw));
    mzA(k) = mean(abs(mz)); mzM(k) = max(abs(mz));
    mdA(k) = mean(abs(mda)); mdM(k) = max(abs(mda));
    iLo(k) = min(double(I(:))); iHi(k) = max(double(I(:)));
    vLo(k) = min(vx)*3.6; vHi(k) = max(vx)*3.6;
    if isfield(S,'tau_dtrace')
        td = double(S.tau_dtrace);
        tdLo(k) = min(td); tdHi(k) = max(td);
    end
    % QP%：分母只算真正跑了 QP 的帧（D(24)>=0）
    ran = q >= 0;
    qpN(k) = sum(ran);
    qpDrop(k) = sum(~ran);
    qp(k) = 100*sum(q(ran) > 0.5)/max(sum(ran),1);
end

T = table(name,Tend,eyM,eyR,epM,ltrM,swM,mzA,mzM,mdA,mdM,iLo,iHi,vLo,vHi, ...
    tdLo,tdHi,qp,qpN,qpDrop, ...
    'VariableNames',{'case','T','ey_max','ey_rms','epsi_max_deg','LTR_max', ...
    'SW_max_deg','Mz_avg','Mz_max','Md_avg','Md_max','Icmd_min','Icmd_max', ...
    'Vx_min','Vx_max','tau_d_min','tau_d_max','QP_pct','QP_frames','QP_skipped'});

fprintf('\n');
fprintf('%-12s %5s %8s %8s %6s %6s %7s %6s %6s %6s %6s %5s %5s %7s %7s %8s %8s %6s\n', ...
    'case','T','ey_max','ey_rms','epsi','LTRmax','SWmax','Mzavg','Mzmax', ...
    'Mdavg','Mdmax','Ilo','Ihi','Vx_lo','Vx_hi','tau_lo','tau_hi','QP%');
for k = 1:height(T)
    fprintf('%-12s %5.2f %8.4f %8.4f %6.2f %6.3f %7.1f %6.0f %6.0f %6.0f %6.0f %5.2f %5.2f %7.1f %7.1f %8.5f %8.5f %6.1f\n', ...
        T.case(k), T.T(k), T.ey_max(k), T.ey_rms(k), T.epsi_max_deg(k), ...
        T.LTR_max(k), T.SW_max_deg(k), T.Mz_avg(k), T.Mz_max(k), ...
        T.Md_avg(k), T.Md_max(k), T.Icmd_min(k), T.Icmd_max(k), ...
        T.Vx_min(k), T.Vx_max(k), T.tau_d_min(k), T.tau_d_max(k), T.QP_pct(k));
end
fprintf('\nQP 帧数（括号内为 D(24)==-1 的帧，不计入分母）：\n');
for k = 1:height(T)
    fprintf('  %-12s %4d (+%d)\n', T.case(k), T.QP_frames(k), T.QP_skipped(k));
end
fprintf('\n');
end
