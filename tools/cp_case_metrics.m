function T = cp_case_metrics(files)
%CP_CASE_METRICS 打印 CP_RESULTS.md 表里的那一行（每个算例一行）。
%
%   cp_case_metrics                       % 默认四个 cp_* 算例
%   cp_case_metrics({f1,f2})
%   T = cp_case_metrics(...)              % 返回 table
%
% 数据全部来自 .mat 的 D（27 x N）与 t，行序见 pmpc_core/pmpc_step.m 头注释：
%   D(1) ey [m]    D(2) epsi [rad]  D(5) LTR     D(6) Md_actual  D(7) Md_cmd
%   D(18) Mz_cmd   D(19) SW [deg]   D(20) Vx [m/s]  D(24) QP_OK
% Icmd 为四角 1 kHz 电流（N x 4）。
% tau_d 取 .mat 里的 tau_dtrace（run_*.m 用 tau_d_effective.m 事后复算）。
% Mz_avg / Md_avg 都是 mean(abs())。

if nargin < 1 || isempty(files)
    root = fileparts(fileparts(mfilename('fullpath')));
    files = {fullfile(root,'PMPC_Results','DLC_mu0.4_interp','cp_dlc_mode1.mat'), ...
             fullfile(root,'PMPC_Results','DLC_mu0.4_interp','cp_dlc_mode2.mat'), ...
             fullfile(root,'PMPC_Results','Slalom_mu0.85_interp','cp_sla_mode1.mat'), ...
             fullfile(root,'PMPC_Results','Slalom_mu0.85_interp','cp_sla_mode2.mat')};
end
if ischar(files) || isstring(files), files = {char(files)}; end

name = strings(numel(files),1);
Tend = nan(numel(files),1);eyM=nan(numel(files),1);eyR=eyM;epM=eyM;
ltrM=eyM;swM=eyM;mzA=eyM;mzM=eyM;mdA=eyM;mdM=eyM;
iLo=eyM;iHi=eyM;tdLo=eyM;tdHi=eyM;qp=eyM;
for k = 1:numel(files)
    [~,nm] = fileparts(files{k}); name(k) = string(nm);
    S = load(files{k});
    D = double(S.D); t = double(S.t(:));
    if size(D,1) ~= 27, D = D.'; end
    ey = D(1,:); epsi = D(2,:); ltr = D(5,:); mda = D(6,:);
    mz = D(18,:); sw = D(19,:); q = D(24,:);
    I = S.Icmd; if isstruct(I) && isfield(I,'signals'), I = I.signals.values; end
    Tend(k) = t(end);
    eyM(k) = max(abs(ey));  eyR(k) = sqrt(mean(ey.^2));
    epM(k) = max(abs(rad2deg(epsi)));
    ltrM(k) = max(abs(ltr));
    swM(k) = max(abs(sw));
    mzA(k) = mean(abs(mz)); mzM(k) = max(abs(mz));
    mdA(k) = mean(abs(mda)); mdM(k) = max(abs(mda));
    iLo(k) = min(double(I(:))); iHi(k) = max(double(I(:)));
    if isfield(S,'tau_dtrace')
        td = double(S.tau_dtrace);
        tdLo(k) = min(td); tdHi(k) = max(td);
    end
    qp(k) = 100*mean(q > 0.5);
end

T = table(name,Tend,eyM,eyR,epM,ltrM,swM,mzA,mzM,mdA,mdM,iLo,iHi,tdLo,tdHi,qp, ...
    'VariableNames',{'case','T','ey_max','ey_rms','epsi_max_deg','LTR_max', ...
    'SW_max_deg','Mz_avg','Mz_max','Md_avg','Md_max','Icmd_min','Icmd_max', ...
    'tau_d_min','tau_d_max','QP_pct'});

fprintf('\n');
fprintf('%-14s %5s %8s %8s %7s %7s %7s %7s %7s %7s %7s %6s %6s %9s %9s %6s\n', ...
    'case','T','ey_max','ey_rms','epsi°','LTRmax','SWmax','Mz_avg','Mz_max', ...
    'Md_avg','Md_max','Ilo','Ihi','tau_d_lo','tau_d_hi','QP%');
for k = 1:height(T)
    fprintf('%-14s %5.2f %8.4f %8.4f %7.2f %7.3f %7.1f %7.0f %7.0f %7.0f %7.0f %6.2f %6.2f %9.5f %9.5f %6.1f\n', ...
        T.case(k), T.T(k), T.ey_max(k), T.ey_rms(k), T.epsi_max_deg(k), ...
        T.LTR_max(k), T.SW_max_deg(k), T.Mz_avg(k), T.Mz_max(k), ...
        T.Md_avg(k), T.Md_max(k), T.Icmd_min(k), T.Icmd_max(k), ...
        T.tau_d_min(k), T.tau_d_max(k), T.QP_pct(k));
end
fprintf('\n');
end
