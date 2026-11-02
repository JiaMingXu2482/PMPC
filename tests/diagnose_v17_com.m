function diagnose_v17_com(resultDir)
%DIAGNOSE_V17_COM Read the latest CarSim run without changing its results.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'scripts','lib'));
D = func_ReadERD(fullfile(resultDir,'LastRun'));
F = [D.Fd_L1,D.Fd_R1,D.Fd_L2,D.Fd_R2];
v = [D.CmpRD_L1,D.CmpRD_R1,D.CmpRD_L2,D.CmpRD_R2];
mask = D.t >= 0.5 & all(isfinite(F),2) & all(isfinite(v),2);
active = mask & any(abs(v)>20,2);
power = F .* v;
fprintf('damper |F| p50/p99/max = %.0f / %.0f / %.0f N\n', ...
    prctile(abs(F(mask,:)),50,'all'),prctile(abs(F(mask,:)),99,'all'),max(abs(F(mask,:)),[],'all'));
fprintf('damper F*v negative fraction (|v|>20 mm/s) = %.1f%%; signed integral = %.3g N*m\n', ...
    100*mean(power(active,:)<0,'all'),sum(power(mask,:),'all')*0.001/1000);
sw = D.Steer_SW;
fprintf('steer |rate| p99/max = %.1f / %.1f deg/s\n', ...
    prctile(abs(diff(sw)/0.001),99),max(abs(diff(sw)/0.001)));
S = load(fullfile(resultDir,'pmpc_mil_logs.mat'),'logsout');
for k=1:S.logsout.numElements
    e = S.logsout.getElement(k);
    fprintf('log %s: size %s, N=%d\n',e.Name,mat2str(size(e.Values.Data)),numel(e.Values.Time));
end
fdLog = S.logsout.get('damper_F_des').Values;
cmd = reshape(fdLog.Data,4,[]).';
meas = interp1(D.t,F,fdLog.Time,'linear','extrap');
fprintf('Fdes |F| p99 = %.0f N; |Fplant-Fdes| p50/p99 = %.0f/%.0f N\n', ...
    prctile(abs(cmd),99,'all'),prctile(abs(meas-cmd),50,'all'), ...
    prctile(abs(meas-cmd),99,'all'));
diag = S.logsout.get('priority_diag').Values.Data;
fprintf('PMPC exitflag <= 0: %.1f%%, fallback: %.1f%%; gammaDB p50/max %.3f/%.3f\n', ...
    100*mean(diag(:,11)<=0),100*mean(diag(:,12)>0), ...
    median(diag(:,10)),max(diag(:,10)));
end
