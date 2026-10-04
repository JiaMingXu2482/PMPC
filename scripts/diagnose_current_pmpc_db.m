function R = diagnose_current_pmpc_db()
%DIAGNOSE_CURRENT_PMPC_DB Log QP yaw request, longitudinal request and brakes.
% Logging is enabled only in memory; the Simulink model is not saved.
root = startup_pmpc();
simfile = fullfile(root,'simfile.sim');
sf = fileread(simfile);
runId = regexp(sf,'ROOT_FILE_NAME\)\$\s+(Run_[^\s]+)','tokens','once');
wd = regexp(sf,'WORK_DIR\)\$\s+([^\r\n]+)','tokens','once');
assert(~isempty(runId) && ~isempty(wd),'Invalid simfile.sim');
resdir = fullfile(strtrim(wd{1}),'Results',runId{1});
expanded = fullfile(resdir,'Run_all.par');
func_SimModel(expanded,'pmpc_mil');
assert(func_CarSimRunning(),'CarSim must be running');
func_CarSimLib();
assignin('base','PMPC_SIMFILE',simfile);
assignin('base','PMPC_VERBOSE',0);
mil_init_PMPC();
mdl = 'pmpc_mil';
load_system(mdl);
set_param([mdl '/CarSim'],'SIMFILE','simfile.sim');
stopTime = func_CarSimStopTime(fileread(expanded));
if isfinite(stopTime) && stopTime > 0
    set_param(mdl,'StopTime',num2str(stopTime,'%.12g'));
end
set_param(mdl,'SignalLogging','on');
ph = get_param([mdl '/PMPC_MF'],'PortHandles');
set_param(ph.Outport(1),'DataLogging','on', ...
    'DataLoggingNameMode','Custom','DataLoggingName','db_sys');
set_param(ph.Outport(3),'DataLogging','on', ...
    'DataLoggingNameMode','Custom','DataLoggingName','db_long_diag');
out = sim(mdl);
X = out.logsout.getElement('db_sys').Values;
L = out.logsout.getElement('db_long_diag').Values;
sx = local_time_first(X);
sl = local_time_first(L);
tx = X.Time(:);
tl = L.Time(:);
[tx,ix] = unique(tx,'last'); sx = sx(ix,:);
[tl,il] = unique(tl,'last'); sl = sl(il,:);
[matched,j] = ismember(round(tx*1e6),round(tl*1e6));
assert(all(matched),'Controller outputs are not time aligned');
sl = sl(j,:);
D = func_ReadERD(fullfile(resdir,'LastRun'));
station = interp1(D.t,D.Station,tx,'linear','extrap');
dlc = station >= 0 & station <= 200;
mask = station >= 280 & station <= 389.956;
assert(any(mask),'The current CarSim Run has no R70 J-turn interval');
fprintf('DLC: mean/max |MFx_req| %.2f / %.2f N*m, gamma_DB > 0.01 on %.1f%% of ticks\n', ...
    mean(abs(sx(dlc,18))),max(abs(sx(dlc,18))),100*mean(sx(dlc,15)>0.01));
R.dataset = D.Dataset;
R.t = tx(mask);
R.station = station(mask);
R.mfx_request = sx(mask,18);
R.mfx_limit = sx(mask,17);
R.afs_request = sx(mask,24);
R.gamma_db = sx(mask,15);
R.fx_request = sl(mask,9);
R.fx_achieved = sl(mask,10);
R.brake_cmd = sx(mask,1:4); % [L1 L2 R1 R2], N*m
R.left_right_cmd = sum(R.brake_cmd(:,1:2),2)-sum(R.brake_cmd(:,3:4),2);
R.total_brake_cmd = sum(R.brake_cmd,2);
R.brake_actual = [interp1(D.t,D.My_Bk_L1,R.t), ...
    interp1(D.t,D.My_Bk_L2,R.t), ...
    interp1(D.t,D.My_Bk_R1,R.t), ...
    interp1(D.t,D.My_Bk_R2,R.t)];
R.left_right_actual = sum(R.brake_actual(:,1:2),2) ...
    -sum(R.brake_actual(:,3:4),2);
fprintf('J-turn samples: %d | mean/max |MFx_req|: %.2f / %.2f N*m\n', ...
    numel(R.t),mean(abs(R.mfx_request)),max(abs(R.mfx_request)));
fprintf('gamma_DB mean/max: %.4f / %.4f | |MFx_req| > 50 N*m: %.1f%%\n', ...
    mean(R.gamma_db),max(R.gamma_db),100*mean(abs(R.mfx_request)>50));
fprintf('MFx physical bound mean/max: %.2f / %.2f N*m | mean |AFS request|: %.2f N\n', ...
    mean(R.mfx_limit),max(R.mfx_limit),mean(abs(R.afs_request)));
fprintf('Fx_req mean/max: %.2f / %.2f N | Fx_req > 1 N: %.1f%%\n', ...
    mean(R.fx_request),max(R.fx_request),100*mean(R.fx_request>1));
fprintf('mean |left-right brake|: command %.2f N*m, CarSim %.2f N*m\n', ...
    mean(abs(R.left_right_cmd)),mean(abs(R.left_right_actual)));
idle = R.fx_request <= 1 & abs(R.mfx_request) <= 1;
fprintf('No request on %.1f%% of J-turn ticks; mean brake sum there %.2f N*m, >100 N*m on %.1f%% of those ticks\n', ...
    100*mean(idle),mean(R.total_brake_cmd(idle)), ...
    100*mean(R.total_brake_cmd(idle)>100));
for t0 = [11 12 13 14 15]
    [~,k] = min(abs(R.t-t0));
    fprintf('t=%.2f s: MFx=%.1f, gamma=%.3f, Fx_req=%.1f, four-wheel cmd=[%.1f %.1f %.1f %.1f] N*m\n', ...
        R.t(k),R.mfx_request(k),R.gamma_db(k),R.fx_request(k),R.brake_cmd(k,:));
end
close_system(mdl,0);
end

function data = local_time_first(values)
data = values.Data;
n = numel(values.Time);
if size(data,1) == n, return; end
timeDim = find(size(data) == n,1,'last');
assert(~isempty(timeDim),'Cannot locate logged time dimension');
order = [timeDim 1:timeDim-1 timeDim+1:ndims(data)];
data = permute(data,order);
data = reshape(data,n,[]);
end
