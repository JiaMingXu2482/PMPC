function R = analyze_nlcsnn_internal(matFile)
%ANALYZE_NLCSNN_INTERNAL Summarize current switching and passivity activity.
if nargin < 1 || isempty(matFile)
    root = fileparts(fileparts(mfilename('fullpath')));
    matFile = fullfile(root,'scratchpad','nlcsnn_internal_signals.mat');
end
S = load(matFile);
[ti,ii] = unique(S.i_cmd_100Hz_t,'last');
I = S.i_cmd_100Hz(ii,:);
[tp,ip] = unique(S.v_plant_1kHz_t,'last');
V = S.v_plant_1kHz(ip,:);
Fr = S.F_raw_1kHz(ip,[1 3 2 4]);
Fs = S.F_safe_1kHz(ip,[1 3 2 4]);
windows = [0 tp(end); 3.0 3.8; 5.3 6.2; 6.2 7.8];
corners = {'L1','R1','L2','R2'};
R = repmat(struct('window',[],'corner','', 'iMean',[], 'iStd',[], ...
    'diRms',[], 'largeStepPct',[], 'iAtSoftPct',[], 'rawActivePct',[], ...
    'safeAtFloorPct',[], 'rawToSafeRms',[], 'safeCEffMedian',[]),0,1);

fprintf('NLCSNN internal signal diagnosis\n');
for iw = 1:size(windows,1)
    t0=windows(iw,1); t1=windows(iw,2);
    ki=ti>=t0 & ti<=t1; kp=tp>=t0 & tp<=t1;
    fprintf('\nwindow %.1f-%.1f s\n',t0,t1);
    for j=1:4
        ij=I(ki,j); dij=diff(ij);
        v=V(kp,j); fr=Fr(kp,j); fs=Fs(kp,j);
        valid=abs(v)>=5;
        ce=abs(fs(valid)./v(valid));
        % v_plant uses rebound-positive NLCSNN polarity, whereas force uses
        % CarSim polarity; passive operation therefore requires F*v <= 0.
        rawActive=fr.*v>0;
        atFloor=valid & abs(fs+0.8*v)<=max(3,0.08*abs(0.8*v));
        q=struct('window',[t0 t1],'corner',corners{j}, ...
            'iMean',mean(ij),'iStd',std(ij),'diRms',sqrt(mean(dij.^2)), ...
            'largeStepPct',100*mean(abs(dij)>=0.05), ...
            'iAtSoftPct',100*mean(ij>=1.55), ...
            'rawActivePct',100*mean(rawActive), ...
            'safeAtFloorPct',100*mean(atFloor(valid)), ...
            'rawToSafeRms',sqrt(mean((fs-fr).^2)), ...
            'safeCEffMedian',median(ce));
        R(end+1)=q; %#ok<AGROW>
        fprintf('%s: I %.3f+/-%.3f A, dI_rms %.3f, |dI|>=.05 %.1f%%, Isoft %.1f%%, raw active %.1f%%, floor %.1f%%, correction %.1f N, c50 %.2f\n', ...
            corners{j},q.iMean,q.iStd,q.diRms,q.largeStepPct,q.iAtSoftPct, ...
            q.rawActivePct,q.safeAtFloorPct,q.rawToSafeRms,q.safeCEffMedian);
    end
end
end
