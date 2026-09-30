function R = diagnose_nlcsnn_chatter(activeTag, passiveTag)
%DIAGNOSE_NLCSNN_CHATTER Quantify local DLC damper chatter from CarSim ERD.

if nargin < 1 || isempty(activeTag)
    activeTag = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        'scratchpad', 'DLC_NLCSNN_active');
end
if nargin < 2 || isempty(passiveTag)
    passiveTag = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        'scratchpad', 'DLC_passive_noSAS');
end

addpath(fullfile(fileparts(mfilename('fullpath')), 'lib'));
A = func_ReadERD(activeTag);
P = func_ReadERD(passiveTag);
windows = [0 A.t(end); 3.0 3.8; 5.3 6.2; 6.2 7.8];
corners = {'L1','L2','R1','R2'};
R = repmat(struct('window',[],'corner','', ...
    'velBandRmsActive',[],'velBandRmsPassive',[], ...
    'forceBandRmsActive',[],'forceBandRmsPassive',[], ...
    'dominantHzActive',[],'dominantHzPassive',[], ...
    'medianCEffActive',[],'medianCEffPassive',[], ...
    'nearFloorPctActive',[],'nearFloorPctPassive',[]), 0, 1);

fprintf('NLCSNN chatter diagnosis (band 8-30 Hz)\n');
for iw = 1:size(windows,1)
    t0 = windows(iw,1); t1 = windows(iw,2);
    fprintf('\nwindow %.1f-%.1f s\n', t0, t1);
    for ic = 1:numel(corners)
        c = corners{ic};
        va = A.(['CmpRD_' c]); fa = A.(['Fd_' c]);
        vp = P.(['CmpRD_' c]); fp = P.(['Fd_' c]);
        ia = A.t >= t0 & A.t <= t1;
        ip = P.t >= t0 & P.t <= t1;
        [vrA, fA, dA, ceA, floorA] = local_metrics_(A.t(ia), va(ia), fa(ia));
        [vrP, fP, dP, ceP, floorP] = local_metrics_(P.t(ip), vp(ip), fp(ip));
        k = numel(R)+1;
        R(k) = struct('window',[t0 t1],'corner',c, ...
            'velBandRmsActive',vrA,'velBandRmsPassive',vrP, ...
            'forceBandRmsActive',fA,'forceBandRmsPassive',fP, ...
            'dominantHzActive',dA,'dominantHzPassive',dP, ...
            'medianCEffActive',ceA,'medianCEffPassive',ceP, ...
            'nearFloorPctActive',floorA,'nearFloorPctPassive',floorP); %#ok<AGROW>
        fprintf('%s: CmpRD %.3f/%.3f mm/s, Fd %.2f/%.2f N, peakHz %.1f/%.1f, c50 %.3f/%.3f, c<=1.0 %.1f/%.1f %%\n', ...
            c, vrA, vrP, fA, fP, dA, dP, ceA, ceP, floorA, floorP);
    end
end
end

function [vBandRms, fBandRms, dominantHz, cMedian, nearFloorPct] = local_metrics_(t,v,f)
fs = 1/median(diff(t));
vBand = bandpass(v,[8 30],fs);
fBand = bandpass(f,[8 30],fs);
vBandRms = sqrt(mean(vBand.^2));
fBandRms = sqrt(mean(fBand.^2));
nfft = max(256,2^nextpow2(numel(v)));
[pxx,ff] = periodogram(v-mean(v),[],nfft,fs);
band = ff >= 8 & ff <= 30;
fb = ff(band); pb = pxx(band);
if isempty(pb), dominantHz = NaN; else, [~,im] = max(pb); dominantHz = fb(im); end
valid = abs(v) >= 5 & f.*v >= 0;
ce = abs(f(valid)./v(valid));
if isempty(ce)
    cMedian = NaN; nearFloorPct = NaN;
else
    cMedian = median(ce);
    nearFloorPct = 100*mean(ce <= 1.0);
end
end
