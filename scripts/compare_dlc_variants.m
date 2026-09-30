function T = compare_dlc_variants(names, tags)
%COMPARE_DLC_VARIANTS Compact handling and wheel-hop comparison from ERD.
if nargin < 1
    names = {'old','slew010','slew005','slew002','passive'};
    root = fileparts(fileparts(mfilename('fullpath')));
    tags = cellfun(@(n) fullfile(root,'scratchpad',['DLC_NLCSNN_' n]), ...
        names, 'UniformOutput', false);
    tags{1} = fullfile(root,'scratchpad','DLC_NLCSNN_active');
    tags{5} = fullfile(root,'scratchpad','DLC_passive_noSAS');
end
addpath(fullfile(fileparts(mfilename('fullpath')),'lib'));
n = numel(tags);
eyRms=zeros(n,1); eyPeak=zeros(n,1); ltrPeak=zeros(n,1); rollPeak=zeros(n,1);
cmpFull=zeros(n,1); cmpMid=zeros(n,1); cmpTail=zeros(n,1); fdFull=zeros(n,1);
for k=1:n
    D=func_ReadERD(tags{k}); fs=1/median(diff(D.t));
    ey=D.Lat_Veh-D.Lat_Targ;
    V=[D.CmpRD_L1 D.CmpRD_L2 D.CmpRD_R1 D.CmpRD_R2];
    F=[D.Fd_L1 D.Fd_L2 D.Fd_R1 D.Fd_R2];
    VB=bandpass(V,[8 30],fs); FB=bandpass(F,[8 30],fs);
    mid=D.t>=5.3 & D.t<=6.2; tail=D.t>=6.2 & D.t<=7.8;
    eyRms(k)=sqrt(mean(ey.^2)); eyPeak(k)=max(abs(ey));
    ltrPeak(k)=max(abs(D.LTR)); rollPeak(k)=max(abs(D.Roll));
    cmpFull(k)=sqrt(mean(VB(:).^2));
    x=VB(mid,:); cmpMid(k)=sqrt(mean(x(:).^2));
    x=VB(tail,:); cmpTail(k)=sqrt(mean(x(:).^2));
    fdFull(k)=sqrt(mean(FB(:).^2));
end
T=table(string(names(:)),eyRms,eyPeak,ltrPeak,rollPeak, ...
    cmpFull,cmpMid,cmpTail,fdFull, ...
    'VariableNames',{'variant','eyRms','eyPeak','ltrPeak','rollPeakDeg', ...
    'cmpBandRms','cmpBandRms_5p3_6p2','cmpBandRms_6p2_7p8','fdBandRms'});
disp(T);
end
