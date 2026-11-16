function [t, I, t0, exc_end] = gen_prof(prof, I0, I1, warm)
%GEN_PROF 生成电流波形（1 ms 网格）。前 warm 秒保持 I0，之后按 prof 变化。
dt = 1e-3; t0 = warm;
switch lower(prof)
    case 'step',   T = 0.60; Tr = 0;
    case 'ramp10', T = 0.60; Tr = 0.010;
    case 'ramp50', T = 0.60; Tr = 0.050;
    case 'smooth', T = 0.80; Tr = 0;
    otherwise, error('未知 prof');
end
t = (0:dt:(t0+T)).';
I = I0*ones(size(t));
switch lower(prof)
    case 'step'
        I(t>=t0) = I1;
    case {'ramp10','ramp50'}
        kk = t>=t0 & t<t0+Tr;
        I(kk) = I0 + (I1-I0)*(t(kk)-t0)/Tr;
        I(t>=t0+Tr) = I1;
    case 'smooth'
        Tper = 0.20;
        ks = t>=t0 & t<t0+2*Tper;
        I(ks) = I0 + (I1-I0)*0.5*(1-cos(2*pi*(t(ks)-t0)/Tper));
        I(t>=t0+2*Tper) = I0;
end
% 10 ms 零阶保持（与 inv_current_impl_10ms 的更新周期一致）
ticks = floor(t/0.01 + 1e-9);
for k = 2:numel(t)
    if ticks(k)==ticks(k-1), I(k)=I(k-1); end
end
exc_end = t0 + max(Tr, 0.40*(strcmpi(prof,'smooth')));
end
