function F = fopdt_sim(t, Icmd, g, tau, Lms, F0)
%FOPDT_SIM tau*dF/dt + F = g(I(t-L))，整数毫秒 L，教科书 ZOH 递推。
dt = median(diff(t));
L = round(Lms); Is = Icmd;
if L > 0, Is(L+1:end) = Icmd(1:end-L); Is(1:L) = Icmd(1); end
gin = interp1(g(1,:), g(2,:), min(max(Is,g(1,1)),g(1,end)), 'linear');
if tau <= 0
    F = gin;
else
    a = exp(-dt/tau); F = zeros(numel(t),1); F(1) = F0;
    for k = 1:numel(t)-1, F(k+1) = a*F(k) + (1-a)*gin(k); end
end
end
