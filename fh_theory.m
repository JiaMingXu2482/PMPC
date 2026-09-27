function fh_theory
%FH_THEORY  固定转角下 LTR 随入弯车速怎么变 —— 用实测轮胎表解稳态转向平衡
%  平衡:  m*V*r = Fyf*cos(d) + Fyr        (侧向)
%         0     = lf*Fyf*cos(d) - lr*Fyr  (横摆)
%  alpha_f = atan((vy+lf*r)/V) - d ,  alpha_r = atan((vy-lr*r)/V)
%  LTR = k_LTR * a_y ,  a_y = V*r
P  = func_VehicleParams(1);
mu = 0.9;  muE = 0.78;  g = P.g;
TF = func_TireTable('carsim', mu, 2);
TR = func_TireTable('carsim', mu, 2);
Fzf = P.m*g*P.lr/P.L;   Fzr = P.m*g*P.lf/P.L;      % 轴载
cphi = P.ms*P.h_S2R/(P.Kt - P.ms*g*P.h_S2R);
kap  = P.m*P.h_TL - P.ms*P.h_S2R;  bta = 2/(P.m*g*P.tf);
kLTR = bta*(kap + P.Kt*cphi);
ayth = g*(P.tf/2)/(P.h_SL + P.h_S2R*P.ms*g*P.h_S2R/(P.Kt-P.ms*g*P.h_S2R));

d = 0.176;                                          % 鱼钩保持的前轮转角 (实测峰 0.178)
fprintf('前轮转角 %.4f rad = %.2f deg | k_LTR = %.5f | a_y,th = %.3f m/s^2\n\n', ...
        d, rad2deg(d), kLTR, ayth);
fprintf('%7s %9s %9s %9s %9s %9s %8s\n', ...
        'V km/h','r rad/s','a_y m/s2','a_y (g)','alpha_f°','alpha_r°','|LTR|');
fprintf('%s\n', repmat('-',1,68));
for Vk = [80 70 60 50 45 40 35 30 25 20 15]
    V = Vk/3.6;
    x = fsolve(@(z) bal(z,V,d,P,TF,TR,Fzf,Fzr), [0;0.3], ...
               optimoptions('fsolve','Display','off','FunctionTolerance',1e-10));
    vy = x(1); r = x(2);  ay = V*r;
    af = atan((vy+P.lf*r)/V) - d;   ar = atan((vy-P.lr*r)/V);
    fprintf('%7.0f %9.4f %9.3f %9.4f %9.3f %9.3f %8.4f\n', ...
            Vk, r, ay, ay/g, rad2deg(af), rad2deg(ar), abs(kLTR*ay));
end
fprintf('\n附着上限 a_y = mu_eff*g = %.3f m/s^2 (%.3f g) -> |LTR| = %.4f\n', ...
        muE*g, muE, kLTR*muE*g);
fprintf('LTR=0.8 需要 a_y <= %.3f m/s^2 (%.3f g)\n', 0.8/kLTR, 0.8/kLTR/g);
end
function F = bal(z,V,d,P,TF,TR,Fzf,Fzr)
vy=z(1); r=z(2);
af = atan((vy+P.lf*r)/V) - d;
ar = atan((vy-P.lr*r)/V);
Fyf = func_TireTable('eval', TF, af, Fzf);
Fyr = func_TireTable('eval', TR, ar, Fzr);
F = [ P.m*V*r - (Fyf*cos(d) + Fyr);
      P.lf*Fyf*cos(d) - P.lr*Fyr ];
end
