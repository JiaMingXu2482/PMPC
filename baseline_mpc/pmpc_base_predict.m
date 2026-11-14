function p = pmpc_base_predict(c, P, plan)
%PMPC_BASE_PREDICT 基线 MPC 的 6 状态线性化 + ZOH 预测矩阵。
%#codegen
% x = [Vy; r; phi; p; ey; epsi]（6 维，含侧倾角与侧倾角速度）
% u = [delta; Mz]（2 维，不含减振器力 Md）
% w = [道路曲率; 前轴轮胎偏置; 后轴轮胎偏置]
%
% 输出：
%   p.Y   6N x 2K   控制增量到预测状态的映射
%   p.y   6N x 1    零增量自由响应
%   p.AI  2K x 2K   累计增量到绝对输入的映射（下三角块）
%   p.Ut  2K x 1    上一拍输入复用到各节点
%
% 与 pmpc_core/build_prediction 的差别只有两处：删掉 Md 一阶响应状态
% 及其 B 列（控制量从 3 维降到 2 维）；时滞 tau_d 不建模（减振器固定电流）。
% 侧倾自由度与轮胎线性化完全一致，因此两套控制器面对同一个被控对象。

N = coder.const(size(c.kap,1));
K = coder.const(numel(plan)/2);
V = P.vehicle;
v = c.v;
cd = cos(c.delta);
Cf = c.Cf;
Cr = c.Cr;

% 连续六状态：横向、横摆、侧倾角/角速度、横向/航向跟踪误差
A = [ (Cf*cd + Cr)/(V.m*v),                (Cf*V.lf*cd - Cr*V.lr)/(V.m*v) - v, 0, 0, 0, 0; ...
      (V.lf*Cf*cd - V.lr*Cr)/(V.Iz*v),     (V.lf^2*Cf*cd + V.lr^2*Cr)/(V.Iz*v), 0, 0, 0, 0; ...
       0, 0, 0, 1, 0, 0; ...
       V.ms*V.h_S2R*(Cf*cd + Cr)/(V.m*V.Ix*v), ...
       V.ms*V.h_S2R*(Cf*V.lf*cd - Cr*V.lr)/(V.m*V.Ix*v), ...
       (V.ms*V.g*V.h_S2R - V.Kt)/V.Ix, 0, 0, 0; ...
       1, 0, 0, 0, 0, v; ...
       0, 1, 0, 0, 0, 0];

B = [ -Cf*cd/V.m,                    0; ...
      -V.lf*Cf*cd/V.Iz,              1/V.Iz; ...
       0,                            0; ...
      -V.ms*V.h_S2R*Cf*cd/(V.m*V.Ix), 0; ...
       0,                            0; ...
       0,                            0];

D = [ 0,  cd/V.m,              1/V.m; ...
      0,  V.lf*cd/V.Iz,       -V.lr/V.Iz; ...
      0,  0,                   0; ...
      0,  V.ms*V.h_S2R*cd/(V.m*V.Ix), V.ms*V.h_S2R/(V.m*V.Ix); ...
      0,  0,                   0; ...
     -v,  0,                   0];

% 输入与扰动在预测步内零阶保持
E = expm([A, B, D; zeros(5,11)]*P.M.Ts);   % 6 状态 + 2 控制 + 3 扰动 = 11

% 增广上一拍输入
Aa = [E(1:6,1:6), E(1:6,7:8); zeros(2,6), eye(2)];
Ba = [E(1:6,7:8); eye(2)];
Da = [E(1:6,9:11); zeros(2,3)];

X = zeros(8, 2*K);
x = c.zeta;
Y = zeros(6*N, 2*K);
y = zeros(6*N, 1);
for j = 1:N
    x = Aa*x + Da*[c.kap(j); c.offset];
    X = Aa*X;
    if j <= K
        X(:, (j-1)*2+(1:2)) = X(:, (j-1)*2+(1:2)) + Ba;
    end
    y((j-1)*6+(1:6))         = x(1:6);
    Y((j-1)*6+(1:6), :)      = X(1:6, :);
end

p = struct('Y', Y, 'y', y, ...
           'AI', kron(tril(ones(K)), eye(2)), ...
           'Ut', repmat(c.zeta(7:8), K, 1));
end
