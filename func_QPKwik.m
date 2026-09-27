function [x, status, nAct, lam] = func_QPKwik(H, f, A, b, lb, ub, maxiter, usews)
%   lam (可选, 2026-09-25): 不等式乘子, 顺序同 Am = [-A; I; -I], 即前 size(A,1) 个
%   对应 A*x <= b, 其后 n 个对应 x >= lb, 再 n 个对应 x <= ub。均 >= 0。
%   用于论文 III-E 的在线证书 (func_SolveMPCQP)。
%FUNC_QPKWIK  用 MPC Toolbox 的 KWIK active-set 求解器解 QP
%   min 0.5*x'Hx + f'x   s.t.  A*x <= b ,  lb <= x <= ub
%
%   为什么不用 quadprog: R2018a 的 quadprog **不支持代码生成**
%   (codegen 支持是 R2020a 才加的, 见 PLAN_HIL迁移方案.md §1.1)。
%   mpcqpsolver 自 R2015b 起就支持 codegen, 是 HIL 上唯一的官方路径。
%
%   本函数负责吸收三处接口差异, 让调用方仍按 quadprog 的习惯传参:
%     1. mpcqpsolver 要 Linv = inv(chol(H,'lower')), 不是 H
%     2. mpcqpsolver 的不等式约定是 A*x >= b, 与 quadprog 相反
%     3. mpcqpsolver 没有独立的 lb/ub, 必须折进不等式
%
%   输入 H 必须**正定**(KWIK 的硬前提)。实测本项目 Jacobi 预缩放后
%   cond(H)=4833、最小特征值 4.16e-06 > 0, 满足。
%
%   iA0/iA 是逻辑型活动集(长度 = size(A,1)+2*n), 用于热启动。
%   实测: 冷启动 108 次迭代, 用上一拍活动集热启动只要 1 次。
%
%   status: >0 最优(值 = 迭代次数); 0 达到最大迭代(解可能次优甚至不可行);
%           -1 不可行; -2 数值错误。调用方必须按 status<=0 走失败回退。

n = numel(f);
BIG = 1e12;                      % 代替 Inf: mpcqpsolver 不接受无穷界,
ubf = ub; lbf = lb;              % 且 codegen 要求行数固定, 不能删行
ubf(~isfinite(ubf)) =  BIG;
lbf(~isfinite(lbf)) = -BIG;

% A*x <= b          ->  -A*x >= -b
% x  <= ub          ->  -x   >= -ub
% x  >= lb          ->   x   >=  lb
Am = [ -A ; eye(n) ; -eye(n) ];
bm = [ -b ; lbf    ; -ubf    ];
m  = numel(bm);

%  活动集热启动用 persistent 而非出入参: 它的长度 m = size(A,1)+2n
%  在一次 build 内是常量, 但初值没法在 func_InitialParams 里算出来
%  (依赖 A_cons 的行数)。persistent 的尺寸由首次赋值确定, 正好合适。
%  usews: 是否用活动集热启动。**三个调用点不能共用同一个 persistent**,
%  所以只有上层 MPC (迭代多、收益大) 用热启动; 两个分配 QP 冷启动
%  (实测只要 1~3 次迭代, 热启动无意义)。
if nargin < 8, usews = true; end
%  ⚠️ persistent 必须放进**只有 usews=true 才会实例化**的局部函数里, 不能写在这儿。
%  写在主函数里的话, 三个调用点会共用同一个 persistent 的类型与尺寸:
%    上层 MPC QP  m = size(A_cons,1)+2n = 154
%    两个分配 QP  m = 0 + 2*4          = 8
%  codegen 直接报 "大小不匹配(大小 [154 x 1] ~= 大小 [8 x 1])"。
%  (以前 A_cons 行数是变尺寸的, 两边都退化成 [:? x 1] 才侥幸糊过去;
%   把 sigma_budget/fx_couple 挪进 MPCParameters 让行数固定后, 冲突就显形了。)
%  usews 在每个调用点都是编译期常量, 所以下面的 if 会被折掉,
%  分配 QP 的 specialization 根本不会实例化 local_warmstart。
if usews
    iA0 = local_warmstart(m, false(m,1), false);
else
    iA0 = false(m,1);
end

[L, p] = chol(H, 'lower');
if p ~= 0
    x = zeros(n,1);  status = -2;  nAct = 0;  lam = zeros(m,1);  return
end
Linv = L \ eye(n);

opt = mpcqpsolverOptions;
opt.MaxIter         = maxiter;
opt.FeasibilityTol  = 1e-6;
opt.IntegrityChecks = false;     % 上游已做有限性检查, 关掉省时间

[x, st_, iA_out, lambda_] = mpcqpsolver(Linv, f, Am, bm, zeros(0,n), zeros(0,1), iA0, opt);
lam = lambda_.ineqlin;          % 只读, 不影响 x (KWIK 本来就算出了乘子)
status = double(st_);   % codegen: mpcqpsolver 返回 int32, 而早退分支给的是 double
if usews, local_warmstart(m, iA_out, true); end    % 存给下一拍热启动
nAct = double(sum(iA_out));
end

% -------------------------------------------------------------------------
function iA = local_warmstart(m, iA_new, doSet)
%LOCAL_WARMSTART  活动集热启动的存取; persistent 只在这里, 只被上层 QP 实例化
persistent iA_ws
if isempty(iA_ws)                % codegen: 赋值前只允许 isempty 检查
    iA_ws = false(m,1);
end
if numel(iA_ws) ~= m
    iA_ws = false(m,1);
end
if doSet
    iA_ws = iA_new;
end
iA = iA_ws;
end
