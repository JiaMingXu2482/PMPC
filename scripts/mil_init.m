function mil_init(mode)
%MIL_INIT  R2018a 机器上跑 MIL(CarSim + Simulink)的初始化
%
%   mil_init        控制器由 CarSim 数据集名自动决定(推荐)
%   mil_init(1)     强制本文 PMPC
%   mil_init(2)     强制 baseline MPC
%
%   步骤: CarSim 里选数据集 -> Send to Simulink -> 回 MATLAB 跑 mil_init -> sim
%
%   与 HIL 包的 hil_init 的区别: MIL 保留每拍打印(方便看跟踪误差和 QP 成败),
%   控制器也默认交给数据集名自动判定(那台机器有 CarSim, 读得到 simfile.sim)。
if nargin < 1, mode = 0; end

assignin('base', 'PMPC_VERBOSE', 1);        % MIL 保留每拍打印
if mode > 0
    assignin('base', 'PMPC_MODE',    mode);
    assignin('base', 'PMPC_ZENGRHO', 0);
else
    evalin('base', 'clear PMPC_MODE PMPC_ZENGRHO');
end
evalin('base', 'clear PMPC_P');
evalin('base', 'setup_pmpc;');

P = evalin('base','PMPC_P');
fprintf('\n===== MIL 就绪 =====\n');
if P.Pm.ContrlMode == 1
    fprintf('  控制器    ③ 本文 PMPC (Nr=%d)\n', P.Pm.MPCParameters.Nr);
elseif P.Pm.MPCParameters.Nr == 0 && evalin('base','exist(''PMPC_ZENGRHO'',''var'')') && evalin('base','PMPC_ZENGRHO')
    fprintf('  控制器    ② Zeng rho 调权\n');
else
    fprintf('  控制器    ① 固定权重 MPC / 见上方 setup_pmpc 的打印\n');
end
fprintf('  每拍打印  %s\n', local_onoff(P.Pm.MPCParameters.Verbose));
fprintf('  PMPC_P    Pm %d 字段 / S0 %d 字段\n', ...
        numel(fieldnames(P.Pm)), numel(fieldnames(P.S0)));
fprintf('\n  两个模型:\n');
fprintf('    pmpc_mil   验证 HIL 架构(PMPC_MF SampleTime 0.01 + ode1) <- 上 NI 前跑这个\n');
fprintf('    cq3_2019   原 MIL(Function-Call 触发 + ode3), 论文数据来源\n');
fprintf('  CarSim 的 Models:Simulink 指向哪个就跑哪个。\n');
fprintf('\n  跑 sim(''pmpc_mil'')  -> 核对 chk_regress(''-base'',''baseline_ref_mil'')\n');
fprintf('  跑 sim(''cq3_2019'')  -> 核对 chk_regress(''-base'',''baseline_ref'')\n');
fprintf('  —— 看指标表, 不看 MD5\n');
fprintf('  详见 README_MIL.md\n\n');
end

function s = local_onoff(v)
if v, s = '开'; else, s = '关'; end
end
