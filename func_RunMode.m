function [mode, zeng, dsname] = func_RunMode()
%FUNC_RUNMODE  从当前 CarSim Run 数据集名自动识别控制器模式
% -------------------------------------------------------------------------
%  三个对比控制器共用同一个 Simulink 模型, 仅靠 ContrlMode / ZengRho_on 区分。
%  靠手工设工作区变量容易跑错配置且不自知, 故改为从数据集名自动识别:
%
%    名字含 ZENG  -> mode=2, zeng=1   先进对比: Zeng 2025 约束 + rho 调权
%    名字含 PMPC  -> mode=1, zeng=0   本文方法: 优先级变量 sigma/gamma
%    其余         -> mode=2, zeng=0   baseline: 固定权重 MPC (无 sigma/gamma)
%
%  匹配顺序: ZENG -> PMPC -> 其余 (因 "PMPC" 本身含 "MPC")
%  工作区变量 PMPC_MODE / PMPC_ZENGRHO 若存在仍可覆盖本函数的判定。
% -------------------------------------------------------------------------
mode = 2;  zeng = 0;  dsname = '<unknown>';
try
    par = fullfile(func_CarSimResDir(), 'Run_all.par');
    ds  = regexp(fileread(par), 'FullDataName CarSim Run Control`([^`]+)`','tokens','once');
    dsname = ds{1};
catch
    return;                      % 读不到就按 baseline, 不让它中断仿真
end
U = upper(dsname);
if     contains(U,'ZENG'), mode = 2;  zeng = 1;
elseif contains(U,'PMPC'), mode = 1;  zeng = 0;
else,                      mode = 2;  zeng = 0;
end
end
