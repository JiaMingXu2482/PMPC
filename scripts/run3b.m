function tags = run3b(outdir, varargin)
%RUN3B  依次跑三个控制器 (MPC / ZENG / PMPC), 把 ERD 复制到 outdir
%   run3b()          -> 写到 func_ErdDir()  = <Simulink_Model>\erd_out
%   run3b(dir)       -> 写到指定目录
%   run3b(dir,'-force') -> 展开结果过期也照跑(透传给 run_ds)
%   tags = run3b(..) -> 返回 {'MPC','ZENG','PMPC'}
%
%   控制器由**数据集自己决定**(2026-09-18 起): 每个控制器有自己的 CarSim
%   数据集 DLC80_mu0.5_<TAG>, run_ds 切 simfile.sim 指向, func_RunMode 从
%   数据集名认出模式。这样不会出现"用 PMPC 数据集跑出 baseline"。
%
%     DLC80_mu0.5_MPC   -> ① 固定权重 MPC (无 sigma/gamma)
%     DLC80_mu0.5_ZENG  -> ② Zeng rho 调权 + 纵向限速
%     DLC80_mu0.5_PMPC  -> ③ 本文 PMPC
%
%   旧做法是在同一个数据集上用工作区的 PMPC_MODE/PMPC_ZENGRHO 强切,
%   run_ds 会先清掉这两个变量(它们优先级高于数据集, 残留就会盖掉自动识别)。
%
%   前置条件:
%     - 仿真时 CarSim 必须开着(求解器要取许可)
%     - 三个数据集都必须 Send to Simulink 过, 且在那之后没再改过;
%       改过就要重新 Send, run_ds 会检查时间戳并报错
if nargin < 1 || isempty(outdir), outdir = func_ErdDir(); end
if ~exist(outdir,'dir'), mkdir(outdir); end

tags = {'MPC','ZENG','PMPC'};
for k = 1:numel(tags)
    info = run_ds(tags{k}, varargin{:});
    copyfile(fullfile(info.resdir,'LastRun.vsb'), fullfile(outdir,[tags{k} '.vsb']));
    copyfile(fullfile(info.resdir,'LastRun.vs'),  fullfile(outdir,[tags{k} '.vs']));
end
fprintf('  ERD 已写入 %s\n', outdir);
end
