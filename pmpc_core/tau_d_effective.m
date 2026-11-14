function tau_d = tau_d_effective(M,vd)
%TAU_D_EFFECTIVE 本拍真正进入预测矩阵的时滞常数 [s]。
%
%   预测模型里"阻尼力矩命令 -> 实际力矩"是一阶环节，时间常数就是这里的返回值：
%     tau_d -> 0     认为阻尼力瞬时建立（不建模时延，无补偿）
%     tau_d >> 0     预测模型知道力要慢慢建立，控制器会提前给力（补偿）
%
%   M.tau_d_mode = 1（默认）：常数，取 M.tau_d。各工况在自己 case 里给值：
%     DLC 用全局默认 0.01；Slalom 0.20；FishHook noComp 1e-4、comp25 0.025。
%   M.tau_d_mode = 2（comp 口径）：时变，忽略 M.tau_d，按四角压缩速度幅值
%     vmag = 1000*mean(abs(vd)) [mm/s] 查表，标定来自减振器力阶跃响应的 t10-90
%     （50/100/200/400 mm/s -> 17/19/22/22 ms），分段线性、末段饱和。
%
%   两种情况最后都夹一个 1e-4 下限，避免 1/tau_d 发散。
%   数值 0.017/0.019/0.022 是标定结果，不要为了让 mode1/mode2 接近而改。
%
%   单独抽成函数是为了让“仿真内部真正用的值”和“事后记录下来的值”用同一份口径，
%   不会各算一套：
%     - 仿真内部：只有 build_prediction 调它，值进 A(7,7) / B(7,3) 的 ZOH；
%     - 事后记录：run_*.m 用同一函数、按本拍实测 vd（CarSim 的 CmpRD_*）
%       逐拍复算，写进 .mat 的 tau_dtrace。
%   这个值不进 logVec：pmpc_step 的 logVec 是 Stateflow 数据对象，尺寸锁死
%   [27 1]，加不了第 28 列（试过，编译报错）。所以 .mat 里的 tau_dtrace 是
%   事后复算量，不是仿真内部逐拍回显量——它能回答“这次用的是哪个 tau_d”，
%   不回答“这个 tau_d 对不对”。

if isfield(M,'tau_d_mode') && M.tau_d_mode==2
    vmag=1000*mean(abs(vd));              % [mm/s]
    if vmag<=50
        tau_d=0.017;
    elseif vmag<=100
        tau_d=0.017+0.002*(vmag-50)/50;
    elseif vmag<=200
        tau_d=0.019+0.003*(vmag-100)/100;
    else
        tau_d=0.022;
    end
else
    tau_d=M.tau_d;
end
tau_d=max(tau_d,1e-4);
end
