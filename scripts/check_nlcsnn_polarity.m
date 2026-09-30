function check_nlcsnn_polarity()
%CHECK_NLCSNN_POLARITY 极性交叉验证: NLCSNN vs 旧查表 func_MRDamper
%
% 背景:
%   部署代码假设 F_CarSim = -F_NLCSNN (CarSim 力"压缩为正",
%   NLCSNN 力"拉伸为正"),取反发生在 4 处:
%     func_NLCSNNContext.m: F0 / Fmax 两处 "-nlcsnn_predict_force(...)"
%     func_NLCSNNApply.m:   "F_request_nl = -F_des(j)" 与 "F_actual(j) = -F_nl"
%   该假设未经台架验证。若假设错误,四个角的阻尼力方向全部反转。
%
% 原理:
%   旧查表 func_MRDamper 是已验证可用的基线(CarSim 极性),
%   与 NLCSNN 来自同一类台架数据。同一 (v, I) 下,两者力的符号应当一致。
%
% 运行(在 PMPC 仓库根目录):
%   >> check_nlcsnn_polarity
% 需要路径: nlcsnn/ (模型文件), controller/ (含 func_MRDamper.m)
%
% 判读:
%   符号一致率 > 90%  -> 假设正确,4 处取反保持不动
%   符号一致率 < 10%  -> 假设反了,去掉 4 处取反(具体位置见文末输出)

%% 0. 路径与参数(与 setup_pmpc.m / func_NLCSNNContext 对齐)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'nlcsnn'));
addpath(fullfile(repoRoot, 'controller'));

x_ref = 281.0645;   % mm, NLCSNN.x_ref
temp  = 42.5;       % degC, NLCSNN.temp
I_list = [0, 1.0];  % A: 查表有 0/1/2/3/4A, NLCSNN 有效 0~1.6A,取交集
V_MIN  = 30;        % mm/s: 只统计 |v| 足够大的点,避开过零噪声
F_MIN  = 10;        % N: 双方力幅值过小的点不计入统计

%% 1. NLCSNN 正弦 rollout,取稳态段的 (v, F)
net = nlcsnn_damper_init();   % 默认加载 nlcsnn/nlcsnn_weights.mat
f0 = 1.66; Amp = 10;          % Hz, mm: 训练常用工况
dt = 0.001; t = (0:dt:4-dt).';
x_nl = x_ref + Amp*sin(2*pi*f0*t);  % NLCSNN 系: x 增大 = 拉伸
v_nl = gradient(x_nl, dt);          % mm/s, 拉伸为正
a_nl = gradient(v_nl, dt);          % mm/s^2

fprintf('=== NLCSNN vs func_MRDamper 极性交叉验证 ===\n');
fprintf('假设: F_CarSim = -F_NLCSNN, v_CarSim(压缩为正) = -v_NLCSNN\n\n');

total_agree = 0; total_n = 0;
fprintf('NLCSNN rollout 中 (2 个电流 x 4000 步,约需几十秒)...\n');
for ii = 1:numel(I_list)
    I = I_list(ii);
    h = zeros(8,1); F_nl = zeros(size(t));
    for n = 1:numel(t)
        % dcurr = 0: 该通道在模型内未被使用(见 build_nfl 注释)
        [F_nl(n), h] = nlcsnn_damper_step(net, x_nl(n), v_nl(n), ...
            a_nl(n), I, 0, temp, dt, h);
    end
    ss = t > 3.0;                 % 取稳态段
    v_n = v_nl(ss); F_n = F_nl(ss);

    % 换算到 CarSim 极性(当前部署假设)
    v_cs = -v_n;                  % mm/s, 压缩为正
    F_cs = -F_n;                  % N, 假设 F_CarSim = -F_NLCSNN

    % 查表: func_MRDamper(v[m/s], I[A]) -> F[N] (CarSim 极性,基线已验证)
    % 注意: 若你处 func_MRDamper 签名/单位不同,改这里
    F_tb = zeros(size(v_cs));
    for k = 1:numel(v_cs)
        F_tb(k) = func_MRDamper(v_cs(k)/1000, I);
    end

    sel = abs(v_cs) > V_MIN & abs(F_cs) > F_MIN & abs(F_tb) > F_MIN;
    agree = sum(sign(F_cs(sel)) == sign(F_tb(sel)));
    nsel = sum(sel);
    total_agree = total_agree + agree; total_n = total_n + nsel;

    rms_cs = sqrt(mean(F_cs(sel).^2));
    rms_tb = sqrt(mean(F_tb(sel).^2));
    fprintf('I = %.1f A: 符号一致 %d/%d (%.1f%%), 幅值比 NLCSNN/查表 = %.2f\n', ...
        I, agree, nsel, 100*agree/max(nsel,1), rms_cs/max(rms_tb,eps));

    % 抽几个速度点,肉眼核对符号
    v_show = [-300 -100 100 300];
    fprintf('   v_cs[mm/s] : '); fprintf('%8d', v_show); fprintf('\n');
    fprintf('   查表 F[N]  : ');
    for vc = v_show, fprintf('%8.0f', func_MRDamper(vc/1000, I)); end
    fprintf('\n');
    fprintf('   NLCSNN F[N]: ');
    for vc = v_show
        [~, im] = min(abs(v_cs - vc));
        fprintf('%8.0f', F_cs(im));
    end
    fprintf('\n\n');
end

%% 2. 结论
frac = total_agree / max(total_n, 1);
fprintf('----------------------------------------\n');
fprintf('总计符号一致率: %.1f%%\n', 100*frac);
if frac > 0.9
    fprintf('结论: 极性假设正确,4 处取反保持不动。\n');
elseif frac < 0.1
    fprintf('结论: 极性反了! 去掉以下 4 处取反:\n');
    fprintf('  func_NLCSNNContext.m:\n');
    fprintf('    F0   = -nlcsnn_predict_force(...)  ->  去掉负号\n');
    fprintf('    Fmax = -nlcsnn_predict_force(...)  ->  去掉负号\n');
    fprintf('  func_NLCSNNApply.m:\n');
    fprintf('    F_request_nl = -F_des(j)  ->  F_request_nl = F_des(j)\n');
    fprintf('    F_actual(j)  = -F_nl       ->  F_actual(j)  = F_nl\n');
else
    fprintf('结论: 结果不一致,先检查 func_MRDamper 签名/单位是否与假设一致。\n');
end
end
