function T = Atuning_mpc()
%ATUNING_MPC  Editable tuning defaults for fixed-weight MPC.
% Prediction uses Ts; the controller is executed separately at 0.01 s.
T.controller = 'MPC';
T.Ts = 0.1;
T.Np = 10;
T.Nc = 5;
% 状态代价：[Vy, r, phi, dphi, e_y, e_psi]
T.Q1 = 0; T.Q2 = 0; T.Q3 = 100;
T.Q4 = 0; T.Q5 = 2e4; T.Q6 = 2e2;
T.R1 = 1; T.R2 = 10; T.R3 = 5;
T.S1 = 0; T.S2 = 0; T.S3 = 0;
T.Qf_scale = 1;%% 最后一个预测节点的状态权重倍率；1 表示不额外加权
% 以下为统一 QP 结构保留的优先级参数，固定权重 MPC 模式下不参与调权
T.V1 = 100; T.V2 = 8e4; T.V3 = 1;  T.tau_gamma = 0; T.Wdg = [0 0 0];
% 软约束松弛量惩罚：越大，越不允许对应约束被违反
T.W1 = 5e4; T.W2 = 5e4; T.W3 = 5e6; T.W4 = 5e6;
end
