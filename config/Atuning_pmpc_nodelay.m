function T = Atuning_pmpc_nodelay()
%ATUNING_PMPC_NODELAY  Independent tuning for the legacy six-state variant.
% Shared weights match PMPC; only damper-delay prediction is ablated.
T.controller = 'PMPC-noDelay';
T.Ts = 0.1;
T.Np = 10;
T.Nc = 5;

T.Q1 = 0; T.Q2 = 0; T.Q3 = 1e2;
T.Q4 = 0; T.Q5 = 2e4; T.Q6 = 2e2;
T.R1 = 1; T.R2 = 10; T.R3 = 5;
T.S1 = 0; T.S2 = 0; T.S3 = 0;
T.Qf_scale = 1;
T.V1 = 100; T.V2 = 8e4; T.V3 = 1;
T.tau_gamma = 0;
T.Wdg = [0 0 0];
T.W1 = 5e4; T.W2 = 5e4; T.W3 = 5e6; T.W4 = 5e6;

T.prio_vartheta = 2;
T.prio_wmin = 1;
T.prio_wmax = [1e10; 1e8];

end
