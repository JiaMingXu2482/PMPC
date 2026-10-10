function T = Atuning_pmpc()
%ATUNING_PMPC  Editable tuning defaults for priority PMPC.
T.controller = 'PMPC';
T.Ts = 0.1;
T.Np = 10;
T.Nc = 10;

T.Q1 = 0; T.Q2 =    0; T.Q3 = 20;
T.Q4 = 0; T.Q5 =  200; T.Q6 = 80;
T.R1 = 0.2; T.R2 = 0.5; T.R3 = 0.2;
T.S1 = 0; T.S2 = 0; T.S3 = 0;
T.Qf_scale = 1;
T.V1 = 0; T.V2 = 0; T.V3 = 0;
T.tau_gamma = 0;
T.Wdg = [0 0 0];
T.W1 = 1e3; T.W2 = 1e3; T.W3 = 5e3; T.W4 = 5e3;

% Priority rule: L1 road/rollover, L2 stability, L3 tracking/actuators.
T.prio_vartheta = 3;
T.prio_wmin = 1;
T.prio_wmax = [1e5; 1e6];

end
