function T = Atuning_zeng()
%ATUNING_ZENG  Editable tuning defaults for ZENG rho weighting.
T.controller = 'ZENG';
T.Ts = 0.1;
T.Np = 10;
T.Nc = 5;

T.Q1 = 0; T.Q2 = 0; T.Q3 = 100;
T.Q4 = 0; T.Q5 = 2e4; T.Q6 = 2e2;
T.R1 = 1; T.R2 = 10; T.R3 = 5;
T.S1 = 0; T.S2 = 0; T.S3 = 0;
T.Qf_scale = 1;
T.V1 = 100; T.V2 = 8e4; T.V3 = 1;
T.tau_gamma = 0;
T.Wdg = [0 0 0];
T.W1 = 5e4; T.W2 = 5e4; T.W3 = 5e6; T.W4 = 5e6;

end
