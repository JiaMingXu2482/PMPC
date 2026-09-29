% TEST_NLCSNN_PORT  Golden-vector test for the MATLAB NLCSNN port.
%
%   Verifies nlcsnn_damper_step against reference outputs generated from the
%   original PyTorch model (see ../ref_nlcsnn.py). Run from this folder:
%       test_nlcsnn_port
%
%   Pass criteria: max abs force error < 1e-6 N, max abs state error < 1e-9.

ORDER = 'predict_then_update';   % must match nlcsnn_damper_step.m

thisDir = fileparts(mfilename('fullpath'));
net = nlcsnn_damper_init(fullfile(thisDir, 'nlcsnn_weights.mat'));
G = load(fullfile(thisDir, 'golden_vectors.mat'));

U = G.U_phys;
dt = G.dt_macro;
N = size(U, 1);

switch ORDER
    case 'predict_then_update'
        F_ref = G.F_predict_then_update;
        h_ref = G.h_final_predict_then_update;
    case 'update_then_predict'
        F_ref = G.F_update_then_predict;
        h_ref = G.h_final_update_then_predict;
    otherwise
        error('unknown ORDER');
end

h = zeros(8, 1);
F = zeros(N, 1);
for k = 1:N
    [F(k), h] = nlcsnn_damper_step(net, U(k,1), U(k,2), U(k,3), ...
        U(k,4), U(k,5), U(k,6), dt, h);
end

errF = max(abs(F - F_ref(:)));
errH = max(abs(h - h_ref(:)));
fprintf('max |F_matlab - F_ref| = %.3e N\n', errF);
fprintf('max |h_matlab - h_ref| = %.3e\n', errH);

tolF = 1e-6; tolH = 1e-9;
if errF < tolF && errH < tolH
    fprintf('PASS: MATLAB port matches the PyTorch reference.\n');
else
    fprintf('FAIL: port deviates from reference.\n');
end
