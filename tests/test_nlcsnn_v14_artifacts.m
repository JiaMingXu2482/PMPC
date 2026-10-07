function tests = test_nlcsnn_v14_artifacts
% Verify that the MATLAB export is tied to the supplied v14 checkpoint.
tests = functiontests(localfunctions);
end

function testExportedCheckpointAndShapes(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
weightsFile = fullfile(root, 'nlcsnn', 'v14_weights.mat');
verifyTrue(testCase, isfile(weightsFile));
if ~isfile(weightsFile), return; end
S = load(weightsFile);
verifyEqual(testCase, char(S.checkpoint_md5), ...
    '8a3bd0d508429ab2c00083321561561a');
verifyEqual(testCase, size(S.weights.gru_weight_ih_l0), [192 35]);
verifyEqual(testCase, size(S.weights.gru_weight_hh_l1), [192 64]);
verifyEqual(testCase, size(S.weights.gru_bias_ih_l0), [192 1]);
verifyEqual(testCase, double(S.model_cfg.gru_hidden), 64);
verifyEqual(testCase, double(S.model_cfg.gru_layers), 2);
verifyEqual(testCase, double(S.norm_cfg.i_scale), 1.6, 'AbsTol', 1e-8);
verifyGreaterThan(testCase, double(S.current_lag_length), 1);
verifyEqual(testCase, double(S.state_length), ...
    128 + double(S.current_lag_length) - 1);
verifyTrue(testCase, isfield(S, 'current_lag_kernel'));
if ~isfield(S, 'current_lag_kernel'), return; end
verifyEqual(testCase, numel(S.current_lag_kernel), double(S.current_lag_length));
verifyLessThan(testCase, S.current_lag_kernel(1), S.current_lag_kernel(end));
end

function testGoldenSequencesIncludeZeroSpeedBias(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
goldenFile = fullfile(root, 'nlcsnn', 'v14_golden_vectors.mat');
verifyTrue(testCase, isfile(goldenFile));
if ~isfile(goldenFile), return; end
G = load(goldenFile);
verifyEqual(testCase, size(G.u_norm, 1), 6);
verifyEqual(testCase, size(G.u_norm, 3), 4);
verifyEqual(testCase, size(G.gru_state, 1), 128);
verifyEqual(testCase, size(G.force_norm, 2), 4);
verifyTrue(testCase, all(isfinite(G.force_norm(:))));
verifyTrue(testCase, all(isfinite(G.i_effective(:))));
zeroSpeed = abs(squeeze(G.u_norm(2, :, :))) < 1e-8;
verifyGreaterThan(testCase, max(abs(G.force_norm(zeroSpeed))), 1e-4);
end
