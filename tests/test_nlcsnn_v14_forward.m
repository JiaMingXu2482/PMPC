function tests = test_nlcsnn_v14_forward
% Compare each streaming step with independent PyTorch v14 reference traces.
tests = functiontests(localfunctions);
end

function testFourIndependentGoldenHistories(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'nlcsnn'));
G = load(fullfile(root, 'nlcsnn', 'v14_golden_vectors.mat'));
net = nlcsnn_damper_init(fullfile(root,'nlcsnn','v14_weights.mat'));
verifyTrue(testCase, isfield(net, 'state_length'));
if ~isfield(net, 'state_length'), return; end
force = zeros(size(G.force_norm), 'single');
state = zeros(size(G.gru_state), 'single');
effective = zeros(size(G.i_effective), 'single');
for corner = 1:4
    h = zeros(double(net.state_length), 1, 'single');
    for k = 1:size(G.u_norm, 2)
        [force(k, corner), h, effective(k, corner)] = ...
            nlcsnn_v14_step(net, G.u_norm(:, k, corner), h);
        state(:, k, corner) = h(1:128);
    end
end
verifyLessThan(testCase, max(abs(force(:) - G.force_norm(:))), 2e-4);
verifyLessThan(testCase, max(abs(state(:) - G.gru_state(:))), 3e-5);
verifyLessThan(testCase, max(abs(effective(:) - G.i_effective(:))), 3e-6);
verifyGreaterThan(testCase, max(abs(force(:, 1))), 1e-4); % v=0 bias
verifyGreaterThan(testCase, abs(effective(99, 3) - effective(150, 3)), 0.1);
end

function testPublicPhysicalInterfaceUsesSameStepForce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'nlcsnn'));
G = load(fullfile(root, 'nlcsnn', 'v14_golden_vectors.mat'));
net = nlcsnn_damper_init(fullfile(root,'nlcsnn','v14_weights.mat'));
verifyTrue(testCase, isfield(net, 'norm_cfg'));
if ~isfield(net, 'norm_cfg'), return; end
n = net.norm_cfg;
h = zeros(double(net.state_length), 1, 'single');
for k = 1:6
    u = G.u_norm(:, k, 2);
    [F, h] = nlcsnn_damper_step(net, ...
        u(1)*n.x_scale+n.x_ref, u(2)*n.v_scale, u(3)*n.a_scale, ...
        u(4)*n.i_scale, u(5)*n.di_scale, ...
        u(6)*n.t_scale+n.t_ref, 0.001, h);
    verifyEqual(testCase, F, double(G.force_norm(k, 2))*n.f_scale, ...
        'AbsTol', 1);
end
end

function testLegacyDefaultStaysActiveUntilModelWiring(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'nlcsnn'));
net = nlcsnn_damper_init();
verifyTrue(testCase, isfield(net,'Wx1'));
verifyFalse(testCase, isfield(net,'weights'));
end
