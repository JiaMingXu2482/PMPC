function tests = test_nlcsnn_plant_smoothness
%TEST_NLCSNN_PLANT_SMOOTHNESS Guard against reintroducing a 10 ms force hold.
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testForwardForceChangesAtOneKhzUnderFixedCurrent(testCase)
P = setup_pmpc();
cfg = P.Pm.NLCSNN;
net = cfg.net;
N = 400;
t = (0:N-1)' * cfg.dt_plant;
cmpD = 10*sin(2*pi*5*t);
cmpRD = 10*2*pi*5*cos(2*pi*5*t);

h = zeros(8,4);
a_filt = zeros(4,1);
v_prev = zeros(4,1);
initialized = 0;
force = zeros(N,1);
for k = 1:N
    [Fk, h, a_filt, v_prev, initialized] = func_NLCSNNPlant4( ...
        net, repmat(cmpD(k),4,1), repmat(cmpRD(k),4,1), ...
        0.8*ones(4,1), h, a_filt, v_prev, initialized, cfg);
    force(k) = Fk(1);
end

step = abs(diff(force(101:end)));
verifyGreaterThan(testCase, nnz(step > 1e-4), 0.90*numel(step));
verifyLessThan(testCase, max(step), 500);
end
