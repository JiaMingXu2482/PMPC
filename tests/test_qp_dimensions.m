function tests = test_qp_dimensions
tests = functiontests(localfunctions);
end

function testUnifiedQpStateCertificateHasFixedSize(testCase)
% The unified QP layout must keep the MATLAB Function state shape fixed.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
P = setup_pmpc();

verifyEqual(testCase, P.Pm.MPCParameters.Ne, 8);
verifyEqual(testCase, P.Pm.MPCParameters.Nr, 3);
verifySize(testCase, P.S0.cert, [36 1]);
verifySize(testCase, P.S0.InitialParams.prevstate.nlcsnn.h, [8 4]);
verifySize(testCase, P.S0.InitialParams.prevstate.nlcsnn.i_prev, [4 1]);
end
