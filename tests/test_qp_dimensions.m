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
% Multirate (2026-09-29): the controller keeps only i_prev; h / v_prev /
% a_filt / initialized moved to the 1 kHz plant Unit Delays.
verifySize(testCase, P.S0.InitialParams.prevstate.nlcsnn.i_prev, [4 1]);
verifyFalse(testCase, isfield(P.S0.InitialParams.prevstate.nlcsnn, 'h'));
end
