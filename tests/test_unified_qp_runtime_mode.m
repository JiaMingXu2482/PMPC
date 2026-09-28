function tests = test_unified_qp_runtime_mode
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
end

function testUnifiedDimensionsAcrossModes(testCase)
[P_mpc, c1]  = local_setup_mode(2, 0);
[P_zeng, c2] = local_setup_mode(2, 1);
[P_pmpc, c3] = local_setup_mode(1, 0);
cleanup = onCleanup(@() local_restore(c1, c2, c3)); %#ok<NASGU>

verifyEqual(testCase, P_mpc.Pm.MPCParameters.Ne, 8);
verifyEqual(testCase, P_zeng.Pm.MPCParameters.Ne, 8);
verifyEqual(testCase, P_pmpc.Pm.MPCParameters.Ne, 8);
verifyEqual(testCase, P_mpc.Pm.MPCParameters.Nr, 3);
verifyEqual(testCase, P_zeng.Pm.MPCParameters.Nr, 3);
verifyEqual(testCase, P_pmpc.Pm.MPCParameters.Nr, 3);

s1 = local_qp_shape(P_mpc);
s2 = local_qp_shape(P_zeng);
s3 = local_qp_shape(P_pmpc);
verifyEqual(testCase, s1, s2);
verifyEqual(testCase, s1, s3);

verifyEqual(testCase, P_mpc.S0.Constraints.ControllerMode, 2);
verifyEqual(testCase, P_zeng.S0.Constraints.ControllerMode, 2);
verifyEqual(testCase, P_pmpc.S0.Constraints.ControllerMode, 1);
verifyEqual(testCase, P_mpc.S0.Constraints.ZengRho_on, 0);
verifyEqual(testCase, P_zeng.S0.Constraints.ZengRho_on, 1);
verifyEqual(testCase, P_pmpc.S0.Constraints.PrioModeRT, 1);
end

function [P, backup] = local_setup_mode(mode, zeng)
backup = local_backup({'PMPC_MODE','PMPC_ZENGRHO'});
assignin('base', 'PMPC_MODE', mode);
assignin('base', 'PMPC_ZENGRHO', zeng);
P = setup_pmpc();
end

function s = local_qp_shape(P)
M = P.Pm.MPCParameters;
C = P.S0.Constraints;
Nu = M.Nu; Nc = M.Nc; Np = M.Np; Nx = M.Nx; Ny = M.Ny; Ne = M.Ne; Nr = M.Nr;

AI = kron(tril(ones(Nc)), eye(Nu));
Ut = zeros(Nu*Nc,1);
zeta = zeros(Nx+Nu,1);
Pred = struct('PSI', zeros(Np*Ny, Nx+Nu), ...
              'THETA', zeros(Np*Ny, Nu*Nc), ...
              'PHI', zeros(Np*Ny, 1), ...
              'GAMMA', 0);
Envelope = struct('Hsh', zeros(4,Ny), 'Gsh', ones(4,1), ...
                  'Henv', zeros(4,Ny), 'Genv', ones(4,1), ...
                  'Hr', zeros(2,Ny), 'Gr', ones(2,1), ...
                  'Or', zeros(2,Nu), 'Eenv', [zeros(4,4), eye(4)], ...
                  'gkap', zeros(4,1), ...
                  'sc_sh', ones(4,1), 'sc_env', ones(4,1), 'sc_r', ones(2,1));
Lim = struct('kap', zeros(Np,1), ...
             'Fyfmax', 1, 'MFxmax', 1, 'Mdmax', 1, 'Mdmin', -1, 'Mdnom', 0, ...
             'eps_ub', inf(Ne,1), ...
             'sigma_budget', 0, 'gamma_prev', [0;0;1], ...
             'kappa_fx', 0, 'u_op', 0, 'fx_couple', 0, ...
             'dFyfmax', 1, 'dMFxmax', 1, 'dMdmax', 1, ...
             'dlt_ub', 1, 'dlt_lb', -1);
Wts = struct('Q', eye(Np*Ny), 'R', eye(Nu*Nc), 'S', eye(Nu*Nc), ...
             'W', eye(Ne), 'V', eye(Nr));

[A_cons, b_cons, lb, ub] = func_BuildQPConstraints(M, C, Envelope, Pred, AI, Ut, zeta, Lim);
[H, f] = func_BuildQPCost(M, C, Pred, Wts, zeta, AI, Ut, zeros(Np,1), zeros(Np,1), 1, zeros(Nr,1), zeros(Nr));

s = [size(H,1), size(H,2), numel(f), size(A_cons,1), size(A_cons,2), numel(b_cons), numel(lb), numel(ub)];
end

function backup = local_backup(names)
backup = cell(numel(names),3);
for i = 1:numel(names)
    backup{i,1} = names{i};
    backup{i,2} = evalin('base', sprintf('exist(''%s'',''var'')', names{i})) == 1;
    if backup{i,2}
        backup{i,3} = evalin('base', names{i});
    else
        backup{i,3} = [];
    end
end
end

function local_restore(varargin)
for k = 1:nargin
    bk = varargin{k};
    for i = 1:size(bk,1)
        n = bk{i,1};
        if bk{i,2}
            assignin('base', n, bk{i,3});
        else
            evalin('base', ['clear ' n]);
        end
    end
end
end
