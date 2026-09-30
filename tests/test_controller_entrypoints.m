function tests = test_controller_entrypoints
tests = functiontests(localfunctions);
end

function testEachEntryPointAcceptsOnlyItsOwnVariantBundle(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();

cases = {1, @mpc_block,  'mpc_block'; ...
         2, @zeng_block, 'zeng_block'; ...
         3, @pmpc_block, 'pmpc_block'};
u = zeros(54,1); hp = zeros(8,4); xp = zeros(4,1); vp = zeros(4,1); ap = zeros(4,1);

for k = 1:size(cases,1)
    assignin('base','PMPC_CONTROLLER_VARIANT',cases{k,1});
    assignin('base','PMPC_VERBOSE',0);
    P = setup_pmpc();
    eval(['clear ' cases{k,3}]); %#ok<EVLC>
    [sys, iCmd] = cases{k,2}(u, hp, xp, vp, ap, P);
    verifySize(testCase, sys, [54 1]);
    verifySize(testCase, iCmd, [4 1]);
end

evalin('base','clear PMPC_CONTROLLER_VARIANT PMPC_VERBOSE');
end

function testEntryPointRejectsForeignBundle(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
startup_pmpc();
assignin('base','PMPC_CONTROLLER_VARIANT',2);
assignin('base','PMPC_VERBOSE',0);
P = setup_pmpc();
u = zeros(54,1); hp = zeros(8,4); xv = zeros(4,1);

clear mpc_block
verifyError(testCase, @() mpc_block(u,hp,xv,xv,xv,P), 'mpc_block:WrongVariant');
evalin('base','clear PMPC_CONTROLLER_VARIANT PMPC_VERBOSE');
end
