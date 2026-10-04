function tests = test_pmpc_safety_witness
tests = functiontests(localfunctions);
end

function setupOnce(~)
addpath(fileparts(fileparts(mfilename('fullpath'))));
startup_pmpc();
end

function testWorstOfRoadAndLTRRows(testCase)
[p,A,b,x,du,pc] = fixture();
b(51) = 0.10; % first LTR row
b(61) = 0.04; % first road corner row
A(61,16) = -1;
x(16) = 0.50; % positive s1 cannot conceal the physical violation
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyEqual(testCase,d.m_plan,0.04,'AbsTol',1e-12);
verifyTrue(testCase,d.plan_valid);
verifyEqual(testCase,d.m_witness,0.04,'AbsTol',1e-12);
end

function testCandidateIsFeasibleWitness(testCase)
[p,A,b,x,du,pc] = fixture();
A(61,1) = 0.2;
b(61) = 0.04;
x(1) = 0.5; % planned road margin -0.06
pc(1) = 0.04;
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyEqual(testCase,d.m_plan,-0.06,'AbsTol',1e-12);
verifyEqual(testCase,d.m_candidate,0.04,'AbsTol',1e-12);
verifyEqual(testCase,d.m_witness,0.04,'AbsTol',1e-12);
verifyTrue(testCase,d.candidate_valid);
end

function testNegativeDoesNotClaimGlobalInfeasibility(testCase)
[p,A,b,x,du,pc] = fixture();
b(51) = -0.03;
pc(1) = -0.03;
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyEqual(testCase,d.m_witness,-0.03,'AbsTol',1e-12);
verifyFalse(testCase,d.unknown);
end

function testFailedOrNonfiniteSolveIsUnknown(testCase)
[p,A,b,x,du,pc] = fixture();
d = func_PMPCSafetyWitness(p,A,b,x,0,du,0,pc);
verifyTrue(testCase,d.unknown);
verifyTrue(testCase,isnan(d.m_witness));
x(1) = NaN;
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyTrue(testCase,d.unknown);
verifyTrue(testCase,isnan(d.m_witness));
end

function testInvalidCandidateCannotBecomeWitness(testCase)
[p,A,b,x,du,pc] = fixture();
x(1) = 0.5;
A(61,1) = 0.2; b(61) = 0.04;
pc(14) = 0.1; % projected candidate violates a hard input row
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyFalse(testCase,d.candidate_valid);
verifyEqual(testCase,d.m_witness,-0.06,'AbsTol',1e-12);
end

function testSpeedMarginExcludesRoadRowsBeyondControlHorizon(testCase)
[p,A,b,x,du,pc] = fixture();
b(81) = -0.5; % sixth road node; Nc=5 cannot adjust a new input there
pc(1) = -0.5;
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyEqual(testCase,d.m_witness,-0.5,'AbsTol',1e-12);
verifyEqual(testCase,d.m_speed,0.2,'AbsTol',1e-12);
end

function testSpeedMarginUsesRollRatherThanTrackingViolation(testCase)
[p,A,b,x,du,pc] = fixture();
b(51) = -0.08; % first roll row
b(61) = -0.12; % first road corner must remain a QP, not speed, concern
pc(1) = -0.12;
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyEqual(testCase,d.m_speed,-0.08,'AbsTol',1e-12);
end

function testRoadViolationAloneDoesNotRequestLongitudinalSpeedCut(testCase)
[p,A,b,x,du,pc] = fixture();
b(61) = -0.4;
pc(1) = -0.4;
d = func_PMPCSafetyWitness(p,A,b,x,1,du,0,pc);
verifyEqual(testCase,d.m_witness,-0.4,'AbsTol',1e-12);
verifyEqual(testCase,d.m_speed,0.2,'AbsTol',1e-12);
end

function [p,A,b,x,du,pc] = fixture()
p = struct('Nu',3,'Nc',5,'Ne',8,'Nr',3,'Ncons_sh',5, ...
    'Ncons_r',5,'Ncons_env',10,'Np',10);
A = zeros(100,26);
b = 0.2*ones(100,1);
x = zeros(26,1);
du = zeros(15,1);
pc = zeros(14,1);
pc(1) = 0.2;
end
