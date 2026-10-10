function tests=test_lex_priority
tests=functiontests(localfunctions);
end
function setupOnce(~)
startup_pmpc();
end
function testConflictingObjectives(t)
% x=[u;s1;s2;gamma]. L1 asks u>=.8, L2 asks u<=.2.
A=[-1 -1 0 0;1 0 -1 0]; b=[-.8;.2];
H=eye(4); f=[100;1e10;1e9;1];
[x,st]=func_QPLexPriority(H,f,A,b,[-1;0;0;0],[1;inf;inf;1],1,[1;1],1e-5);
verifyGreaterThan(t,st,0);
verifyLessThanOrEqual(t,x(2),3e-5);
verifyLessThan(t,abs(x(3)-.6),5e-5);
verifyLessThan(t,abs(x(1)-.8),5e-5);
end
function testSafetyDominatesLargeTrackingCost(t)
A=[-1 -1 0 0;1 0 -1 0]; b=[-.5;.75];
H=eye(4); f=[1e5;1e10;1e9;1];
[x,st]=func_QPLexPriority(H,f,A,b,[-1;0;0;0],[1;inf;inf;1],1,[1;1],1e-5);
verifyGreaterThan(t,st,0);
verifyLessThanOrEqual(t,max(x(2:3)),3e-5);
verifyGreaterThan(t,x(1),.5-4e-5);
end
function testPhysicalSlackScaling(t)
A=[-1 -.1 0 0;1 0 -.2 0]; b=[-.8;.2];
H=eye(4); f=[100;1e10;1e9;1];
[x,st]=func_QPLexPriority(H,f,A,b,[-1;0;0;0],[1;inf;inf;1],1,[.1;.2],1e-5);
verifyGreaterThan(t,st,0);
verifyLessThan(t,.1*x(2),3e-5);
verifyLessThan(t,abs(.2*x(3)-.6),5e-5);
end
