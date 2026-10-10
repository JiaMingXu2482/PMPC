function tests=test_damper_moment_box
tests=functiontests(localfunctions);
end
function setupOnce(~)
startup_pmpc();
end
function testNarrowPositiveAndNegative(t)
[lo,hi]=func_ShrinkDamperMomentBox(98,100);
verifyEqual(t,[lo hi],[98.1 99.9],'AbsTol',1e-12);
[lo,hi]=func_ShrinkDamperMomentBox(-100,-98);
verifyEqual(t,[lo hi],[-99.9 -98.1],'AbsTol',1e-12);
end
function testNonzeroSingleton(t)
for v=[-100 0 100]
    [lo,hi]=func_ShrinkDamperMomentBox(v,v);
    verifyEqual(t,[lo hi],[v v],'AbsTol',1e-12);
end
end
function testHistoricalValidIntervalsUnchanged(t)
[lo,hi]=func_ShrinkDamperMomentBox(-100,200);
verifyEqual(t,[lo hi],[-90 180],'AbsTol',1e-12);
[lo,hi]=func_ShrinkDamperMomentBox(10,100);
verifyEqual(t,[lo hi],[11 90],'AbsTol',1e-12);
end
function testAlwaysContainedAndNonempty(t)
for a=-100:4:100
    for width=[0 .01 1 10 100 1000]
        [lo,hi]=func_ShrinkDamperMomentBox(a,a+width);
        verifyLessThanOrEqual(t,lo,hi+1e-10);
        verifyGreaterThanOrEqual(t,lo,a-1e-10);
        verifyLessThanOrEqual(t,hi,a+width+1e-10);
    end
end
end
