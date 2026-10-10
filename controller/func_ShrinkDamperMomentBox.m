function [lo,hi] = func_ShrinkDamperMomentBox(lower,upper) %#codegen
% Preserve the historical margin unless it reverses a narrow one-sided box.
a=min(lower,upper); b=max(lower,upper);
hi=min(double(b>=0)*.9*b+double(b<0)*1.1*b,b);
lo=max(double(a<=0)*.9*a+double(a>0)*1.1*a,a);
if lo>hi
    % Contract the RANGE, not absolute distances from zero. This also
    % preserves a nonzero singleton, which cannot be shrunk towards zero.
    center=.5*a+.5*b; half=.45*(b-a);
    lo=center-half; hi=center+half;
end
end
