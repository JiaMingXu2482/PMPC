function test_v17_inverse_10ms()
%TEST_V17_INVERSE_10MS Standalone regression for the 100 Hz inverse / 1 kHz plant.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
cases = {'constant', 'sine'};
for c = 1:numel(cases)
    clear v17_inv_mf_impl_10ms v17_fwd_mf_impl
    N = 501;
    t = (0:N-1)'*0.001;
    I = zeros(N,1); Flo = I; Fhi = I; Fclip = I; Fach = I; Fplant = I;
    Fdes = I; x = I; v = I;
    switch cases{c}
        case 'constant'
            v(:) = 250;
            Fdes(:) = 1200;
        case 'sine'
            x = 4*sin(2*pi*3*t);
            v = 4*2*pi*3*cos(2*pi*3*t);
            Fdes = 10*v;
    end
    for k = 1:N
        [I(k), Flo(k), Fhi(k), Fclip(k), Fach(k)] = ...
            v17_inv_mf_impl_10ms(Fdes(k),t(k),30,x(k),v(k),1);
        Fplant(k) = v17_fwd_mf_impl(t(k),30,x(k),v(k),I(k),1);
    end
    tick = (1:10:N)';
    settled = tick(tick >= 201);
    assert(all(isfinite([I;Flo;Fhi;Fclip;Fach;Fplant])));
    assert(all(Flo <= Fhi));
    assert(all(I >= 0 & I <= 1.6));
    assert(all(abs(diff(I(settled))) <= 1.6+1e-10));
    fprintf('%s: max_dI=%.3f A, TV=%.3f A, mean |Fplant-Fach|=%.1f N, mean |Fplant-Fclip|=%.1f N\n', ...
        cases{c},max(abs(diff(I(settled)))),sum(abs(diff(I(settled)))), ...
        mean(abs(Fplant(settled)-Fach(settled))), ...
        mean(abs(Fplant(settled)-Fclip(settled))));
    moving = abs(v(201:end)) > 20;
    q = Fplant(201:end).*v(201:end);
    fprintf('  F*v negative fraction = %.1f%%; signed work = %.3f J\n', ...
        100*mean(q(moving)<0),sum(q)*0.001/1000);
    jumps = find(abs(diff(I(settled))) > 0.4);
    for j = 1:min(numel(jumps),5)
        a = settled(jumps(j)); b = settled(jumps(j)+1);
        fprintf('  jump t=%.3f: I %.2f->%.2f, Fdes %.0f->%.0f, bounds [%.0f,%.0f]->[%.0f,%.0f]\n', ...
            t(b),I(a),I(b),Fdes(a),Fdes(b),Flo(a),Fhi(a),Flo(b),Fhi(b));
    end
end
end
