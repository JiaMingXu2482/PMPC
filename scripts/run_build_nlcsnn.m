function run_build_nlcsnn()
% Driver: CarSim libs, setup_pmpc, build. On failure print the full report AND
% dump the Stateflow Data objects of both charts (config-level diagnosis).
startup_pmpc();
addpath('D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers');
addpath('D:\Program Files\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers\Matlab84+');
try
    setup_pmpc();
catch ME
    fprintf('setup_pmpc FAILED: %s\n', ME.message);
end

fprintf('\n=== build_nlcsnn_multirate ===\n');
failed = false;
try
    build_nlcsnn_multirate();
    fprintf('  >>> build finished OK\n');
catch ME
    failed = true;
    fprintf('  >>> build FAILED: %s\n', ME.identifier);
    fprintf('%s\n', ME.getReport());
    c = ME.cause; lvl = 0;
    while ~isempty(c) && lvl < 4
        lvl = lvl + 1;
        for k = 1:numel(c)
            fprintf('--- cause L%d: %s\n    %s\n', lvl, c{k}.identifier, c{k}.message);
        end
        c = c{1}.cause;
    end
end

if failed
    fprintf('\n=== chart Data dump (in-memory state at failure) ===\n');
    rt = sfroot;
    for p = {'pmpc_mil/PMPC_MF', 'pmpc_mil/NLCSNN_Plant/plant_fn'}
        ch = rt.find('-isa','Stateflow.EMChart','Path',p{1});
        if isempty(ch)
            fprintf('[%s] no chart\n', p{1});
            continue;
        end
        fprintf('[%s]\n', p{1});
        try
            s = ch.Script; nl = find(s == char(10), 1);
            if isempty(nl), nl = numel(s) + 1; end
            fprintf('   script line 1: %s\n', s(1:nl-1));
        catch
        end
        d = ch.find('-isa','Stateflow.Data');
        for k = 1:numel(d)
            extra = '';
            try, extra = sprintf(' Tunable=%d', d(k).Tunable); catch, end
            try, extra = [extra sprintf(' Port=%g', d(k).Port)]; catch, end %#ok<AGROW>
            fprintf('   %-14s Scope=%-10s%s\n', d(k).Name, d(k).Scope, extra);
        end
    end
    fprintf('\n=== plant_fn port count ===\n');
    try
        ph = get_param('pmpc_mil/NLCSNN_Plant/plant_fn','PortHandles');
        fprintf('   %d in / %d out\n', numel(ph.Inport), numel(ph.Outport));
    catch ME2
        fprintf('   ERR %s\n', ME2.message);
    end
end
fprintf('\n=== done ===\n');
end
