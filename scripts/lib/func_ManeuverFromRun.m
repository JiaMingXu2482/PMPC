function mv = func_ManeuverFromRun(dsname, runAllPar)
%FUNC_MANEUVERFROMRUN Resolve J-turn geometry from CarSim's expanded run.
% Segment-builder roads carry the authoritative radius in Run_all.par.
% Generated X-Y combined roads have no SEGMENT_RADIUS and retain the
% radius encoded when that road was generated in the dataset label.
mv = func_ManeuverFromName(dsname);
if mv.type ~= 5 && mv.type ~= 6
    return;
end
if exist(runAllPar,'file') ~= 2
    error('func_ManeuverFromRun:MissingPar', ...
        'J-turn CarSim expanded Run_all.par is missing: %s',runAllPar);
end
raw = fileread(runAllPar);
tokens = regexp(raw,'(?m)^SEGMENT_RADIUS\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$','tokens');
if mv.type == 6 && isempty(tokens)
    return;
end
if numel(tokens) ~= 1
    error('func_ManeuverFromRun:RadiusCount', ...
        'Expected exactly one SEGMENT_RADIUS in %s; found %d.', ...
        runAllPar,numel(tokens));
end
radius = str2double(tokens{1}{1});
if ~isfinite(radius) || radius <= 0
    error('func_ManeuverFromRun:InvalidRadius', ...
        'Invalid J-turn road radius in %s.',runAllPar);
end
mv.R = radius;
mv.wp = sprintf('WayPoints_Type%d_R%g.mat',mv.type,radius);
if mv.type == 6
    lengths = regexp(raw,'(?m)^SEGMENT_LENGTH\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$','tokens');
    if isempty(lengths)
        error('func_ManeuverFromRun:MissingStraight', ...
            'Combined road first straight length is missing in %s.',runAllPar);
    end
    mv.turn_x = str2double(lengths{1}{1});
    if ~isfinite(mv.turn_x) || mv.turn_x <= 0
        error('func_ManeuverFromRun:InvalidStraight', ...
            'Invalid combined road first straight length in %s.',runAllPar);
    end
end
end
