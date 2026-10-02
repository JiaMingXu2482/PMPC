function S = func_EyConsoleSummary(tagOrData)
%FUNC_EYCONSOLESUMMARY Print path-tracking metrics by maneuver segment.
%   Uses the same CG-to-reference-path normal-distance definition as
%   func_Metrics, so it also remains valid when the CarSim driver model is
%   disabled and Lat_Veh/Lat_Targ are both zero.
if ischar(tagOrData) || isstring(tagOrData)
    D = func_ReadERD(char(tagOrData));
else
    D = tagOrData;
end
mv = func_ManeuverFromName(D.Dataset);
if mv.combined
    C = func_CombinedCourse(mv.R);
    % CarSim may use a straight road with DLC cones or an X-Y DLC road.
    % On the recovery straight, station - global X is the road-specific offset.
    recovery = D.Xo>=C.dlc_end_x+10 & D.Xo<=C.turn_x-20;
    stationOffset = 0;
    if nnz(recovery)>=5
        stationOffset = median(D.Station(recovery)-D.Xo(recovery));
    end
    dlcEnd = C.dlc_end_x+stationOffset;
    turnStart = C.turn_x+stationOffset;
    names = {'DLC','J-turn'};
    S = struct('segments',repmat(struct('name','','ey_peak_m',NaN, ...
        'ey_rms_m',NaN,'vx_rms_kmh',NaN),1,2));
    for k = 1:2
        S.segments(k).name = names{k};
        if k == 2
            ix = find(D.Station>=turnStart);
        else
            ix = find(D.Station>=0 & D.Station<dlcEnd);
        end
        if numel(ix)<2
            fprintf('%s: not reached (no complete segment data).\n',names{k});
            continue;
        end
        ds = local_slice_(D,ix);
        metrics = func_Metrics(ds);
        S.segments(k).ey_peak_m = metrics.ey_pk;
        S.segments(k).ey_rms_m = metrics.ey_rms;
        S.segments(k).vx_rms_kmh = sqrt(mean(ds.Vx.^2));
        fprintf('%s: e_y max %.6f m | e_y RMS %.6f m | Vx RMS %.3f km/h\n', ...
            names{k},metrics.ey_pk,metrics.ey_rms,S.segments(k).vx_rms_kmh);
    end
    return;
end
M = func_Metrics(D);
S = struct('ey_peak_m', M.ey_pk, 'ey_rms_m', M.ey_rms);
fprintf('e_y peak (max|e_y|): %.6f m\n', S.ey_peak_m);
fprintf('e_y RMS:             %.6f m\n', S.ey_rms_m);
end

function E = local_slice_(D,ix)
E = D;
fields = fieldnames(D);
for k = 1:numel(fields)
    v = D.(fields{k});
    if isnumeric(v) && isvector(v) && numel(v)==D.N
        E.(fields{k}) = v(ix);
    end
end
E.N = numel(ix);
end
