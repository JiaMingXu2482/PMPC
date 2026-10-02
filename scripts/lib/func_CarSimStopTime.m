function stopTime = func_CarSimStopTime(expandedRunText)
%FUNC_CARSIMSTOPTIME Read the effective time limit from a CarSim run file.
matches = regexp(expandedRunText, ...
    '(?m)^[ \t]*TSTOP[ \t]+([0-9]+(?:\.[0-9]+)?)[ \t\r]*$', ...
    'tokens');
stopTime = NaN;
if ~isempty(matches)
    stopTime = str2double(matches{end}{1});
end
end
