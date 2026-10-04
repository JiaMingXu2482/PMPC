function output = func_ReplaceLastCarSimParameter(input,key,value)
%FUNC_REPLACELASTCARSIMPARAMETER Replace the final expanded CarSim override.
if isstring(input), input = char(input); end
if isstring(key), key = char(key); end
if isstring(value), value = char(value); end
pattern = ['(?m)^' regexptranslate('escape',key) '\s+[^\r\n]*'];
[first,last] = regexp(input,pattern);
assert(~isempty(first),'func_ReplaceLastCarSimParameter:MissingKey', ...
    'CarSim input has no %s entry.',key);
output = [input(1:first(end)-1),key,' ',value,input(last(end)+1:end)];
end
