function root = func_ProjectRoot()
%FUNC_PROJECTROOT  Return the absolute PMPC project root directory.
root = fileparts(fileparts(mfilename('fullpath')));
end
