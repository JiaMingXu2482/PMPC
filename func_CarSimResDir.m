function RES = func_CarSimResDir()
%FUNC_CARSIMRESDIR  从 simfile.sim 解析出 CarSim 本次 Run 的结果目录
%   simfile.sim 每次 "Send to Simulink" 都会被 CarSim 改写, 指向新的 Run_<GUID>,
%   所以必须每次现读, 不能缓存、更不能硬编码。
sf = fileread('simfile.sim');
rn = regexp(sf,'ROOT_FILE_NAME\)\$\s+(\S+)','tokens','once');
wd = regexp(sf,'WORK_DIR\)\$\s+([^\r\n]+)','tokens','once');
if isempty(rn) || isempty(wd)
    error('func_CarSimResDir:Parse','simfile.sim 里找不到 ROOT_FILE_NAME / WORK_DIR');
end
RES = fullfile(strtrim(wd{1}), 'Results', rn{1});
if ~exist(RES,'dir')
    error('func_CarSimResDir:Missing','CarSim 结果目录不存在: %s', RES);
end
end
