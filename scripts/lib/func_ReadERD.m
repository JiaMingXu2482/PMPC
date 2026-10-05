function D = func_ReadERD(tag)
%FUNC_READERD  读 CarSim ERD (.vs 头 + .vsb 二进制), 返回 struct
%   D.(通道别名) = 列向量;  D.t = 时间;  D.Dataset / D.N
%   用法: D = func_ReadERD('C:\...\B_ZENG');   % 不带扩展名
g  = jsondecode(fileread([tag '.vs']));
g  = g.VsChannelGroup;
ch = g.Channels;
if iscell(ch), n = numel(ch); else, n = numel(ch); end
fid = fopen([tag '.vsb'],'r');
fseek(fid, 24, 'bof');                       % 24 字节前缀
a = fread(fid, inf, 'float32=>double', 'l');
fclose(fid);
m = floor(numel(a)/n);
a = reshape(a(1:m*n), n, m).';               % m x n
D = struct();
for i = 1:n
    if iscell(ch), ci = ch{i}; else, ci = ch(i); end
    al = ci.NameAliases;                      % MATLAB jsondecode 去掉了空格
    if ~iscell(al), al = {al}; end
    for j = 1:numel(al)
        nm = matlab.lang.makeValidName(al{j});
        if ~isfield(D, nm), D.(nm) = a(:,i); end
    end
end
D.t       = g.XStart + (0:m-1).' * g.XStep;
D.N       = m;
D.Dataset = g.Dataset;
D.RunAllPar = fullfile(fileparts(tag),'Run_all.par');
end
