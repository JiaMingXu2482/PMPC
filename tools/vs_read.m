function [names, M] = vs_read(path)
%VS_READ 读 CarSim 的 .vs/.vsb 结果文件，返回通道名与数据矩阵。
%
%   [names, M] = vs_read('.../cp_sla_mode2.vs');
%   names 是 string 行向量（通道名，取第一个 NameAlias）；
%   M 是 double，行 = 采样，列 = 通道，列顺序与 names 一致。
%
% 为什么需要它：模型里 logVec 固定 27 列（Stateflow 数据对象写死 [27 1]），
% 塞不进第 28 列；于是本分支用 CarSim 实测的四角压缩速度 CmpRD_* 复算 tau_d，
% 复算用的是控制器同一个 tau_d_effective.m，不是另写一套表。
%
% 实现与 PMPC_Results/tools/plot_section5_figs.m 里的 readVS 一致：
%   .vs  —— JSON 头，含通道名
%   .vsb —— 二进制，前 6 个 int32 头 + nch 行 float32

fid = fopen(path, 'r');
if fid < 0, error('vs_read:open', '打不开 %s', path); end
txt = fread(fid, inf, '*char').';
fclose(fid);

% 找 JSON 主体的结束位置（第一层大括号闭合）
depth = 0; inStr = false; esc = false; last = 0;
for i = 1:numel(txt)
    ch = txt(i);
    if inStr
        if esc, esc = false;
        elseif ch == '\', esc = true;
        elseif ch == '"', inStr = false; end
        continue
    end
    if ch == '"', inStr = true;
    elseif ch == '{', depth = depth + 1;
    elseif ch == '}'
        depth = depth - 1;
        if depth == 0, last = i; break; end
    end
end
hdr = jsondecode(txt(1:last));
ch = hdr.VsChannelGroup.Channels;
names = strings(1, numel(ch));
for i = 1:numel(ch)
    a = ch(i).NameAliases;
    if iscell(a), names(i) = string(a{1}); else, names(i) = string(a); end
end

fid = fopen([path(1:end-3) '.vsb'], 'r');
if fid < 0, error('vs_read:open', '打不开 %s', [path(1:end-3) '.vsb']); end
h6 = fread(fid, 6, 'int32');
nch = h6(6);
M = fread(fid, [nch, Inf], 'float32');
fclose(fid);
M = double(M.');
end
