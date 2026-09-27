function hil50(mdl)
%HIL50  把 pmpc_hil 里的 62 口散布 Mux 就地换成 50 口 —— NI In 块一个都不用重换
%
%   hil50            改 pmpc_hil.slx
%   hil50('别的名')  改指定模型
%
%   什么时候用:
%     2026-09-18 之前控制器吃 62 路, 而 CarSim/NI 只给 50 路, 所以模型里用一个
%     Mux(62) 把 50 路散布回 62 槽, 5-16 槽接一个常数 0。
%     后来把 func_StateEstimation 的下标整体重编成 1..50(脚本改 + 逐字节回归验过),
%     控制器直接吃 50 路, 散布器就多余了。
%
%     开发机上重新跑 build_hil 就行, 但**远程那台机器上的 pmpc_hil.slx 里,
%     50 个 Inport 已经被手工换成 NI In 块了** —— 重新生成就要再换 50 次。
%     所以有了这个脚本: 只动 Mux, 不动上游。
%
%   ⚠️ 全程**不靠块名**, 只顺着连线找上游 —— NI In 块叫什么名字都无所谓。
%
%   改之前:  [NI In] x50 --+--> Mux(62) --> PMPC_MF     (5-16 槽来自常数 0)
%            [常数 0] -----+
%   改之后:  [NI In] x50 -----> Mux(50) --> PMPC_MF
%
%   幂等: 已经是 50 口了会直接返回, 重复跑没事。

if nargin < 1 || isempty(mdl), mdl = 'pmpc_hil'; end
SLOTS = [1:4, 17:62];           % 50 路在原来 62 槽里的位置
load_system(mdl);

% ---------- 1. 顺着 PMPC_MF 的入口往上找那个 Mux ----------
ph = get_param([mdl '/PMPC_MF'], 'PortHandles');
ln = get_param(ph.Inport(1), 'Line');
assert(ln > 0, 'hil50:NoLine', 'PMPC_MF 的输入没接东西');
mux = get_param(get_param(ln,'SrcPortHandle'), 'Parent');
assert(strcmp(get_param(mux,'BlockType'),'Mux'), 'hil50:NotMux', ...
       'PMPC_MF 上游是 %s, 不是 Mux —— 这个模型不是预期的结构', get_param(mux,'BlockType'));

nIn = numel(get_param(mux,'PortHandles').Inport);
if nIn == numel(SLOTS)
    fprintf('  %s 已经是 Mux(%d) 了, 无需改动\n', mdl, nIn);
    close_system(mdl, 0);
    return
end
assert(nIn == 62, 'hil50:BadWidth', '上游 Mux 是 %d 口, 预期 62 或 50', nIn);

% ---------- 2. 记下 62 个槽各自来自哪个块的第几个出口 ----------
mph = get_param(mux,'PortHandles');
src = cell(62,2);
for s = 1:62
    l = get_param(mph.Inport(s), 'Line');
    assert(l > 0, 'hil50:Dangling', '第 %d 槽没接线', s);
    p = get_param(l,'LineParent');                 % 常数那 12 路是分支
    while p > 0, l = p; p = get_param(l,'LineParent'); end
    sp = get_param(l,'SrcPortHandle');
    src{s,1} = get_param(get_param(sp,'Parent'),'Name');
    src{s,2} = get_param(sp,'PortNumber');
end

%  5-16 槽应该全来自同一个常数块 —— 不是的话说明模型结构与预期不符, 立刻停
zname = src{5,1};
for s = 5:16
    assert(strcmp(src{s,1}, zname), 'hil50:ZeroFill', ...
           '第 %d 槽来自 %s, 与第 5 槽的 %s 不同 —— 5-16 本应都是那个常数 0', ...
           s, src{s,1}, zname);
end
zb = [mdl '/' zname];
assert(strcmp(get_param(zb,'BlockType'),'Constant') && ...
       str2double(get_param(zb,'Value')) == 0, 'hil50:NotZero', ...
       '%s 不是一个值为 0 的 Constant, 不敢删', zname);
fprintf('  5-16 槽的填充块: %s (Constant 0)\n', zname);

% ---------- 3. 换成 Mux(50) ----------
pos = get_param(mux,'Position');
local_kill(mux);
local_kill(zb);
NEW = [mdl '/pack50'];
add_block('simulink/Signal Routing/Mux', NEW, 'Inputs', num2str(numel(SLOTS)), ...
          'Position', pos);
for i = 1:numel(SLOTS)
    s = SLOTS(i);
    add_line(mdl, sprintf('%s/%d', src{s,1}, src{s,2}), ...
                  sprintf('pack50/%d', i), 'autorouting','on');
end
add_line(mdl, 'pack50/1', 'PMPC_MF/1', 'autorouting','on');

save_system(mdl);
fprintf('  %s: Mux(62)+%s -> Mux(50), 上游 %d 个块一个没动\n', ...
        mdl, zname, numel(SLOTS));
fprintf('  接下来照常生成代码即可。\n');
end

% -------------------------------------------------------------------------
function local_kill(blk)
%  先断线再删块; 只删挂在本块端口上的那一段
ph = get_param(blk,'PortHandles');
hs = [ph.Inport, ph.Outport];
for k = 1:numel(hs)
    for guard = 1:500
        l = get_param(hs(k),'Line');
        if l <= 0, break; end
        delete_line(l);
    end
end
delete_block(blk);
end
