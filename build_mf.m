function build_mf()
%BUILD_MF  在 Subsystem1 里建 MATLAB Function block (阶段 D)
%   块内容只有一行调用 pmpc_step; 参数 P 走 Scope=Parameter, 从工作区的
%   PMPC_P 解析; 跨拍状态用 persistent, 首拍用 P.S0 初始化。
%
%   建完直接接到 Outport stable_ctr —— 阶段 D 之后 S-Function 块已删除,
%   这是子系统里唯一的控制器。

MDL = 'cq3_2019';
SUB = [MDL '/Subsystem1'];
BLK = [SUB '/PMPC_MF'];
load_system(MDL);

%  幂等: 先把上一次留下的块和线彻底清掉
for nm = {'PMPC_MF'}
    if ~isempty(find_system(SUB,'SearchDepth',1,'Name',nm{1}))
        b = [SUB '/' nm{1}];
        ph = get_param(b,'PortHandles');
        for h = [ph.Inport ph.Outport]
            l = get_param(h,'Line');
            if l > 0, delete_line(l); end
        end
        delete_block(b);
    end
end
add_block('simulink/User-Defined Functions/MATLAB Function', BLK, ...
          'Position',[200 200 320 260]);

% ---- 写块内容 ----
%  用 strjoin+newline 拼, 不写转义序列(省得被各层工具折叠)
src = strjoin({ ...
 'function sys = pmpc_block_blk(u, PMPC_P)' ...
 '%#codegen' ...
 '%  块里只写一行调用 —— 实体在 pmpc_block.m,' ...
 '%  这样可以单独对它跑 codegen 调试(见 cg_block.m)。' ...
 'sys = pmpc_block(u, PMPC_P);' ...
 'end' ...
 }, newline);
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','Path',BLK);
assert(~isempty(ch), 'build_mf:NoChart', '没找到 MATLAB Function 的 chart 对象');
ch.Script = src;

% ---- 把 P 设成 Parameter(不占输入端口) ----
d = ch.find('-isa','Stateflow.Data','-and','Name','PMPC_P');
assert(~isempty(d), 'build_mf:NoData', '没找到数据对象 PMPC_P');
d.Scope = 'Parameter';
%  ⚠️ 必须设成不可调 —— 这是块层面的 coder.Constant。
%  模型的 DefaultParameterBehavior 是 Tunable, 参数就不是编译期常量,
%  于是 nvars = Nu*Nc+Ne+Nr 这类维度又变回运行时值, 数组尺寸推不出来,
%  块会报"无法确定输出大小和/或类型"。HIL 上参数本来就是 build 时固化的。
d.Tunable = false;

% ---- 接线: x(Inport) -> 块 -> stable_ctr(Outport) ----
%  ⚠️ find_system 默认**跳过被注释的块**, 清理旧块时别只靠它判断存在性,
%     必要时用 get_param 直接试。(阶段 D 删 S-Function 时踩过)
ph = get_param([SUB '/stable_ctr'],'PortHandles');
l = get_param(ph.Inport(1),'Line');
if l > 0, delete_line(l); end
add_line(SUB, 'x/1', 'PMPC_MF/1', 'autorouting','on');
add_line(SUB, 'PMPC_MF/1', 'stable_ctr/1', 'autorouting','on');

save_system(MDL);
fprintf('  已建 PMPC_MF 并保存模型\n');
end
