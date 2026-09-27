function ok = func_CarSimRunning()
%FUNC_CARSIMRUNNING  CarSim Browser / CSLM 在不在跑(求解器许可要它)
%   ok = func_CarSimRunning()   true = 在跑, 或者查不了(此时不拦)
%
%   为什么要这个检查:
%     CarSim 求解器要从运行中的 CarSim Browser(或 CSLM.exe)取许可。没开的话
%     vs_sf 不是干脆报错, 而是**弹一个原生 Windows 模态对话框**
%         "VS Solver (CarSim 2019.0) Problem"
%         "You need a running copy of CarSim Browser or CSLM.exe ..."
%     在 matlab -batch 里没人点它, 于是作业**永远挂着**。
%     2026-09-24 实际踩到: 一个批量作业卡了一个多小时, CPU 几乎不动,
%     最后是从 CarSim 的 LastRun_log.txt 里才看出原因。
%     所以宁可提前查一下, 用一句能读懂的错误换掉那个死等。
%
%   查不了就返回 true —— 这个检查只是为了避免死等, 不该反过来变成新的拦路虎。

ok = true;
if ~ispc, return; end
[st, out] = system('tasklist /NH /FO CSV');
if st ~= 0 || isempty(out), return; end            % 查不了, 不拦
ok = ~isempty(regexpi(out, '"[^"]*(carsim|cslm)[^"]*\.exe"', 'once'));
end
