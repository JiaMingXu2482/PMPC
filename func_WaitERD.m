function nb = func_WaitERD(RES, logstamp0, tmo)
%FUNC_WAITERD  等 CarSim 求解器真正写完 ERD。
%  判据是 LastRun_log.txt 被重写且出现 "Run stopped at t =" —— 这是求解器
%  收尾时才写的。只靠 .vsb 大小稳定不可靠: CarSim 分块刷盘, 块间停顿可能
%  超过判稳窗口, 于是复制到**写了一半**的文件(2026-09-12 实测: 同一条轨迹
%  一次 160 拍、一次 173 拍, 前 160 拍逐字节相同 —— 短的那份是截断)。
%  logstamp0 = 仿真前 dir(log).datenum, 用来确认日志确实是这一轮写的。
if nargin < 3, tmo = 90; end
lg  = fullfile(RES,'LastRun_log.txt');
vsb = fullfile(RES,'LastRun.vsb');
t0  = tic;
while toc(t0) < tmo
    d = dir(lg);
    if ~isempty(d) && d.datenum > logstamp0
        txt = '';
        try, txt = fileread(lg); catch, end
        if ~isempty(strfind(txt,'Run stopped at t ='))   %#ok<STREMP>
            a = dir(vsb); pause(0.3); b = dir(vsb);
            if ~isempty(a) && ~isempty(b) && a.bytes==b.bytes && a.bytes>0
                nb = b.bytes; return
            end
        end
    end
    pause(0.2);
end
error('func_WaitERD:Timeout','等了 %.0f s, CarSim 仍未写完 ERD', toc(t0));
end
