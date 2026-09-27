function plot_week
%PLOT_WEEK  周报用图。读 scratchpad 里的 ERD, 输出 PNG 到 .\figs\
%   颜色沿用 CarSim 动画: 1 MPC 黑(动画里是白) / 2 ZENG 蓝 / 3 PMPC 红
SC = func_ErdDir();          % 项目内 erd_out (原为会话临时目录, 见 func_ErdDir)
OUT = fullfile(pwd,'figs');
if ~exist(OUT,'dir'), mkdir(OUT); end
set(0,'defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei');
set(0,'defaultAxesFontSize',10,'defaultLineLineWidth',1.4);

M = func_ReadERD(fullfile(SC,'MPC'));
Z = func_ReadERD(fullfile(SC,'ZENG'));
P = func_ReadERD(fullfile(SC,'PMPC'));
O = func_ReadERD(fullfile(SC,'OLD_ZENG'));      % ZENG 修复前
CL = {[0 0 0], [0 0.35 0.9], [0.85 0.1 0.1]};
NM = {'1 MPC (固定权重)','2 ZENG (rho 调权+纵向)','3 PMPC (本文)'};
A  = {M,Z,P};
MU = 0.5; MUE = 0.4275; G = 9.80665; EMAX = 0.5325;

W  = load('WayPoints_Type1.mat'); W = W.WayPoints_Collect;
XR = W(:,2); YR = W(:,3);

%% ---- 图1 路径跟踪 ----
f=figure('Position',[80 80 900 620],'Color','w');
subplot(2,1,1); hold on; grid on; box on
h0=plot(XR,YR,'--','Color',[.45 .45 .45],'LineWidth',1.1);
h1=plot(XR,YR+EMAX,':','Color',[.7 .7 .7]); plot(XR,YR-EMAX,':','Color',[.7 .7 .7]);
hh=gobjects(1,3);
for k=1:3, hh(k)=plot(A{k}.Xo, A{k}.Yo, 'Color', CL{k}); end
xlabel('纵向位置 X (m)'); ylabel('侧向位置 Y (m)');
title('图1  DLC 80 km/h, mu=0.5 — 路径跟踪'); xlim([0 200]);
legend([h0 h1 hh],[{'参考中心线','车道边界 +/-0.5325 m'},NM],'Location','northwest','Box','off');
subplot(2,1,2); hold on; grid on; box on
plot([0 200],[ EMAX  EMAX],':','Color',[.7 .7 .7]);
plot([0 200],[-EMAX -EMAX],':','Color',[.7 .7 .7]);
for k=1:3, plot(A{k}.Xo, eyf(A{k},XR,YR), 'Color', CL{k}); end
xlabel('纵向位置 X (m)'); ylabel('侧向误差 e_y (m)'); xlim([0 200]); ylim([-0.7 0.7]);
title(sprintf('侧向误差   RMS: %.4f / %.4f / %.4f m', ...
      rms2(eyf(M,XR,YR)), rms2(eyf(Z,XR,YR)), rms2(eyf(P,XR,YR))));
sav(f,OUT,'fig1_路径跟踪');

%% ---- 图2 防侧翻 ----
f=figure('Position',[80 80 900 620],'Color','w');
subplot(2,1,1); hold on; grid on; box on
for k=1:3, plot(A{k}.t, A{k}.LTR, 'Color', CL{k}); end
xlabel('时间 (s)'); ylabel('LTR'); title('图2  侧向载荷转移率与侧倾角');
legend(NM,'Location','southwest','Box','off');
subplot(2,1,2); hold on; grid on; box on
for k=1:3, plot(A{k}.t, A{k}.Roll, 'Color', CL{k}); end
xlabel('时间 (s)'); ylabel('车身侧倾角 (deg)');
title(sprintf('侧倾峰值: %.3f / %.3f / %.3f deg', ...
      max(abs(M.Roll)), max(abs(Z.Roll)), max(abs(P.Roll))));
sav(f,OUT,'fig2_防侧翻');

%% ---- 图3 Zeng 横摆约束满足情况 ----
f=figure('Position',[80 80 900 700],'Color','w');
for k=1:3
    D=A{k}; subplot(3,1,k); hold on; grid on; box on
    bd = rad2deg(MU*G./max(D.Vx/3.6,1));
    h1=plot(D.t, abs(D.AVz), 'Color', CL{k});
    h2=plot(D.t, bd, 'k--','LineWidth',1.1);
    vio = 100*mean(abs(D.AVz)>bd);
    xlabel('时间 (s)'); ylabel('|r| (deg/s)');
    title(sprintf('%s   超限占比 %.1f%%', NM{k}, vio));
    if k==1, legend([h1 h2],{'|r|','Zeng 界 mu*g/V_x'},'Location','northwest','Box','off'); end
    ylim([0 40]); xlim([0 9]);
end
sav(f,OUT,'fig3_横摆约束');

%% ---- 图4 关键发现: 稳态近似的保守度 ----
f=figure('Position',[80 80 900 620],'Color','w');
subplot(2,1,1); hold on; grid on; box on
hh=gobjects(1,3);
for k=1:3
    D=A{k}; hh(k)=plot(D.t, abs(D.Vx/3.6.*deg2rad(D.AVz))/G, 'Color', CL{k});
end
h2=plot([0 9],[MU MU],'k--'); text(7.2,MU+0.06,'Zeng 界 mu*g = 0.500 g');
xlabel('时间 (s)'); ylabel('|V_x * r| / g');
title('图4  稳态近似 a_y ~ V_x*r  (Zeng 判据实际约束的量)');
legend([hh h2],[NM,{'mu*g'}],'Location','northwest','Box','off'); ylim([0 0.9]);
subplot(2,1,2); hold on; grid on; box on
for k=1:3, plot(A{k}.t, abs(A{k}.Ay), 'Color', CL{k}); end
plot([0 9],[MUE MUE],'k--'); text(0.2,MUE+0.05,'轮胎实际能力 mu_{eff}*g = 0.4275 g');
xlabel('时间 (s)'); ylabel('|A_y| / g'); ylim([0 0.9]);
title(sprintf(['真实侧向加速度(CarSim 实测) 峰值 %.3f / %.3f / %.3f g' ...
      '  — 三者都顶在轮胎能力上, 谁都没超附着极限'], ...
      max(abs(M.Ay)), max(abs(Z.Ay)), max(abs(P.Ay))));
sav(f,OUT,'fig4_稳态近似保守度');

%% ---- 图5 车速与油门 ----
f=figure('Position',[80 80 900 620],'Color','w');
subplot(2,1,1); hold on; grid on; box on
hh=gobjects(1,3);
for k=1:3, hh(k)=plot(A{k}.t, A{k}.Vx, 'Color', CL{k}); end
h2=plot([0 9],[80 80],'k:'); xlabel('时间 (s)'); ylabel('V_x (km/h)');
title(sprintf('图5  车速   均值 %.2f / %.2f / %.2f km/h', ...
      mean(M.Vx), mean(Z.Vx), mean(P.Vx)));
legend([hh h2],[NM,{'目标 80'}],'Location','southwest','Box','off');
subplot(2,1,2); hold on; grid on; box on
for k=1:3, plot(A{k}.t, A{k}.Throttle, 'Color', CL{k}); end
xlabel('时间 (s)'); ylabel('油门开度'); ylim([-0.05 1.05]);
title('油门开度(ZENG 因限速目标摆动而频繁触轨, 已用一阶滤波抑制快速翻转)');
sav(f,OUT,'fig5_车速与油门');

%% ---- 图6 执行器 ----
f=figure('Position',[80 80 900 620],'Color','w');
subplot(2,1,1); hold on; grid on; box on
for k=1:3, plot(A{k}.t, A{k}.Steer_SW, 'Color', CL{k}); end
xlabel('时间 (s)'); ylabel('方向盘转角 (deg)'); title('图6  执行器输出');
legend(NM,'Location','northwest','Box','off');
subplot(2,1,2); hold on; grid on; box on
for k=1:3
    D=A{k}; dif=(D.My_Bk_L1+D.My_Bk_L2)-(D.My_Bk_R1+D.My_Bk_R2);
    plot(D.t, dif, 'Color', CL{k});
end
xlabel('时间 (s)'); ylabel('左右制动力矩差 (N*m)');
title(sprintf('差动制动 RMS %.0f / %.0f / %.0f N*m  (PMPC 靠优先级变量几乎不用制动)', ...
      rms2(dbf(M)), rms2(dbf(Z)), rms2(dbf(P))));
sav(f,OUT,'fig6_执行器');

%% ---- 图7 ZENG 本周修复 before/after ----
f=figure('Position',[80 80 1000 680],'Color','w');
co={[.75 .35 .1],[0 0.35 0.9]}; lg={'修复前','修复后'};
subplot(2,2,1); hold on; grid on; box on
plot(O.t,O.Steer_SW,'Color',co{1}); plot(Z.t,Z.Steer_SW,'Color',co{2});
yl=ylim; patch([6.8 9 9 6.8],[yl(1) yl(1) yl(2) yl(2)],[1 .9 .6], ...
      'FaceAlpha',.35,'EdgeColor','none'); ylim(yl);
xlabel('时间 (s)'); ylabel('方向盘转角 (deg)'); legend(lg,'Location','northwest','Box','off');
title('(a) 方向盘转角 — 全程');
subplot(2,2,2); hold on; grid on; box on
plot(O.t,O.Steer_SW,'Color',co{1}); plot(Z.t,Z.Steer_SW,'Color',co{2});
xlim([6.2 9]); xlabel('时间 (s)'); ylabel('方向盘转角 (deg)');
title(sprintf('(b) 尾段放大  |SW| 峰(t>6.8s) %.1f -> %.1f deg', ...
      max(abs(O.Steer_SW(O.t>6.8))), max(abs(Z.Steer_SW(Z.t>6.8)))));
subplot(2,2,3); hold on; grid on; box on
plot(O.t,O.Throttle,'Color',co{1}); plot(Z.t,Z.Throttle,'Color',co{2});
xlabel('时间 (s)'); ylabel('油门开度'); ylim([-0.05 1.05]);
title(sprintf('(c) 油门 0-1 翻转 %d -> %d 次', flips(O.Throttle), flips(Z.Throttle)));
subplot(2,2,4); hold on; grid on; box on
plot(O.t,O.My_Bk_R1,'Color',co{1}); plot(Z.t,Z.My_Bk_R1,'Color',co{2});
xlabel('时间 (s)'); ylabel('右前轮制动力矩 (N*m)');
title(sprintf('(d) 四轮力矩变化率 RMS %.0f -> %.0f N*m/s', dRMS(O), dRMS(Z)));
sgtitle('图7  2 ZENG 本周三处修复的效果','FontName','Microsoft YaHei','FontSize',12);
sav(f,OUT,'fig7_ZENG修复前后');

%% ---- 图8 指标汇总表 ----
rows = {
 'e_y RMS (m)',            @(D) rms2(eyf(D,XR,YR)),                  '%.4f', 1
 '|e_y| 峰 (m)',           @(D) max(abs(eyf(D,XR,YR))),              '%.4f', 1
 '越界占比 (%)',            @(D) 100*mean(abs(eyf(D,XR,YR))>EMAX),    '%.2f', 1
 '|LTR| 峰',               @(D) max(abs(D.LTR)),                     '%.4f', 1
 '侧倾峰 (deg)',            @(D) max(abs(D.Roll)),                    '%.3f', 1
 '|r|>mu*g/Vx (%)',        @(D) 100*mean(abs(D.AVz)>rad2deg(MU*G./max(D.Vx/3.6,1))), '%.1f', 1
 '尾段|SW|峰 (deg)',        @(D) max(abs(D.Steer_SW(D.t>6.8))),       '%.1f', 1
 '差动制动 RMS (N*m)',      @(D) rms2(dbf(D)),                        '%.0f', 1
 '轮缸力矩变化率 RMS',       @(D) dRMS(D),                             '%.0f', 1
 '油门速率 RMS (1/s)',      @(D) rms2(diff(D.Throttle))/(D.t(2)-D.t(1)), '%.2f', 1
 '平均车速 (km/h)',         @(D) mean(D.Vx),                          '%.2f', 0
};
f=figure('Position',[80 80 780 480],'Color','w');
ax=axes('Position',[0 0 1 1]); axis(ax,'off'); xlim([0 1]); ylim([0 1]); hold on
nr = size(rows,1);
y0 = 0.855;  dy = 0.062;  xc = [0.55 0.72 0.89];
hd = {'1 MPC','2 ZENG','3 PMPC'};
for k=1:3
    text(xc(k), y0+dy, hd{k}, 'FontName','Microsoft YaHei','FontSize',11, ...
         'FontWeight','bold','HorizontalAlignment','right');
end
text(0.06, y0+dy, '指标','FontName','Microsoft YaHei','FontSize',11,'FontWeight','bold');
plot([0.05 0.92],[y0+dy/2 y0+dy/2],'k-','LineWidth',1);
for i=1:nr
    yy = y0 - (i-1)*dy;
    text(0.06, yy, rows{i,1}, 'FontName','Microsoft YaHei','FontSize',10.5, ...
         'Interpreter','none');
    v = [rows{i,2}(M) rows{i,2}(Z) rows{i,2}(P)];
    [~,b] = min(v); if ~rows{i,4}, [~,b] = max(v); end
    for k=1:3
        if k==b, w='bold'; c=[.75 .1 .1]; else, w='normal'; c=[0 0 0]; end
        text(xc(k), yy, sprintf(rows{i,3}, v(k)), 'FontName','Consolas', ...
             'FontSize',10.5,'HorizontalAlignment','right','FontWeight',w,'Color',c);
    end
    if mod(i,2)==0
        patch([0.05 0.92 0.92 0.05],[yy-dy/2 yy-dy/2 yy+dy/2 yy+dy/2], ...
              [.95 .95 .95],'FaceAlpha',.6,'EdgeColor','none');
    end
end
plot([0.05 0.92],[y0-(nr-1)*dy-dy/2  y0-(nr-1)*dy-dy/2],'k-','LineWidth',1);
text(0.06, y0-nr*dy-0.02, ...
     '红色加粗 = 该行最优    工况: DLC 80 km/h, \mu=0.5, N_s=12', ...
     'FontName','Microsoft YaHei','FontSize',9.5,'Color',[.35 .35 .35]);
text(0.5, 0.965, '图8  三控制器指标汇总','FontName','Microsoft YaHei', ...
     'FontSize',12.5,'HorizontalAlignment','center','FontWeight','bold');
set(ax,'Children',flipud(get(ax,'Children')));
sav(f,OUT,'fig8_指标汇总');

fprintf('\n8 张图已存到  %s\n', OUT);
end

function e = eyf(D,XR,YR), e = D.Yo - interp1(XR,YR,D.Xo,'linear','extrap'); end
function d = dbf(D), d = (D.My_Bk_L1+D.My_Bk_L2)-(D.My_Bk_R1+D.My_Bk_R2); end
function sav(f,OUT,nm)
print(f, fullfile(OUT,[nm '.png']), '-dpng','-r150'); close(f);
fprintf('  %s.png\n', nm);
end
function y = rms2(x), y = sqrt(mean(x(:).^2)); end
function n = flips(th)
st = zeros(size(th)); st(th>0.95)=1; st(th<0.05)=-1;
s = st(st~=0); n = sum(diff(s)~=0);
end
function r = dRMS(D)
dt = D.t(2)-D.t(1);
v = [D.My_Bk_L1 D.My_Bk_R1 D.My_Bk_L2 D.My_Bk_R2];
r = mean(sqrt(mean(diff(v).^2,1))/dt);
end
