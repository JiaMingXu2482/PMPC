function summary=plot_legacy_longcoord_replay(outputRoot)
% Reusable figures for historical-controller replay; no model/tuning edits.
root=startup_pmpc();
if nargin==0, outputRoot=fullfile(root,'simulation_results','legacy_longcoord_20261009'); end
labels={'MPC','ZENG','PMPC'};
meta=load(fullfile(outputRoot,'manifest.mat'),'manifest');
courses=meta.manifest.courses;
if isfield(meta.manifest,'turn_x'), turnX=meta.manifest.turn_x; else, turnX=330; end
colors=[0.12 .42 .86;.88 .34 .25;.18 .62 .42];
rows={}; figDir=fullfile(outputRoot,'figures'); if ~isfolder(figDir), mkdir(figDir); end
for c=1:numel(courses)
    course=courses{c}; ref=load(fullfile(outputRoot,[course '_reference.mat']));
    S=cell(1,3); Dall=cell(1,3);
    for j=1:3
        runDir=fullfile(outputRoot,'Results',['Run_' course '_' labels{j}]);
        assert(isfile(fullfile(runDir,'COMPLETE.mat')),'Incomplete run: %s',runDir);
        D=func_ReadERD(fullfile(runDir,'LastRun')); Dall{j}=D;
        [ey,station]=project_(ref.W,D);
        signals=table(D.t,station,D.Vx,ey,D.Beta,D.AVz,D.LTR,D.Steer_SW, ...
            'VariableNames',{'time_s','station_m','vx_kmh','ey_m','beta_deg', ...
            'yaw_rate_deg_s','ltr','steering_wheel_deg'});
        S{j}=signals; save(fullfile(runDir,'vehicle_signals.mat'),'signals','D','-v7.3');
        writetable(signals,fullfile(runDir,'vehicle_signals.csv'));
    end
    % Compare on the same reference-station interval, not different endpoints.
    sLow=max(cellfun(@(s)min(s.station_m),S));
    sHigh=min(cellfun(@(s)max(s.station_m),S));
    if strcmp(course,'COM')
        [~,iDlc]=min(abs(ref.W(:,2)-200));
        [~,iTurn]=min(sum((ref.W(:,2:3)-[turnX 0]).^2,2));
        dlcEnd=ref.W(iDlc,7); turnStart=ref.W(iTurn,7);
        segments={'DLC','Jturn'}; bounds=[sLow,dlcEnd;turnStart,sHigh];
    else
        segments={'Slalom'}; bounds=[sLow,sHigh];
    end
    f=figure_([100 100 950 830]); tiledlayout(3,1,'TileSpacing','compact');
    vars={'vx_kmh','ey_m','ltr'}; yl={'V_x (km/h)','e_y (m)','LTR'};
    for a=1:3
        nexttile; hold on;
        for j=1:3
            plot(S{j}.station_m,S{j}.(vars{a}),'Color',colors(j,:), ...
                'LineWidth',1.4,'DisplayName',labels{j});
        end
        grid on; ylabel(yl{a}); xlim([sLow sHigh]);
        if strcmp(course,'COM'), xline(dlcEnd,':','HandleVisibility','off'); xline(turnStart,':','HandleVisibility','off'); end
        if a==1, legend('Location','best'); title([course ' | historical longitudinal coordination']); end
    end
    xlabel('Reference station (m)'); export_(f,figDir,[course '_states']);
    logPath=fullfile(outputRoot,'Results',['Run_' course '_PMPC'],'pmpc_mil_logs.mat');
    logged=load(logPath,'logsout');
    if any(strcmp(logged.logsout.getElementNames,'longitudinal_diag'))
        ts=logged.logsout.getElement('longitudinal_diag').Values;
        values=squeeze(ts.Data);
        if size(values,1)~=numel(ts.Time), values=values'; end
        f=figure_([100 100 950 620]); tiledlayout(2,1,'TileSpacing','compact');
        nexttile; plot(S{3}.time_s,S{3}.vx_kmh,'LineWidth',1.3); hold on;
        % The logged t=0 state is an uninitialized diagnostic, not a zero-speed command.
        capMask=ts.Time>0;
        plot(ts.Time(capMask),3.6*values(capMask,1),'--','LineWidth',1.3);
        ylabel('Speed (km/h)'); grid on; legend('Actual V_x','Curvature preview cap','Location','best');
        title([course ' | historical PMPC longitudinal coordination']);
        nexttile; plot(ts.Time,values(:,9:10),'LineWidth',1.3);
        xlabel('Time (s)'); ylabel('Braking force (N)'); grid on;
        legend('Coordinator request','Total allocated braking (including DB)','Location','best');
        export_(f,figDir,[course '_PMPC_longitudinal']);
        fprintf('%s PMPC: peak requested longitudinal braking %.1f N\n',course,max(values(:,9)));
    end
    f=figure_([100 100 900 500]); hold on;
    for j=1:3, plot(S{j}.station_m,S{j}.vx_kmh,'Color',colors(j,:), ...
            'LineWidth',1.5,'DisplayName',labels{j}); end
    grid on; xlabel('Reference station (m)'); ylabel('V_x (km/h)');
    xlim([sLow sHigh]); legend('Location','best'); export_(f,figDir,[course '_Vx']);
    for b=1:numel(segments)
        assert(bounds(b,2)>bounds(b,1),'Runs do not reach comparison segment.');
        metrics=zeros(3,4); f=figure_([100 100 720 570]); hold on;
        for j=1:3
            s=S{j}; mask=s.station_m>=bounds(b,1) & s.station_m<=bounds(b,2);
            % Equal-distance sampling makes RMS independent of residence time.
            gridS=(ceil(bounds(b,1)*10):floor(bounds(b,2)*10))'/10;
            [us,idx]=unique(s.station_m); e=interp1(us,s.ey_m(idx),gridS);
            v=interp1(us,s.vx_kmh(idx),gridS);
            metrics(j,:)=[max(abs(s.ey_m(mask))),min(s.vx_kmh(mask)), ...
                sqrt(mean(s.ey_m(mask).^2)),sqrt(mean(s.vx_kmh(mask).^2))];
            rows(end+1,:)={course,segments{b},labels{j},bounds(b,1),bounds(b,2), ...
                metrics(j,1),metrics(j,3),metrics(j,2),metrics(j,4),max(abs(s.ltr(mask))), ...
                sqrt(mean(e.^2)),sqrt(mean(v.^2))}; %#ok<AGROW>
            plot(s.beta_deg(mask),s.yaw_rate_deg_s(mask),'Color',colors(j,:), ...
                'LineWidth',1.3,'DisplayName',labels{j});
        end
        grid on; xlabel('\beta (deg)'); ylabel('r (deg/s)');
        title([course ' / ' segments{b}]); legend('Location','best');
        export_(f,figDir,[course '_' segments{b} '_beta_r']);
        % Four axes: ratios to the best observed value; raw metrics in CSV.
        score=zeros(3,4);
        score(:,[1 3])=min(metrics(:,[1 3]),[],1)./max(metrics(:,[1 3]),eps);
        score(:,[2 4])=metrics(:,[2 4])./max(metrics(:,[2 4]),[],1);
        f=figure_([100 100 780 620]); hold on; axis equal off;
        xy=[0 1;1 0;0 -1;-1 0;0 1];
        for r=[.25 .5 .75 1]
            plot(r*xy(:,1),r*xy(:,2),'-','Color',[.8 .8 .8],'HandleVisibility','off');
            text(.035,r,sprintf('%.2g',r),'Color',[.45 .45 .45],'FontSize',9);
        end
        plot([-1 1],[0 0],'-','Color',[.5 .5 .5],'HandleVisibility','off');
        plot([0 0],[-1 1],'-','Color',[.5 .5 .5],'HandleVisibility','off');
        for j=1:3
            z=score(j,[1:4 1])'; plot(z.*xy(:,1),z.*xy(:,2),'-o', ...
                'Color',colors(j,:),'LineWidth',1.5,'DisplayName',labels{j});
        end
        text(0,1.14,'e_{y,max}: smaller','HorizontalAlignment','center');
        text(1.08,0,'V_{x,min}: larger','HorizontalAlignment','left');
        text(0,-1.14,'e_{y,RMS}: smaller','HorizontalAlignment','center');
        text(-1.08,0,'V_{x,RMS}: larger','HorizontalAlignment','right');
        xlim([-1.8 1.8]); ylim([-1.4 1.4]); legend('Location','southoutside','Orientation','horizontal');
        title({[course ' / ' segments{b}],'Normalized ratios; speed retention is not a safety score'});
        export_(f,figDir,[course '_' segments{b} '_diamond']);
    end
end
summary=cell2table(rows,'VariableNames',{'course','segment','controller', ...
    'station_start_m','station_end_m','ey_peak_m','ey_rms_m','vx_min_kmh','vx_rms_kmh', ...
    'ltr_peak','ey_rms_space_m','vx_rms_space_kmh'});
writetable(summary,fullfile(outputRoot,'metrics_common_station.csv'));
save(fullfile(outputRoot,'metrics_common_station.mat'),'summary'); disp(summary);
end

function [d,station]=project_(W,D)
P=W(:,2:3); T=diff(P); L2=sum(T.^2,2); valid=L2>eps;
P0=P(1:end-1,:); P0=P0(valid,:); T=T(valid,:); L2=L2(valid);
s0=W(1:end-1,7); s0=s0(valid);
if isfield(D,'Xcg_TM'), x=D.Xcg_TM; y=D.Ycg_TM;
else, x=D.Xo-1.2466*cosd(D.Yaw); y=D.Yo-1.2466*sind(D.Yaw); end
d=zeros(size(x)); station=d;
for k=1:numel(x)
    t=min(max(((x(k)-P0(:,1)).*T(:,1)+(y(k)-P0(:,2)).*T(:,2))./L2,0),1);
    dx=x(k)-P0(:,1)-t.*T(:,1); dy=y(k)-P0(:,2)-t.*T(:,2);
    [~,j]=min(dx.^2+dy.^2); d(k)=(T(j,1)*dy(j)-T(j,2)*dx(j))/sqrt(L2(j));
    station(k)=s0(j)+t(j)*sqrt(L2(j));
end
end
function f=figure_(pos)
f=figure('Visible','off','Color','w','Position',pos);
end
function export_(f,dir,name)
exportgraphics(f,fullfile(dir,[name '.png']),'Resolution',300);
exportgraphics(f,fullfile(dir,[name '.pdf']),'ContentType','vector');
savefig(f,fullfile(dir,[name '.fig'])); close(f);
end
