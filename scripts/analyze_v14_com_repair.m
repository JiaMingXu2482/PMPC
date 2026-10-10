function report = analyze_v14_com_repair(rawTag, repairedTag, outdir)
%ANALYZE_V14_COM_REPAIR Compare two saved COM runs and export diagnostics.
startup_pmpc();
if ~isfolder(outdir), mkdir(outdir); end
D={func_ReadERD(rawTag),func_ReadERD(repairedTag)};
tags={rawTag,repairedTag};
assert(strcmp(D{1}.Dataset,D{2}.Dataset),'Dataset mismatch.');
report=struct();
labels={'raw','repaired'};
for k=1:2
    d=D{k};
    f=[d.Fd_L1 d.Fd_L2 d.Fd_R1 d.Fd_R2];
    v=[d.CmpRD_L1 d.CmpRD_L2 d.CmpRD_R1 d.CmpRD_R2];
    z=[d.Jnc_L1 d.Jnc_L2 d.Jnc_R1 d.Jnc_R2];
    power=f.*v;
    s=struct('force_rms_N',sqrt(mean(f.^2,'all')), ...
        'velocity_rms_mm_s',sqrt(mean(v.^2,'all')), ...
        'jounce_absmax_mm',max(abs(z),[],'all'), ...
        'negative_power_fraction',mean(power < -5000,'all'), ...
        'ltr_absmax',max(abs(d.LTR)), ...
        'speed_min_kmh',min(d.Vx),'speed_max_kmh',max(d.Vx));
    fprintf('%s: force RMS %.1f N, stroke velocity RMS %.1f mm/s, jounce max %.1f mm, LTR %.3f, negative power %.2f%%\n', ...
        labels{k},s.force_rms_N,s.velocity_rms_mm_s,s.jounce_absmax_mm, ...
        s.ltr_absmax,100*s.negative_power_fraction);
    s.tracking=func_EyConsoleSummary(d);
    logs=load(fullfile(fileparts(tags{k}),'pmpc_mil_logs.mat'));
    current=logs.logsout.getElement('damper_i_cmd').Values.Data;
    s.current_step_rms_A=sqrt(mean(diff(current).^2,'all'));
    fprintf('%s current step RMS %.3f A\n',labels{k},s.current_step_rms_A);
    if any(strcmp(logs.logsout.getElementNames,'damper_force_raw'))
        applied=logs.logsout.getElement('damper_force_applied').Values;
        raw=logs.logsout.getElement('damper_force_raw').Values;
        assert(isequal(applied.Time,raw.Time),'Force log time mismatch.');
        s.guard_changed_fraction=mean(abs(applied.Data-raw.Data)>1,'all');
        s.raw_force_rms_N=sqrt(mean(raw.Data.^2,'all'));
        fprintf('%s guard changes >1 N: %.2f%%; raw/applied logging interval %.4f s\n', ...
            labels{k},100*s.guard_changed_fraction,median(diff(raw.Time)));
    end
    report.(labels{k})=s;
end
fig=figure('Visible','off','Color','w','Position',[100 100 1150 780]);
layout=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for k=1:2
    d=D{k};
    nexttile(layout,k); plot(d.t,d.Vx,'LineWidth',1.1); ylim([75 115]);
    ylabel('V_x (km/h)'); title(labels{k}); grid on
    nexttile(layout,2+k); plot(d.t,[d.Jnc_L1 d.Jnc_L2 d.Jnc_R1 d.Jnc_R2]);
    ylim([-90 150]); ylabel('Wheel jounce (mm)'); grid on
    nexttile(layout,4+k); plot(d.t,[d.Fd_L1 d.Fd_L2 d.Fd_R1 d.Fd_R2]);
    ylim([-4200 4200]); ylabel('Damper force (N)'); xlabel('Time (s)'); grid on
end
set(findall(fig,'Type','axes'),'FontName','Times New Roman','FontSize',11);
exportgraphics(fig,fullfile(outdir,'v14_COM_before_after.png'),'Resolution',180);
savefig(fig,fullfile(outdir,'v14_COM_before_after.fig'));
close(fig);
save(fullfile(outdir,'v14_COM_repair_metrics.mat'),'report');
end
