function outFile = plot_nlcsnn_chatter_fix(outFile)
%PLOT_NLCSNN_CHATTER_FIX Compare late-DLC damper velocity before/after fix.
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(outFile)
    outFile = fullfile(root,'scratchpad','dlc_nlcsnn_chatter_fix.png');
end
addpath(fullfile(fileparts(mfilename('fullpath')),'lib'));
A=func_ReadERD(fullfile(root,'scratchpad','DLC_NLCSNN_active'));
F=func_ReadERD(fullfile(root,'scratchpad','DLC_NLCSNN_final'));
P=func_ReadERD(fullfile(root,'scratchpad','DLC_passive_noSAS'));
corners={'L1','R1','L2','R2'};
fig=figure('Visible','off','Color','w','Position',[100 100 1200 720]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
for k=1:4
    nexttile;
    f=['CmpRD_' corners{k}];
    plot(A.t,A.(f),'Color',[0.85 0.25 0.20],'LineWidth',0.8); hold on;
    plot(F.t,F.(f),'Color',[0.05 0.35 0.90],'LineWidth',1.1);
    plot(P.t,P.(f),'Color',[0.35 0.35 0.35],'LineWidth',0.8);
    xlim([5.0 7.8]); grid on;
    xlabel('Time (s)'); ylabel('CmpRD (mm/s)');
    title(['Damper ' corners{k}]);
    if k==1
        legend('Original NLCSNN','Fixed NLCSNN (accel filter)', ...
            'Passive baseline','Location','best');
    end
end
sgtitle('DLC late-stage wheel-hop comparison (8-30 Hz chatter region)');
exportgraphics(fig,outFile,'Resolution',180);
close(fig);
fprintf('Saved %s\n',outFile);
end
