function manifest=prepare_legacy_longcoord_replay(scenario)
% Prepare identical roads for an isolated historical-controller replay.
if nargin==0, scenario='current_R75'; end
root=startup_pmpc();
legacy=fullfile(root,'scratchpad','legacy_longcoord_4fb1637');
assert(isfolder(legacy),'Extract commit 4fb1637 before preparing this replay.');
data='D:\Program Files\CarSim2019.0\CarSim2019.0_Data';
slalom=fullfile(data,'Results','Run_83faa924-8dd2-4f4e-bb27-1e4aa42811f7','Run_all.par');
switch scenario
    case 'current_R75'
        out=fullfile(root,'simulation_results','legacy_longcoord_20261009');
        com=fullfile(root,'scratchpad','com_r75_db_20261008','PMPC_before','Run_all.par');
        courses={'COM','Slalom'}; sources={com,slalom};
        turnX=330; radius=75;
    case 'historical_R70'
        out=fullfile(root,'simulation_results','legacy_longcoord_original_R70_20261009');
        com=fullfile(root,'simulation_results','combined','Results', ...
            'Run_Combined90_R70_PMPC','Run_all.par');
        courses={'COM'}; sources={com};
        turnX=280; radius=70;
    case 'historical_manual_R70'
        out=fullfile(root,'simulation_results','legacy_longcoord_manual_R70_20261009');
        com=fullfile(root,'simulation_results','archive','results_pmpc_comb_fix', ...
            '20261003_092605','LastRun_all.par');
        courses={'COM'}; sources={com};
        turnX=280; radius=70;
    otherwise
        error('Unknown replay scenario: %s',scenario);
end
assert(~isfolder(out),'Replay folder already exists.'); mkdir(out);
sf=fileread(fullfile(root,'simfile.sim'));
models={'mpc_mil','zeng_mil','pmpc_mil'};
manifest=struct('commit','4fb1637','legacy_root',legacy,'output_root',out, ...
    'models',{models},'courses',{courses},'scenario',scenario, ...
    'turn_x',turnX,'radius',radius,'prepared_at',datestr(now,30));
for c=1:numel(courses)
    par=fileread(sources{c});
    if c==1
        sourceName=sprintf('COMB90_DLC05_JT_R%g_mu0.5to0.85_PMPC',radius);
        mv=func_ManeuverFromRun(sourceName,sources{c});
        assert(mv.R==radius,'Unexpected COM radius.');
        if isfield(mv,'turn_x'), assert(mv.turn_x==turnX,'Unexpected COM turn start.'); end
        if strcmp(scenario,'historical_R70')
            assert(contains(par,'SSTOP 470') && ~contains(par,'SEGMENT_RADIUS'), ...
                'Historical COM source is not the saved generated R70 road.');
        elseif strcmp(scenario,'historical_manual_R70')
            assert(contains(par,'SEGMENT_LENGTH 280') && ...
                contains(par,'SEGMENT_RADIUS 70.000000'), ...
                'Historical COM source is not the saved manual R70 road.');
        end
        W=func_WayPoints(6,radius,false,turnX);
        baseName=erase(sourceName,'_PMPC');
    else
        W=func_WayPoints(2,NaN,false);
        baseName='Slalom70_mu0.85';
        assert(contains(par,'MU_ROAD_CONSTANT 0.85'),'Unexpected Slalom friction.');
    end
    save(fullfile(out,[courses{c} '_reference.mat']),'W');
    for j=1:3
        model=models{j}; tag=upper(erase(model,'_mil'));
        id=['Run_' courses{c} '_' tag];
        runDir=fullfile(out,'Results',id); mkdir(runDir);
        text=par;
        old=regexp(text,'FullDataName CarSim Run Control`([^`]+)`','tokens','once');
        name=[baseName '_' tag]; text=strrep(text,old{1},name);
        old=regexp(text,'(?m)^SIMULINK_MODEL_FILE[^\r\n]+','match','once');
        text=strrep(text,old,['SIMULINK_MODEL_FILE ' fullfile(legacy,[model '.slx'])]);
        if c==2
            text=regexprep(text,'(?m)^SSTOP[ \t]+[^\r\n]+','SSTOP 160');
        end
        write_(fullfile(runDir,'Run_all.par'),text);
        sim=sf;
        keys={'ROOT_FILE_NAME','WORK_DIR','OUTPUT_PATH','OUTPUT_FILE_PREFIX'};
        values={id,[out filesep],fullfile(out,'Results'),fullfile(runDir,'LastRun')};
        for k=1:numel(keys)
            old=regexp(sim,['(?m)^SET_MACRO \$\(' keys{k} '\)\$[^\r\n]+'],'match','once');
            sim=strrep(sim,old,['SET_MACRO $(' keys{k} ')$ ' values{k}]);
        end
        write_(fullfile(out,[id '.sim']),sim);
    end
end
save(fullfile(out,'manifest.mat'),'manifest');
disp(manifest);
end

function write_(path,text)
f=fopen(path,'w'); assert(f>=0); cleanup=onCleanup(@()fclose(f));
fwrite(f,text,'char');
end
