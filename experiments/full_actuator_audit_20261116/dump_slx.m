function dump_slx()
%DUMP_SLX 冻结实际对象：回调、哈希、减振器链路块参数（只读）。
m = 'pmpc_mil';
if ~bdIsLoaded(m), load_system(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))),'pmpc_mil.slx')); end
fid = fopen(fullfile(fileparts(mfilename('fullpath')),'dump.txt'),'w');
P = @(varargin) fprintf(fid, varargin{:});
P('== 模型回调 ==\n');
for cb = {'PreLoadFcn','PostLoadFcn','InitFcn','StartFcn','StopFcn'}
    P('%s: %s\n', cb{1}, strrep(get_param(m,cb{1}), newline(), ' | '));
end
P('\n== 顶层块 ==\n');
top = find_system(m,'SearchDepth',1,'Type','Block');
for i=1:numel(top)
    P('  %s  [%s]\n', strrep(top{i},[m '/'],''), get_param(top{i},'BlockType'));
end
P('\n== 含 PI / FIR / StateSpace / UnitDelay 的子系统 ==\n');
subs = find_system(m,'LookUnderMasks','all','FollowLinks','on','MatchFilter',@Simulink.match.allVariants,'Type','Block');
for i=1:numel(subs)
    bt = get_param(subs{i},'BlockType');
    if any(strcmp(bt,{'Reference','DiscreteFir','DiscreteStateSpace','UnitDelay','DiscretePIDController'}))
        nm = get_param(subs{i},'Name');
        if any(strcmp(bt,{'DiscreteFir','DiscreteStateSpace','UnitDelay','DiscretePIDController'}))
            P('  [%s] %s   (parent=%s)\n', bt, nm, get_param(get_param(subs{i},'Parent'),'Name'));
        end
    end
end
fclose(fid);
type(fullfile(fileparts(mfilename('fullpath')),'dump.txt'));
end
