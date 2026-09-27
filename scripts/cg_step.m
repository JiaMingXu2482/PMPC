function cg_step(asvar)
%CG_STEP  对 pmpc_step 跑 codegen
%   cg_step        参数 Pm 作为编译期常量 (coder.Constant) —— HIL 的目标形态
%   cg_step(1)     参数作为普通变量 (对照用; 会有大量变尺寸问题)
%
%   为什么默认用 coder.Constant: nvars = Nu*Nc+Ne+Nr 这类维度全来自 Pm 的字段,
%   只有把 Pm 折成常量, 这些维度才是编译期常量, 数组尺寸才能固定。
%   HIL 上参数本来就是 build 时固化的, 与目标形态一致。
if nargin < 1, asvar = 0; end
A = load('C:\Users\user\AppData\Local\Temp\args_step.mat');
cfg = coder.config('lib'); cfg.GenerateReport = false;
d = 'C:\Users\user\AppData\Local\Temp\cg_pmpc_step';
if asvar, Pspec = A.P_; else, Pspec = coder.Constant(A.P_); end
try
    codegen('pmpc_step','-config',cfg,'-d',d,'-args',{A.u_, Pspec, A.S_},'-nargout',2);
    disp('>>> pmpc_step 编译通过 <<<');
catch e
    fprintf('IDENT: %s\n', e.identifier);
    m = e.message; fprintf('MSG: %s\n', m);
end
end
