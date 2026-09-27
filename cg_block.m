function cg_block
%CG_BLOCK  对 pmpc_block 跑 codegen (参数作为编译期常量)
A = load('C:\Users\user\AppData\Local\Temp\args_step.mat');
P = evalin('base','PMPC_P');
cfg = coder.config('lib'); cfg.GenerateReport = false;
try
    codegen('pmpc_block','-config',cfg,'-d','C:\Users\user\AppData\Local\Temp\cg_blk', ...
            '-args',{A.u_, coder.Constant(P)},'-nargout',1);
    disp('>>> pmpc_block 编译通过 <<<');
catch e
    fprintf('MSG: %s\n', e.message);
end
end
