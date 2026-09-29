function cg_block
%CG_BLOCK  对 pmpc_block 跑 codegen (参数作为编译期常量)
if evalin('base','exist(''PMPC_P'',''var'')')
    P = evalin('base','PMPC_P');
else
    P = setup_pmpc();
end
u_example = zeros(54,1);
cfg = coder.config('lib'); cfg.GenerateReport = false;
try
    codegen('pmpc_block','-config',cfg,'-d','C:\Users\user\AppData\Local\Temp\cg_blk', ...
            '-args',{u_example, coder.Constant(P)},'-nargout',1);
    disp('>>> pmpc_block 编译通过 <<<');
catch e
    fprintf('MSG: %s\n', e.message);
end
end
