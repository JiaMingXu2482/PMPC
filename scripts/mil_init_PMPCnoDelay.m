function P = mil_init_PMPCnoDelay()
%MIL_INIT_PMPCNODELAY  Prepare the fixed six-state PMPC ablation model.
P = mil_init_variant(4, 'PMPC_NODELAY_P', ...
    'pmpc_nodelay_block', 'pmpc_nodelay_mil');
end
