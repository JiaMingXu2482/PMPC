function P = mil_init_PMPC()
%MIL_INIT_PMPC  Prepare the fixed seven-state PMPC model for manual simulation.
P = mil_init_variant(3, 'PMPC_P', 'pmpc_block', 'pmpc_mil');
end
