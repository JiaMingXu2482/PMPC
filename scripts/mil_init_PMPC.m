function P = mil_init_PMPC()
%MIL_INIT_PMPC  Prepare the eight-state, four-input PMPC model.
P = mil_init_variant(3, 'PMPC_P', 'pmpc_block', 'pmpc_mil');
end
