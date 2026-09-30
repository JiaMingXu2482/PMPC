function P = mil_init_MPC()
%MIL_INIT_MPC  Prepare the fixed six-state MPC model for manual simulation.
P = mil_init_variant(1, 'MPC_P', 'mpc_block', 'mpc_mil');
end
