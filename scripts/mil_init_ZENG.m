function P = mil_init_ZENG()
%MIL_INIT_ZENG  Prepare the fixed seven-state ZENG model for manual simulation.
P = mil_init_variant(2, 'ZENG_P', 'zeng_block', 'zeng_mil');
end
