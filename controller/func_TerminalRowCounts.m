function [n1,n2] = func_TerminalRowCounts(M) %#codegen
% Fixed row counts; no extra online decision variables.
n1=0; n2=0;
if isfield(M,'terminal_on') && M.terminal_on && M.ControllerVariant==3 && M.abl~=3
    assert(M.terminal_steps>=1 && M.terminal_steps==floor(M.terminal_steps));
    assert(M.Nx==7 && M.Nu==3);
    if isfield(M,'terminal_mode') && M.terminal_mode==2
        n1=6*M.terminal_steps;
        n2=4*M.terminal_steps;
    else
        n1=18*M.terminal_steps+2*(M.Nx+M.Nu);
        n2=16*M.terminal_steps+2*(M.Nx+M.Nu);
    end
end
end
