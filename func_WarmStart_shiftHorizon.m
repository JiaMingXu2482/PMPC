function new_WarmStart = func_WarmStart_shiftHorizon(U,MPCParameters)
    % shiftHorizon 平移控制序列，为下一次MPC优化提供热启动值
    % 输入:
    %   U - 当前优化得到的控制序列 (长度为控制时域Nc)
    % 输出:
    %   new_WarmStart - 平移后的控制序列，作为下一次优化的初始值
    Nc  = MPCParameters.Nc;
    Nu  = MPCParameters.Nu;
    % 获取控制序列长度（控制时域）

    
    if Nc <= 1
        % 若控制时域为1，直接返回零向量
        new_WarmStart = zeros(size(U));
    else
        % 平移一个控制步，每步包含 Nu 个控制增量。
        new_WarmStart = [U(Nu+1:Nc*Nu); zeros(Nu,1); U(Nc*Nu+1:end)];
    end
end
