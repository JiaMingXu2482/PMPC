function [Fhi,Flo,F_at0,F_at16] = force_bounds_four( ...
    u,t,Temp_C,I_L1,I_L2,I_R1,I_R2,v_pred,params) %#codegen
%FORCE_BOUNDS_FOUR 四角边界统一入口；Simulink不再拆预测速度或拼接力界。
% u：CarSim 54路原始输出，u(1:4)为压缩速度[mm/s]，u(51:54)为位移[mm]。
% 四角顺序：[L1;L2;R1;R2]。温度为四角共用标量[degC]。
% I_L1/I_L2/I_R1/I_R2必须接原有1 ms历史寄存后的命令[A]，不是本拍逆模型输出。
% v_pred：4xNp [mm/s]；内部逐行取出各角速度，不修改实测历史输入。
% Fhi/Flo：4x(Np+1) [N]，第1列当前界，其余各列为预测界。
% F_at0/F_at16：4x1 [N]，当前两端电流对应力，保留电流端点身份。
% 每1 ms调用，底层函数每10 ms更新边界；不要同时运行原四个边界调用块。
N=coder.const(size(v_pred,2));
Fhi=zeros(4,N+1); Flo=zeros(4,N+1);
F_at0=zeros(4,1); F_at16=zeros(4,1);
I_hist=[I_L1;I_L2;I_R1;I_R2];
for cid=1:4
    [lo,hi,f0,f16]=force_bounds_impl_10ms( ...
        t,Temp_C,u(50+cid),u(cid),I_hist(cid),cid,v_pred(cid,:).', ...
        params.M.preview_alpha,params.M.bounds_keep);
    Flo(cid,:)=lo.';
    Fhi(cid,:)=hi.';
    F_at0(cid)=f0;
    F_at16(cid)=f16;
end
end
