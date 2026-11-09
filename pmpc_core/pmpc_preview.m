function [cmpRD_preview,prediction]=pmpc_preview(meas,Reftraj,params,state,delta_d) %#codegen
%PMPC_PREVIEW 100 Hz：仅用当前测量和上一拍计划，生成4xNp预测压缩速度 [mm/s]。
% 无 Flo/Fhi/Fdes 输入，无持久状态；输出送独立边界模型，不产生本拍QP反馈环。
% prediction输出本拍预测矩阵，经独立信号直接送QP块；QP不再重复线性化/ZOH预测。
% delta_d 为驾驶员前轮转角，u(1)=Delta delta 为 AFS 附加角；总转角=delta_d+u(1)。
if nargin<5, delta_d=0; end
N=coder.const((numel(state.active_main)-4*numel(state.plan)-10)/12);
nd=coder.const(numel(state.plan));
cmpRD_preview=repmat(1000*meas.vd,1,N);
prediction=struct('Y',zeros(7*N,nd),'y',zeros(7*N,1), ...
    'AI',zeros(nd,nd),'Ut',zeros(nd,1),'valid',false);
if meas.Vx<=0 || any(~isfinite([meas.X;meas.Y;meas.Vx;meas.Vy; ...
        meas.yaw;meas.r;meas.phi;meas.p;meas.vd;meas.delta_front; ...
        meas.steering_wheel;meas.Fx;meas.Fy;meas.Fz;meas.Fd;meas.Td; ...
        state.u_prev;state.plan]))
    return
end
% 零力界仅满足工作点接口；预测矩阵不读取 Flo/Fhi、MdloPlan 或 duMax。
[c,~,valid]=prepare_step(meas,Reftraj,params,zeros(4,1),zeros(4,1),state,delta_d);
if ~valid, return; end
p=build_prediction(c,params,state.plan);
prediction.Y=p.Y; prediction.y=p.y;
prediction.AI=p.AI; prediction.Ut=p.Ut;
prediction.valid=all(isfinite(p.Y(:))) && all(isfinite(p.y));
if ~prediction.valid, return; end
pr=p.y(4:7:end)+p.Y(4:7:end,:)*state.plan;
a=min(max(params.M.preview_L/params.M.Ts,0),1);
for j=1:N
    if j==1, p0=meas.p; else, p0=pr(j-1); end
    pL=p0+a*(pr(j)-p0); % 第j个命令在 (j-1)*Ts+L 时刻的侧倾角速度
    if isfinite(pL)
        % 保留当前实测的升沉/路面/俯仰成分，仅外推侧倾贡献。
        cmpRD_preview(:,j)=1000*(meas.vd+params.vehicle.kv_roll*(pL-meas.p));
    end
end
end
