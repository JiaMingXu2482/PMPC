function [cmpRD_preview,prediction]=pmpc_preview(meas,Reftraj,params,state,delta_d) %#codegen
%PMPC_PREVIEW 100 Hz：仅用当前测量和上一拍计划，生成4xNp预测压缩速度 [mm/s]。
% 无 Flo/Fhi/Fdes 输入，无持久状态；输出送独立边界模型，不产生本拍QP反馈环。
% prediction输出本拍预测矩阵，经独立信号直接送QP块；QP不再重复线性化/ZOH预测。
% delta_d 为驾驶员前轮转角，u(1)=Delta delta 为 AFS 附加角；总转角=delta_d+u(1)。
if nargin<5, delta_d=0; end
% FishHook：驾驶员转角取实测前轮角（CarSim 开环转动机器人给出）。
if isfield(params.M,'steer_ext') && params.M.steer_ext
    delta_d = mean(meas.delta_front(:));
end
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
% 预瞄点 = 该节点区间起点 + preview_L；用 [0;tcum] 与 [meas.p;pr] 分段线性插值。
% 注意节点 0 用当前实测 meas.p，否则 L 落在第一个区间内时会错误地取到 pr(1)。
tcum=cumsum(c.dtg);
for j=1:N
    tp = tcum(j) - c.dtg(j) + params.M.preview_L;
    if tp <= 0
        pL = meas.p;
    elseif tp >= tcum(N)
        pL = pr(N);
    else
        m = 1;
        while m <= N && tcum(m) < tp
            m = m + 1;
        end
        if m > N, m = N; end
        if m > 1
            ta = tcum(m-1); va = pr(m-1);
        else
            ta = 0; va = meas.p;
        end
        tb = tcum(m); vb = pr(m);
        if tb <= ta
            a = 0;
        else
            a = (tp - ta)/(tb - ta);
        end
        pL = va + min(max(a,0),1)*(vb - va);
    end
    if isfinite(pL)
        % 保留当前实测的升沉/路面/俯仰成分，仅外推侧倾贡献。
        cmpRD_preview(:,j)=1000*(meas.vd+params.vehicle.kv_roll*(pL-meas.p));
    end
end
end
