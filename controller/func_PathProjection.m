function Proj = func_PathProjection(VehiclePara, WayPoints_Index, W, VehStateMeasured)
%FUNC_PATHPROJECTION Project the vehicle CG onto a station-ordered path.
% Keep the historical spline projection and nearest-index arithmetic intact.

psi = VehStateMeasured.Yaw;
x = VehStateMeasured.X - VehiclePara.lf*cos(psi);
y = VehStateMeasured.Y - VehiclePara.lf*sin(psi);
vel = VehStateMeasured.x_dot;
prj = struct('ey',0,'epsi',0,'Velr',vel,'xr',x,'yr',y,'psir',psi);
Proj = struct('WPIndex',-1,'PrjP',prj,'s0',0,'valid',false);

n = size(W,1);
if n < 6 || size(W,2) < 7 || ~all(isfinite([x,y,psi,vel]))
    return;
end
for j = 1:n
    if ~all(isfinite(W(j,2:5))) || ~isfinite(W(j,7))
        return;
    end
    if j > 1 && W(j,7) <= W(j-1,7)
        return;
    end
end

distMin = 1000;
indexMin = 0;
startIndex = max(1,min(WayPoints_Index,n));
for i = startIndex:n
    dx = W(i,2)-x;
    dy = W(i,3)-y;
    dist = sqrt(dx.^2+dy.^2);
    if dist < distMin
        distMin = dist;
        indexMin = i;
    end
end
if indexMin < 1
    return;
end
if indexMin >= n
    Proj.WPIndex = -2;
    return;
end

Proj.WPIndex = indexMin;
[px,py,heading,ey,epsi] = local_error(W,x,y,psi);
Proj.PrjP.ey = -ey;
Proj.PrjP.epsi = -epsi;
Proj.PrjP.Velr = vel;
Proj.PrjP.xr = px;
Proj.PrjP.yr = py;
Proj.PrjP.psir = heading;
Proj.s0 = local_arcpos(W,indexMin,x,y);
Proj.valid = isfinite(Proj.s0) && isfinite(px) && isfinite(py) ...
    && isfinite(ey) && isfinite(epsi);
end

function [px,py,heading,ey,epsi] = local_error(W,x,y,psi)
interpPoints = 1500;
windowSize = 15;
Xref = W(:,2);
Yref = W(:,3);
psiRef = W(:,4);
dist = sqrt((Xref-x).^2+(Yref-y).^2);
[~,nearest_] = min(dist(:));
nearest = nearest_(1);
startIdx = max(1,nearest-floor(windowSize/2));
endIdx = min(length(Xref),startIdx+windowSize);
if endIdx-startIdx < 5
    startIdx = max(1,endIdx-5);
end
localX = Xref(startIdx:endIdx);
localY = Yref(startIdx:endIdx);
localPsi = psiRef(startIdx:endIdx);
interpIdx = linspace(1,length(localX),interpPoints);
Xi = spline(1:length(localX),localX,interpIdx);
Yi = spline(1:length(localY),localY,interpIdx);
Psii = spline(1:length(localPsi),localPsi,interpIdx);
interpDist = sqrt((Xi-x).^2+(Yi-y).^2);
[~,idx_] = min(interpDist(:));
idx = idx_(1);
px = Xi(idx);
py = Yi(idx);
dx = px-x;
dy = py-y;
heading = atan2(dy,dx);
ey = -dx*sin(psi)+dy*cos(psi);
epsi = local_wrapToPi(Psii(idx)-psi);
end

function angle = local_wrapToPi(angle)
for k = 1:numel(angle)
    a = angle(k);
    if a < -pi || a > pi
        angle(k) = mod(a+pi,2*pi)-pi;
    end
end
end

function s = local_arcpos(W,idx,x,y)
n = size(W,1);
s = W(idx,7);
dmin = inf;
for j = max(idx-1,1):min(idx,n-1)
    tx = W(j+1,2)-W(j,2);
    ty = W(j+1,3)-W(j,3);
    len2 = tx*tx+ty*ty;
    if len2 > 0
        u = ((x-W(j,2))*tx+(y-W(j,3))*ty)/len2;
        if j > 1, u = max(u,0); end
        if j < n-1, u = min(u,1); end
        px = W(j,2)+u*tx;
        py = W(j,3)+u*ty;
        d = (x-px)*(x-px)+(y-py)*(y-py);
        if d < dmin
            dmin = d;
            s = W(j,7)+u*(W(j+1,7)-W(j,7));
        end
    end
end
end
