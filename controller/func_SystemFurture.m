function [PSI,THETA,GAMMA,PHI] = func_SystemFurture(MPCParameters, StateSpaceModel, Bezier_SK, dist)
% func_SystemFurture
% -------------------------------------------------------------------------
% 构建 MPC 预测矩阵（支持时变系统版本）
% 预测方程：
%   Y = PSI*zeta0 + THETA*DeltaU + PHI*GAMMA
%
% 其中 StateSpaceModel.A_aug/B_aug/D_aug 均为 {Np x 1} cell（可时变）
% -------------------------------------------------------------------------

Nu = MPCParameters.Nu;
Nc = MPCParameters.Nc;
Np = MPCParameters.Np;
Ny = MPCParameters.Ny;

A_aug_cell = StateSpaceModel.A_aug;
B_aug_cell = StateSpaceModel.B_aug;
D_aug_cell = StateSpaceModel.D_aug;
C_aug      = StateSpaceModel.C_aug;

%% -------- codegen: 原来用 cell + cell2mat, 改成直接写进预分配的大矩阵 --------
%  各块尺寸本来就是固定的, cell2mat 的拼装方式等价于按块索引写入:
%    PSI  : (Ny*Np) x na      第 p 块行 = (p-1)*Ny+(1:Ny)
%    THETA: (Ny*Np) x (Nu*Nc) 第 (p,k) 块
%    PHI  : (Ny*Np) x (nd*Np) 第 (p,j) 块
na = size(A_aug_cell,1);
nd = size(D_aug_cell,2);
PSI   = zeros(Ny*Np, na);
THETA = zeros(Ny*Np, Nu*Nc);
PHI   = zeros(Ny*Np, nd*Np);

% -------- 先构建“从 0 到 p 的状态转移累乘” --------
Aprod = zeros(na, na, Np);
for p = 1:Np
    if p == 1
        Aprod(:,:,p) = A_aug_cell(:,:,p);
    else
        Aprod(:,:,p) = A_aug_cell(:,:,p) * Aprod(:,:,p-1);
    end
end

for p = 1:Np
    rp = (p-1)*Ny + (1:Ny);
    PSI(rp,:) = C_aug * Aprod(:,:,p);

    for k = 1:Nc
        ck = (k-1)*Nu + (1:Nu);
        if k <= p
            A_pk = eye(na);
            for jj = (k+1):p
                A_pk = A_aug_cell(:,:,jj) * A_pk;
            end
            THETA(rp,ck) = C_aug * A_pk * B_aug_cell(:,:,k);
        end
    end

    for j = 1:Np
        cj = (j-1)*nd + (1:nd);
        if j <= p
            A_pj = eye(na);
            for jj = (j+1):p
                A_pj = A_aug_cell(:,:,jj) * A_pj;
            end
            PHI(rp,cj) = C_aug * A_pj * D_aug_cell(:,:,j);
        end
    end
end

% -------- 扰动向量 GAMMA --------
%  D 为 6x3: [kappa, F_off_f_eff, F_off_r]。PHI 按 [d(1);...;d(Np)] 分块，
%  故 GAMMA 交错排列 [kap(1); off_f; off_r; kap(2); off_f; off_r; ...]。
%  两个偏置在预测时域内按常值保持（可测扰动）。
if nargin < 4 || isempty(dist), dist = [0;0]; end
if isempty(Bezier_SK)
    kap = zeros(Np,1);
else
    Bezier_SK = Bezier_SK(:);
    if length(Bezier_SK) >= Np
        kap = Bezier_SK(1:Np);
    else
        kap = [Bezier_SK; zeros(Np-length(Bezier_SK),1)];
    end
end
%  codegen: nd 上面已算过(三维数组的第 2 维), 这里不再重算
if nd >= 3
    %  dist 可以是 2x1(方案二: 时域内常值) 或 2xNp(方案三: 逐节点)。
    %  codegen: dist 原地改尺寸会让 coder 推成变尺寸, 改为写入定长缓冲
    dist_p = zeros(2, Np);
    nc_ = size(dist,2);
    for q_ = 1:Np
        if nc_ == 1
            dist_p(:,q_) = dist(1:2,1);
        elseif q_ <= nc_
            dist_p(:,q_) = dist(1:2,q_);
        else
            dist_p(:,q_) = dist(1:2,nc_);
        end
    end
    if nd == 5
        % 8x4 model: free longitudinal acceleration and affine correction.
        GAMMA = reshape([kap.'; dist_p(1,:); dist_p(2,:); ...
            repmat(dist(3,1),1,Np); ones(1,Np)], [], 1);
    else
        GAMMA = reshape([kap.'; dist_p(1,:); dist_p(2,:)], [], 1);
    end
else
    GAMMA = kap;
end

end
