"""TASK-009 修正版：统一输入语义 + 统一样本掩码。
时间定义（100 Hz 日志，dt=0.01 s）：
  Md_cmd[k] 在 [t_k, t_{k+1}) 上保持（ZOH）。
  预测 t_{k+n}（即 h = n*10 ms 之后）的实际 Md。
预测器：
  tau0   瞬时：等于最后一个施加的指令 Md_cmd[k+n-1]
  tau1   一阶 ZOH rollout：y0=Md_act[k]，输入序列 Md_cmd[k..k+n-1]，tau 按工况
  persist 保持起点测量：Md_act[k]
  alloc  一阶 rollout，但目标换成分配后 Md（Bd'*Fdes）
样本掩码：k 从 k0（t>=1 s）到 N-1-n，四个预测器与所有 h 共用同一集合。
"""
import numpy as np, scipy.io as sio, json
R='D:/OneDrive - mail.scut.edu.cn/Thesis/Code/Controller/MAIN/experiments/nonuniform_grid_20261115/results/'
CASES={'dlc75_nc_uni':(R+'fair_dlc75_nc_uni.mat',0.01),
       'sla80_nc_uni':(R+'fair_sla80_nc_uni.mat',0.20),
       'dlc75_c_nu'  :(R+'fair_dlc75_c_nu.mat',0.01),
       'sla80_c_nu'  :(R+'fair_sla80_c_nu.mat',0.20)}
BD=0.5*np.array([-1.1,-1.1,1.1,1.1])
HS=[10,20,50,100]; TAUS=[1e-4,0.01,0.05,0.20]
def rollout(u, y0, tau, dt):
    a=np.exp(-dt/tau); y=y0
    for i in range(len(u)): y = a*y + (1-a)*u[i]
    return y
out={}
for name,(f,tau) in CASES.items():
    d=sio.loadmat(f, squeeze_me=False); D=np.asarray(d['D'],float)
    t=np.ravel(d['t']); dt=float(np.mean(np.diff(t)))
    mdc=D[6]; mda=D[5]; alloc=(D[13:17,:].T)@BD
    k0=int(np.searchsorted(t,1.0)); N=len(t)
    res={}
    for h in HS:
        n=h//10
        kk=np.arange(k0, N-n)                 # 统一掩码
        ytrue=mda[kk+n]
        p={}
        p['tau0']=mdc[kk+n-1]
        p['persist']=mda[kk]
        for tt in TAUS:
            p[f'roll{tt:g}']=np.array([rollout(mdc[k:k+n], mda[k], tt, dt) for k in kk])
        p['alloc1']=np.array([rollout(alloc[k:k+n], mda[k], tau, dt) for k in kk])
        res[h]={k2:float(np.sqrt(np.mean((v-ytrue)**2))) for k2,v in p.items()}
        res[h]['n_samples']=int(len(kk))
    out[name]=res
for name in CASES:
    res=out[name]; tau=CASES[name][1]
    print(f"== {name}  (rollout tau={tau:.3f} s)  每 h 样本数 {res[10]['n_samples']}")
    keys=['tau0','persist',f'roll{1e-4:g}',f'roll0.01',f'roll0.05',f'roll0.2','alloc1']
    print('  h[ms]'+''.join(f'{k:>11}' for k in keys))
    for h in HS:
        print(f'  {h:5d}'+''.join(f'{res[h][k]:11.2f}' for k in keys))
json.dump(out,open('pred_compare2.json','w'),indent=2)
