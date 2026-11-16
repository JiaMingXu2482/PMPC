"""TASK-009 第3步（离线、100 Hz 日志）：同输入下 Md 预测误差比较。
四种预测器：
  tau0  : 瞬时到位  y_hat = Md_cmd(k)
  tau1  : 一阶 ZOH，用**随后记录的真实命令序列**做强迫响应（同输入，非在线可预知）
  persist: y_hat = Md_act(k)（未来保持起点测量）
  alloc : 用分配后 Md（Bd'*Fdes）作为目标
对齐：100 Hz，h = 1/2/5/10 个采样（10/20/50/100 ms）。
"""
import numpy as np, scipy.io as sio, json
R='D:/OneDrive - mail.scut.edu.cn/Thesis/Code/Controller/MAIN/experiments/nonuniform_grid_20261115/results/'
CASES={'dlc75_nc_uni':(R+'fair_dlc75_nc_uni.mat',0.01),
       'sla80_nc_uni':(R+'fair_sla80_nc_uni.mat',0.20),
       'dlc75_c_nu'  :(R+'fair_dlc75_c_nu.mat',0.01),
       'sla80_c_nu'  :(R+'fair_sla80_c_nu.mat',0.20)}
HS=[10,20,50,100]
def fopdt(u, y0, tau, dt):
    a=np.exp(-dt/tau); y=np.empty(len(u)); y[0]=y0
    for i in range(len(u)-1): y[i+1]=a*y[i]+(1-a)*u[i]
    return y
out={}
for name,(f,tau) in CASES.items():
    d=sio.loadmat(f, squeeze_me=False); D=np.asarray(D_ if False else d['D'],float) if False else np.asarray(d['D'],float)
    t=np.ravel(d['t']); dt=float(np.mean(np.diff(t)))
    mdc=D[6]; mda=D[5]
    i0=int(np.searchsorted(t,1.0))
    res={}
    for h in HS:
        n=h//10
        ytrue=mda[i0+n:]; ycmd=mdc[i0+n:]
        idx=np.arange(i0, len(t)-n)
        pred={}
        pred['tau0']=mdc[idx]
        pred['persist']=mda[idx]
        pred['tau1']=np.array([fopdt(mdc[k:k+n+1], mda[k], tau, dt)[-1] for k in idx])
        res[h]={k:float(np.sqrt(np.mean((v-ytrue)**2))) for k,v in pred.items()}
        res[h]['mae_tau1']=float(np.mean(np.abs(pred['tau1']-ytrue)))
    out[name]=res
    print(f'== {name}  tau={tau}  N={len(t)-i0}')
    print(f"{'h[ms]':>6}{'tau0':>9}{'tau1':>9}{'persist':>10}")
    for h in HS:
        r=res[h]; print(f"{h:6d}{r['tau0']:9.2f}{r['tau1']:9.2f}{r['persist']:10.2f}")
json.dump(out,open('pred_compare.json','w'),indent=2)
