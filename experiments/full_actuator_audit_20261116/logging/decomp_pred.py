"""分解式预测验证：整 Md 套 τ vs 只对电流可调部分套 τ。
数据：log_{case}.mat（1 kHz Fa/Fat0 + 100 Hz Md_cmd/Md_act）。
预测器（全部在 100 Hz 控制拍上，ZOH 输入）：
  P1 instant   : Md_cmd[k+n-1]
  P2 roll_tau  : 整 Md 一阶 rollout, 初值 Md_act[k]
  P3 persist   : Md_act[k]
  P4 decomp    : Md_at0[k] + rollout(Δmd_cmd, Δmd_act[k], tau)   ← 运动部分保持，只有 Δ 走一阶
  P5 decomp_or : Md_at0[k+n] + rollout(...)                      ← 用未来真实 Md_at0（上界参考，非在线可实现）
"""
import scipy.io as sio, numpy as np, json
BD=np.array([-0.55,-0.55,0.55,0.55])
def roll(u,y0,tau,dt):
    a=np.exp(-dt/tau); y=y0
    for x in u: y=a*y+(1-a)*x
    return y
def load(case):
    M=sio.loadmat(f'log_{case}.mat', squeeze_me=False); v=M['rec'][0,0]
    tb=np.asarray(v['LogIcmd1'][0,0]['time'],float).ravel()
    F0=np.asarray(v['LogFat0'][0,0]['values'],float)
    if F0.shape[0]==4: F0=F0.T
    Md0_1k=F0@BD
    D=np.asarray(M['D'],float); t100=np.asarray(M['t'],float).ravel()
    idx=np.clip(np.searchsorted(tb,t100,side='right')-1,0,len(tb)-1)
    Md0_100=Md0_1k[idx]
    return D,t100,Md0_100
out={}
for case,tau_case in [('dlc75',0.01),('sla80',0.20)]:
    D,t100,Md0=load(case)
    mdc=D[6]; mda=D[5]; dt=float(np.mean(np.diff(t100)))
    k0=int(np.searchsorted(t100,1.0)); N=len(t100)
    print(f'== {case} ==  (τ_case={tau_case}, τ_meas=0.0123)')
    hdr=f"{'h[ms]':>6}{'inst':>9}{'roll.01':>9}{'roll.0123':>10}{'persist':>9}{'decomp.0123':>12}{'decomp_or':>10}"
    print(hdr)
    res={}
    for h in [10,20,50,100]:
        n=h//10; kk=np.arange(k0,N-n)
        ytrue=mda[kk+n]
        p={}
        p['inst']=mdc[kk+n-1]
        p['persist']=mda[kk]
        p['roll.01']=np.array([roll(mdc[k:k+n],mda[k],tau_case,dt) for k in kk])
        p['roll.0123']=np.array([roll(mdc[k:k+n],mda[k],0.0123,dt) for k in kk])
        dm_cmd=mdc-Md0; dm_act=mda-Md0
        p['decomp']=np.array([Md0[k]+roll(dm_cmd[k:k+n],dm_act[k],0.0123,dt) for k in kk])
        p['decomp_or']=np.array([Md0[k+n]+roll(dm_cmd[k:k+n],dm_act[k],0.0123,dt) for k in kk])
        r={k:float(np.sqrt(np.mean((v-ytrue)**2))) for k,v in p.items()}
        res[h]=r
        print(f"{h:6d}{r['inst']:9.1f}{r['roll.01']:9.1f}{r['roll.0123']:10.1f}"
              f"{r['persist']:9.1f}{r['decomp']:12.1f}{r['decomp_or']:10.1f}")
    out[case]=res
json.dump(out,open('decomp_pred.json','w'),indent=2)
