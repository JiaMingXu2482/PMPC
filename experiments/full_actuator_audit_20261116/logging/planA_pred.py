"""方案 A 离线验证：用已有力界（Flo/Fhi 的 Np+1 节点）= 现成的 Md_at0 预示。
P_A: y[k+n] = Md0_preview(k,n) + rollout(ΔMd_cmd, ΔMd_act[k], τ)
其中 Md0_preview(k,n) = Bd'·Flo(:,n+1,k)（力界第 n 个节点的 0 A 力）。
对照：inst / roll(τ) / persist / decomp(保持) / decomp_or(未来真值)
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
    Flo=np.asarray(v['LogFlo'][0,0]['values'],float)     # 4 x 11 x N1k
    Fhi=np.asarray(v['LogFhi'][0,0]['values'],float)
    U=np.asarray(v['LogU'][0,0]['values'],float)         # N1k x 54
    if F0.shape[0]==4: F0=F0.T
    Md0_1k=F0@BD
    # 用当前压缩速度符号选端点：压缩(v>0)时 0A 是较大的那侧(Fhi)，复位时是较小的那侧(Flo)
    vnow=U[:,0:4]                                        # N1k x 4, cmpRD
    sgn=(vnow>0).astype(float)                           # 1=压缩
    # 逐节点构造 4x11
    Md0p_1k=np.zeros((Flo.shape[2], Flo.shape[1]))
    for n in range(Flo.shape[1]):
        f = sgn*Fhi[:,n,:].T + (1-sgn)*Flo[:,n,:].T      # N1k x 4
        Md0p_1k[:,n] = f@BD
    D=np.asarray(M['D'],float); t100=np.asarray(M['t'],float).ravel()
    idx=np.clip(np.searchsorted(tb,t100,side='right')-1,0,len(tb)-1)
    return D,t100,Md0_1k[idx],Md0p_1k[idx,:]
out={}
for case,tau_case in [('dlc75',0.01),('sla80',0.20)]:
    D,t100,Md0,Md0p=load(case)
    mdc=D[6]; mda=D[5]; dt=float(np.mean(np.diff(t100)))
    k0=int(np.searchsorted(t100,1.0)); N=len(t100)
    print(f'== {case} ==   τ={tau_case}  (参考: 12.3 ms)')
    print(f"{'h':>5}{'inst':>9}{'rollτ':>9}{'persist':>9}{'decomp':>9}{'**P_A**':>9}{'oracle':>9}{'A限':>7}")
    res={}
    for h in [10,20,50,100]:
        n=h//10; kk=np.arange(k0,N-n); ytrue=mda[kk+n]
        dm_cmd=mdc-Md0; dm_act=mda-Md0
        p={}
        p['inst']=mdc[kk+n-1]
        p['persist']=mda[kk]
        p['roll']=np.array([roll(mdc[k:k+n],mda[k],tau_case,dt) for k in kk])
        p['decomp']=np.array([Md0[k]+roll(dm_cmd[k:k+n],dm_act[k],0.0123,dt) for k in kk])
        p['oracle']=np.array([Md0[k+n]+roll(dm_cmd[k:k+n],dm_act[k],0.0123,dt) for k in kk])
        # P_A：力界第 n 个节点（n=0 为当前节点）
        idxf=np.clip(n,0,10)
        md0p=Md0p[kk, idxf]
        p['PA']=md0p+np.array([roll(dm_cmd[k:k+n],dm_act[k],0.0123,dt) for k in kk])
        r={k2:float(np.sqrt(np.mean((vv-ytrue)**2))) for k2,vv in p.items()}
        # A 相对于 oracle 的达成率
        gain_inst=r['inst']-r['oracle']; gain_A=r['inst']-r['PA']
        res[h]=dict(r=r, reach=float(gain_A/gain_inst) if abs(gain_inst)>1e-9 else float('nan'))
        print(f"{h:5d}{r['inst']:9.1f}{r['roll']:9.1f}{r['persist']:9.1f}{r['decomp']:9.1f}"
              f"{r['PA']:9.1f}{r['oracle']:9.1f}{100*res[h]['reach']:6.0f}%")
    out[case]=res
json.dump(out,open('planA_pred.json','w'),indent=2)
