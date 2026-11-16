"""TASK-009：Md 分解 —— 运动共同阻尼(Md@0A) vs 电流可调部分。"""
import scipy.io as sio, numpy as np, json
BD=0.5*np.array([-1.1,-1.1,1.1,1.1])
def get(rec,k):
    v = rec[0,0][k] if isinstance(rec,np.ndarray) else getattr(rec,k)
    if isinstance(v,np.ndarray) and v.dtype.names: v=v[0,0]
    return np.asarray(v['values'],float), np.asarray(v['time'],float).ravel()
def zoh_up(t,y,tu):
    idx=np.clip(np.searchsorted(t,tu,side='right')-1,0,len(y)-1)
    return y[idx]
rms=lambda x: float(np.sqrt(np.mean(np.square(x))))
out={}
for case in ['dlc75','sla80']:
    M=sio.loadmat(f'log_{case}.mat', squeeze_me=False); rec=M['rec']
    tb,_=get(rec,'LogIcmd1')
    Fa ,_=get(rec,'LogFa1');  Fa=np.stack([np.ravel(get(rec,f'LogFa{c}')[0]) for c in range(1,5)],1)
    F0r,t0r=get(rec,'LogFat0'); F0=np.atleast_2d(F0r)
    F16r,_=get(rec,'LogFat16'); F16=np.atleast_2d(F16r)
    print(f'{case}: Fa{Fa.shape} Fat0{F0.shape} Fat16{F16.shape} t0 {t0r.shape}')
    Md_act=Fa@BD
    if F0.shape[0]==4: F0=F0.T
    if F16.shape[0]==4: F16=F16.T
    Md0=zoh_up(t0r,F0@BD,tb); Md16=zoh_up(t0r,F16@BD,tb)
    rng=Md16-Md0
    good=np.abs(rng)>1.0
    alpha=np.full_like(Md_act,np.nan); alpha[good]=(Md_act[good]-Md0[good])/rng[good]
    i0=int(np.searchsorted(tb,1.0)); s=slice(i0,None)
    res=dict(n=int(len(tb)), Md_act_rms=rms(Md_act[s]), Md_at0_rms=rms(Md0[s]),
             Md_range_rms=rms(rng[s]),
             corr_with_0A=float(np.corrcoef(Md_act[s],Md0[s])[0,1]),
             alpha_median=float(np.nanmedian(alpha[s])),
             alpha_p10=float(np.nanpercentile(alpha[s],10)),
             alpha_p90=float(np.nanpercentile(alpha[s],90)),
             share_motion=float(rms(Md0[s])**2/max(rms(Md_act[s])**2,1e-9)))
    out[case]=res
    print(f'== {case} (t>=1 s) ==')
    for k,v in res.items(): print(f'   {k:16s} {v:12.4f}')
json.dump(out,open('md_decomp_1k.json','w'),indent=2)
