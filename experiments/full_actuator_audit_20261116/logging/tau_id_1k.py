"""TASK-009：1 kHz 口径的等效 tau 辨识 + Md 分解（只用 log_*.mat）。"""
import scipy.io as sio, numpy as np, json
from scipy.signal import lfilter
BD=0.5*np.array([-1.1,-1.1,1.1,1.1])
DT=1e-3
def get(rec,k):
    v = rec[0,0][k] if isinstance(rec,np.ndarray) else getattr(rec,k)
    if isinstance(v,np.ndarray) and v.dtype.names: v = v[0,0]
    return np.asarray(v['values'],float).ravel(), np.asarray(v['time'],float).ravel()
def zoh_sim(u, y0, tau, L):
    if L>0: u=np.r_[np.full(L,u[0]), u[:-L]]
    if tau<=0: return u.copy()
    a=np.exp(-DT/tau)
    y=lfilter([0,1-a],[1,-a],u, zi=[a*y0])[0]
    return y
def fit(u,y,taus,Ls):
    best=(np.inf,None,None)
    for tau in taus:
        for L in Ls:
            yh=zoh_sim(u,y[0],tau,L)
            e=float(np.sqrt(np.mean((y-yh)**2)))
            if e<best[0]: best=(e,tau,L)
    return best
TAUS=np.r_[0.0, np.logspace(np.log10(0.2e-3),np.log10(60e-3),28)]
LS=range(0,11)
out={}
for case in ['dlc75','sla80']:
    M=sio.loadmat(f'log_{case}.mat', squeeze_me=False); rec=M['rec']
    print(f'=== {case} ===')
    print(f"{'chan':>10}{'corner':>7}{'tau[ms]':>9}{'L[ms]':>7}{'RMSE':>10}{'persist':>10}{'gain%':>8}")
    res={}
    for ch,pre in [('Ia','LogIa'),('Fa','LogFa')]:
        for c in range(1,5):
            u,_=get(rec,f'LogIcmd{c}'); y,_=get(rec,f'{pre}{c}')
            n=min(len(u),len(y)); u,y=u[:n],y[:n]
            e,tau,L=fit(u,y,TAUS,LS)
            p=float(np.sqrt(np.mean((y-y[0])**2)))
            res[f'{ch}{c}']=dict(rmse=e,tau_ms=tau*1e3,L_ms=L,persist=p,gain=100*(1-e/p),n=int(n))
        # 打印
    for c in range(1,5):
        r=res[f'Ia{c}']; print(f"{'Ia':>10}{c:>7}{r['tau_ms']:9.2f}{r['L_ms']:7d}{r['rmse']:10.4f}{r['persist']:10.4f}{r['gain']:8.1f}")
    for c in range(1,5):
        r=res[f'Fa{c}']; print(f"{'Fa':>10}{c:>7}{r['tau_ms']:9.2f}{r['L_ms']:7d}{r['rmse']:10.1f}{r['persist']:10.1f}{r['gain']:8.1f}")
    out[case]=res
json.dump(out,open('tau_id_1k.json','w'),indent=2)
