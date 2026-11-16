import numpy as np, scipy.io as sio, json, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
BASE='D:/OneDrive - mail.scut.edu.cn/Thesis/Code/Controller/'
SRC={
 'Baseline':      lambda s: BASE+f'Baseline/PMPC_Results/Baseline_MPC/bl_dlc_{s}_v3.mat',
 'noComp(unif)':  lambda s: f'./results/fair_dlc{s}_nc_uni.mat',
 'noComp(nonunif)':lambda s: f'./results/fair_dlc{s}_nc_nu.mat',
 'Comp(uniform)': lambda s: f'./results/fair_dlc{s}_c_uni.mat',
 'Comp(nonunif)': lambda s: f'./results/fair_dlc{s}_c_nu.mat',
}
def load(f):
    C=sio.loadmat(f, squeeze_me=False); D=np.asarray(C['D'],float); t=np.ravel(C['t'])
    return D,t
def met(D,t):
    ey,epsi,phi,p,ltr,mz,sw=D[0],D[1],D[2],D[3],D[4],D[17],D[18]
    qp=D[23]; rms=lambda x: float(np.sqrt(np.mean(x**2)))
    out=dict(ey_max=float(np.max(np.abs(ey))), ey_rms=rms(ey),
        ltr=float(np.max(np.abs(ltr))), phi=float(np.degrees(np.max(np.abs(phi)))),
        pmax=float(np.degrees(np.max(np.abs(p)))), swmax=float(np.max(np.abs(sw))),
        swrate=float(np.max(np.abs(np.diff(sw)/np.mean(np.diff(t))))),
        mz_avg=float(np.mean(np.abs(mz))), qp=float(np.mean(qp>0.5)),
        t_end=float(t[-1]))
    if D.shape[0]>=27:
        mda,mdc=D[5],D[6]
        out['mdc_rms']=rms(mdc); out['mda_rms']=rms(mda); out['err']=rms(mdc-mda)
    else:
        out['mdc_rms']=float('nan'); out['mda_rms']=float('nan'); out['err']=float('nan')
    return out
res={}; ser={}
for s in [70,75,80]:
    for name,f in SRC.items():
        D,t=load(f(s)); res[(s,name)]=met(D,t); ser[(s,name)]=(t,D)
hdr=f"{'v':>4} {'controller':>15}{'ey_max':>9}{'ey_rms':>9}{'LTR':>7}{'phi°':>7}{'SW°':>8}{'SWrate':>8}{'Mz_avg':>8}{'Mdact':>8}{'QP%':>6}"
print(hdr)
for s in [70,75,80]:
    for name in SRC:
        r=res[(s,name)]
        print(f"{s:4d} {name:>15}{r['ey_max']:9.4f}{r['ey_rms']:9.4f}{r['ltr']:7.3f}{r['phi']:7.2f}"
              f"{r['swmax']:8.1f}{r['swrate']:8.0f}{r['mz_avg']:8.0f}{r['mda_rms']:8.1f}{100*r['qp']:6.1f}")
json.dump({f'{k[0]}_{k[1]}':v for k,v in res.items()}, open('compare_bl_nc_cp.json','w'), indent=2)
fig,ax=plt.subplots(1,3,figsize=(15,3.8))
sty={'Baseline':'k:','noComp':'b--','Comp(uniform)':'g-.','Comp(nonunif)':'r-'}
for i,s in enumerate([70,75,80]):
    for name in SRC:
        t,D=ser[(s,name)]; ax[i].plot(t,D[0],sty[name],lw=1.1,label=name)
    ax[i].set_title(f'DLC {s} km/h'); ax[i].set_xlabel('t [s]'); ax[i].set_ylabel('e_y [m]')
    ax[i].grid(alpha=.3); ax[i].legend(fontsize=7)
fig.tight_layout(); fig.savefig('fig_bl_nc_cp.png',dpi=140)
print('saved fig_bl_nc_cp.png')
