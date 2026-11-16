import numpy as np, scipy.io as sio, json, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
BASE='D:/OneDrive - mail.scut.edu.cn/Thesis/Code/Controller/'
SRC={
 'Baseline':      BASE+'Baseline/PMPC_Results/Baseline_MPC/bl_sla_80_v3.mat',
 'noComp':        BASE+'Proposed_noComp/PMPC_Results/Slalom_mu0.85_interp/nc_sla_80.mat',
 'Comp(uniform)': './results/sla_uniform_c3_80.mat',
 'Comp(nonunif)': './results/sla_nu_c3_80.mat',
}
def load(f):
    C=sio.loadmat(f, squeeze_me=False); D=np.asarray(C['D'],float); t=np.ravel(C['t'])
    return D,t
def met(D,t):
    ey,epsi,phi,p,ltr,mz,sw=D[0],D[1],D[2],D[3],D[4],D[17],D[18]
    qp=D[23]; rms=lambda x: float(np.sqrt(np.mean(x**2)))
    o=dict(ey_max=float(np.max(np.abs(ey))), ey_rms=rms(ey),
        ltr=float(np.max(np.abs(ltr))), phi=float(np.degrees(np.max(np.abs(phi)))),
        pmax=float(np.degrees(np.max(np.abs(p)))), swmax=float(np.max(np.abs(sw))),
        swrate=float(np.max(np.abs(np.diff(sw)/np.mean(np.diff(t))))),
        mz_avg=float(np.mean(np.abs(mz))), qp=float(np.mean(qp>0.5)))
    if D.shape[0]>=27:
        o['mdc_rms']=rms(D[6]); o['mda_rms']=rms(D[5]); o['err']=rms(D[6]-D[5])
    else:
        o['mdc_rms']=o['mda_rms']=o['err']=float('nan')
    return o
res={}; ser={}
for name,f in SRC.items():
    D,t=load(f); res[name]=met(D,t); ser[name]=(t,D)
print(f"{'controller':>15}{'ey_max':>9}{'ey_rms':>9}{'LTR':>7}{'phi°':>7}{'SW°':>8}{'SWrate':>8}{'Mz_avg':>8}{'Mdact':>8}{'QP%':>6}")
for name in SRC:
    r=res[name]
    print(f"{name:>15}{r['ey_max']:9.4f}{r['ey_rms']:9.4f}{r['ltr']:7.3f}{r['phi']:7.2f}"
          f"{r['swmax']:8.1f}{r['swrate']:8.0f}{r['mz_avg']:8.0f}{r['mda_rms']:8.1f}{100*r['qp']:6.1f}")
json.dump(res, open('compare_slalom.json','w'), indent=2)
fig,ax=plt.subplots(1,3,figsize=(15,3.6))
sty={'Baseline':'k:','noComp':'b--','Comp(uniform)':'g-.','Comp(nonunif)':'r-'}
for k,(ttl,row) in enumerate([('e_y [m]',0),('SW [deg]',18),('LTR',4)]):
    for name in SRC:
        t,D=ser[name]; ax[k].plot(t,D[row],sty[name],lw=1.1,label=name)
    ax[k].set_title(ttl); ax[k].set_xlabel('t [s]'); ax[k].grid(alpha=.3); ax[k].legend(fontsize=7)
fig.tight_layout(); fig.savefig('fig_slalom_compare.png',dpi=140); print('saved fig_slalom_compare.png')
