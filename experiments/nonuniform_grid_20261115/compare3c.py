import numpy as np, scipy.io as sio, json, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
R='./results/'
def load(f):
    C=sio.loadmat(R+f, squeeze_me=False); D=np.asarray(C['D'],float); t=np.ravel(C['t'])
    return D,t
def met(D,t):
    ey,epsi,phi,p,ltr,mda,mdc,sw=D[0],D[1],D[2],D[3],D[4],D[5],D[6],D[18]
    qp=D[23]; cand=D[24]; rms=lambda x: float(np.sqrt(np.mean(x**2)))
    return dict(ey_max=float(np.max(np.abs(ey))), ey_rms=rms(ey),
        ltr=float(np.max(np.abs(ltr))), phi=float(np.degrees(np.max(np.abs(phi)))),
        pmax=float(np.degrees(np.max(np.abs(p)))), swmax=float(np.max(np.abs(sw))),
        swrate=float(np.max(np.abs(np.diff(sw)/np.mean(np.diff(t))))),
        mdc_rms=rms(mdc), mda_rms=rms(mda), err=rms(mdc-mda),
        qp=float(np.mean(qp>0.5)), cand=float(np.mean(cand>0.5)), t_end=float(t[-1]))
names={1:'MPC',2:'ZENG',3:'Proposed'}
res={}; ser={}
for sp in [70,75,80]:
    for c in [1,2,3]:
        f=f'nu_uniform_c{c}_{sp}.mat'
        D,t=load(f); res[(sp,c)]=met(D,t); ser[(sp,c)]=(t,D)
print(f"{'v':>4} {'ctrl':>9}{'ey_max':>9}{'ey_rms':>9}{'LTR':>7}{'phi°':>7}{'SW°':>8}{'SWrate':>8}{'Mdcmd':>8}{'Mdact':>8}{'QP%':>6}")
for sp in [70,75,80]:
    for c in [1,2,3]:
        r=res[(sp,c)]
        print(f"{sp:4d} {names[c]:>9}{r['ey_max']:9.4f}{r['ey_rms']:9.4f}{r['ltr']:7.3f}{r['phi']:7.2f}"
              f"{r['swmax']:8.1f}{r['swrate']:8.0f}{r['mdc_rms']:8.1f}{r['mda_rms']:8.1f}{100*r['qp']:6.1f}")
json.dump({f'{k[0]}_{names[k[1]]}':v for k,v in res.items()}, open('compare_3ctrl.json','w'), indent=2)
fig,ax=plt.subplots(1,3,figsize=(14,3.6))
for i,sp in enumerate([70,75,80]):
    for c,st in [(1,'k:'),(2,'b--'),(3,'r-')]:
        t,D=ser[(sp,c)]; ax[i].plot(t,D[0],st,lw=1.1,label=names[c])
    ax[i].set_title(f'DLC {sp} km/h (uniform grid)'); ax[i].set_xlabel('t [s]')
    ax[i].set_ylabel('e_y [m]'); ax[i].grid(alpha=.3); ax[i].legend(fontsize=8)
fig.tight_layout(); fig.savefig('fig_3controllers.png',dpi=140)
print('saved fig_3controllers.png')
