import numpy as np, scipy.io as sio, json, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
R='./results/'
def load(f):
    C=sio.loadmat(R+f, squeeze_me=False)
    D=np.asarray(C['D'],float); t=np.ravel(C['t']); ri=C.get('runinfo')
    return D,t,ri
rows={}
for tag,f in [('A uniform 10x100ms','nu_uniform_c3_70.mat'),('B nonuniform 19','nu_nu_c3_70.mat')]:
    D,t,ri=load(f)
    ey,epsi,phi,p,ltr,mda,mdc,sw=D[0],D[1],D[2],D[3],D[4],D[5],D[6],D[18]
    qp=D[23]; cand=D[24]
    rms=lambda x: float(np.sqrt(np.mean(x**2)))
    dsw=np.diff(sw)/np.mean(np.diff(t))
    rows[tag]=dict(t=t,ey=ey,sw=sw,mda=mda,mdc=mdc,
        ey_max=float(np.max(np.abs(ey))), ey_rms=rms(ey),
        ltr=float(np.max(np.abs(ltr))), phi=float(np.degrees(np.max(np.abs(phi)))),
        pmax=float(np.degrees(np.max(np.abs(p)))), swmax=float(np.max(np.abs(sw))),
        swrate=float(np.max(np.abs(dsw))),
        mdc_rms=rms(mdc), mda_rms=rms(mda),
        mdc_max=float(np.max(np.abs(mdc))), mda_max=float(np.max(np.abs(mda))),
        err_same=rms(mdc-mda),
        err_next=rms(mdc[:-1]-mda[1:]),
        qp=float(np.mean(qp>0.5)), cand=float(np.mean(cand>0.5)),
        np_=None)
A,B=rows['A uniform 10x100ms'],rows['B nonuniform 19']
def d(x,y): return 100*(y-x)/x
print(f"{'metric':16}{'A':>10}{'B':>10}{'B vs A':>10}")
for k in ['ey_max','ey_rms','ltr','phi','pmax','swmax','swrate','mdc_rms','mda_rms','mdc_max','mda_max','err_same','err_next']:
    print(f"{k:16}{A[k]:10.3f}{B[k]:10.3f}{d(A[k],B[k]):9.1f}%")
print(f"{'QP%':16}{100*A['qp']:10.1f}{100*B['qp']:10.1f}")
print(f"{'cand%':16}{100*A['cand']:10.1f}{100*B['cand']:10.1f}")
json.dump({'A':{k:v for k,v in A.items() if k not in('t','ey','sw','mda','mdc')},
           'B':{k:v for k,v in B.items() if k not in('t','ey','sw','mda','mdc')}},
          open('compare_corrected.json','w'), indent=2)
fig,ax=plt.subplots(3,1,figsize=(10,7),sharex=True)
for tag,c in [('A uniform 10x100ms','k'),('B nonuniform 19','r')]:
    o=rows[tag]; lbl='A uniform 10x100ms' if tag.startswith('A') else 'B nonuniform 19'
    ax[0].plot(o['t'],o['ey'],c,lw=1.1,label=lbl)
    ax[1].plot(o['t'],o['sw'],c,lw=1.0,label=lbl)
    ax[2].plot(o['t'],o['mdc']-o['mda'],c,lw=0.8,label=lbl)
ax[0].set_ylabel('e_y [m]'); ax[1].set_ylabel('SW [deg]'); ax[2].set_ylabel('Md_cmd - Md_act [N m]')
ax[2].set_xlabel('t [s]')
for a in ax: a.grid(alpha=.3); a.legend(fontsize=8)
fig.tight_layout(); fig.savefig('fig_AB_compare_corrected.png',dpi=140); print('saved fig')
