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
        mdc_rms=rms(mdc), mda_rms=rms(mda), err_same=rms(mdc-mda),
        err_next=rms(mdc[:-1]-mda[1:]), qp=float(np.mean(qp>0.5)), cand=float(np.mean(cand>0.5)),
        t_end=float(t[-1]))
res={}; series={}
for sp in [70,75,80]:
    for tag,pre in [('A','nu_uniform_c3_'),('B','nu_nu_c3_')]:
        f=f'{pre}{sp}.mat'; D,t=load(f); res[(sp,tag)]=met(D,t); series[(sp,tag)]=(t,D)
print(f"{'v':>4}{'grid':>3}{'ey_max':>8}{'ey_rms':>8}{'LTR':>7}{'phi°':>7}{'SW°':>8}{'SWrate':>8}"
      f"{'Mdcmd':>8}{'Mdact':>8}{'err_same':>9}{'err_next':>9}{'QP%':>6}")
for sp in [70,75,80]:
    for tag in ['A','B']:
        r=res[(sp,tag)]
        print(f"{sp:4d}{tag:>3}{r['ey_max']:8.4f}{r['ey_rms']:8.4f}{r['ltr']:7.3f}{r['phi']:7.2f}"
              f"{r['swmax']:8.1f}{r['swrate']:8.0f}{r['mdc_rms']:8.1f}{r['mda_rms']:8.1f}"
              f"{r['err_same']:9.2f}{r['err_next']:9.2f}{100*r['qp']:6.1f}")
print()
print("变化 (B vs A):")
for sp in [70,75,80]:
    A,B=res[(sp,'A')],res[(sp,'B')]
    print(f"  {sp} km/h: ey_max {100*(B['ey_max']/A['ey_max']-1):+6.1f}%  "
          f"ey_rms {100*(B['ey_rms']/A['ey_rms']-1):+6.1f}%  "
          f"SW_max {100*(B['swmax']/A['swmax']-1):+6.1f}%  "
          f"Md_err {100*(B['err_same']/A['err_same']-1):+6.1f}%  "
          f"LTR {100*(B['ltr']/A['ltr']-1):+6.1f}%")
json.dump({f'{k[0]}_{k[1]}':v for k,v in res.items()}, open('compare_3speeds.json','w'), indent=2)
fig,ax=plt.subplots(2,2,figsize=(11,6))
for sp,c in [(70,'k'),(75,'b'),(80,'r')]:
    t,D=series[(sp,'A')]; ax[0,0].plot(t,D[0],c+'-',lw=1.0,label=f'{sp} uniform')
    t,D=series[(sp,'B')]; ax[0,0].plot(t,D[0],c+'--',lw=1.0,label=f'{sp} nonuniform')
    ax[0,1].plot(t,D[18],c+'--',lw=1.0,label=f'{sp} nonuniform')
    t,D=series[(sp,'A')]; ax[0,1].plot(t,D[18],c+'-',lw=1.0,label=f'{sp} uniform')
ax[0,0].set_ylabel('e_y [m]'); ax[0,1].set_ylabel('SW [deg]'); ax[0,0].legend(fontsize=7); ax[0,1].legend(fontsize=7)
sp=[70,75,80]; w=0.35; x=np.arange(3)
ax[1,0].bar(x-w/2,[res[(s,'A')]['ey_max'] for s in sp],w,label='A uniform')
ax[1,0].bar(x+w/2,[res[(s,'B')]['ey_max'] for s in sp],w,label='B nonuniform')
ax[1,0].set_xticks(x); ax[1,0].set_xticklabels([f'{s} km/h' for s in sp]); ax[1,0].set_ylabel('ey_max [m]'); ax[1,0].legend(fontsize=8)
ax[1,1].bar(x-w/2,[res[(s,'A')]['swmax'] for s in sp],w,label='A uniform')
ax[1,1].bar(x+w/2,[res[(s,'B')]['swmax'] for s in sp],w,label='B nonuniform')
ax[1,1].set_xticks(x); ax[1,1].set_xticklabels([f'{s} km/h' for s in sp]); ax[1,1].set_ylabel('SW_max [deg]'); ax[1,1].legend(fontsize=8)
for a in ax.ravel(): a.grid(alpha=.3)
fig.tight_layout(); fig.savefig('fig_3speeds.png',dpi=140); print('saved fig_3speeds.png')
