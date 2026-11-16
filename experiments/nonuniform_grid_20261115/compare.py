import numpy as np, scipy.io as sio, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
R='./results/'
def m(f):
    C=sio.loadmat(R+f, squeeze_me=False); D=np.asarray(C['D'],float); t=np.ravel(C['t'])
    return D,t
out={}
for tag,f in [('A uniform 10x100ms','nu_uniform_c3_70.mat'),('B 非均匀 19 步','nu_nu_c3_70.mat')]:
    D,t=m(f)
    ey=D[0]; epsi=D[1]; phi=D[2]; p=D[3]; ltr=D[4]; mda=D[5]; mdc=D[6]; sw=D[18]; qp=D[23]
    out[tag]=dict(t=t,ey=ey,ltr=ltr,sw=sw,mda=mda,mdc=mdc,phi=phi,p=p,qp=qp)
    rms=lambda x: float(np.sqrt(np.mean(x**2)))
    dsw=np.diff(sw)/np.mean(np.diff(t))
    print(f"{tag:20} ey_max={np.max(np.abs(ey)):.4f} ey_rms={rms(ey):.4f} "
          f"LTR={np.max(np.abs(ltr)):.3f} phi={np.degrees(np.max(np.abs(phi))):.2f}deg "
          f"p={np.degrees(np.max(np.abs(p))):.1f}deg/s SW={np.max(np.abs(sw)):.1f}deg "
          f"SWrate={np.max(np.abs(dsw)):.0f}deg/s "
          f"Md_track_rms={rms(mdc-mda):.1f} QP={100*np.mean(qp>0.5):.1f}%")
# 图
fig,ax=plt.subplots(3,1,figsize=(10,7),sharex=True)
for tag,c in [('A uniform 10x100ms','k'),('B 非均匀 19 步','r')]:
    o=out[tag]; ax[0].plot(o['t'],o['ey'],c,lw=1.1,label=tag)
    ax[1].plot(o['t'],o['sw'],c,lw=1.0,label=tag)
    ax[2].plot(o['t'],o['mdc']-o['mda'],c,lw=0.8,label=tag)
ax[0].set_ylabel('e_y [m]'); ax[1].set_ylabel('SW [deg]'); ax[2].set_ylabel('Md_cmd - Md_act [N·m]')
ax[2].set_xlabel('t [s]')
for a in ax: a.grid(alpha=.3); a.legend(fontsize=8)
fig.tight_layout(); fig.savefig('fig_AB_compare.png',dpi=140)
print('saved fig_AB_compare.png')
