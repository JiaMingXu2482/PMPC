import numpy as np, scipy.io as sio
R='./results/'
P=[('noComp / uniform','nu_fh_uniform_nocomp_c3.mat'),
   ('noComp / nonunif','nu_fh_nu_nocomp_c3.mat'),
   ('Comp25 / uniform','nu_fh_uniform_comp25_c3.mat'),
   ('Comp25 / nonunif','nu_fh_nu_comp25_c3.mat')]
def met(f):
    C=sio.loadmat(R+f, squeeze_me=False); D=np.asarray(C['D'],float); t=np.ravel(C['t'])
    ey,ltr,phi,p,mda,mdc,mz,sw=D[0],D[4],D[2],D[3],D[5],D[6],D[17],D[18]
    rms=lambda x: float(np.sqrt(np.mean(x**2)))
    return dict(ey_max=float(np.max(np.abs(ey))), ey_rms=rms(ey), ltr=float(np.max(np.abs(ltr))),
        phi=float(np.degrees(np.max(np.abs(phi)))), pmax=float(np.degrees(np.max(np.abs(p)))),
        mz=float(np.mean(np.abs(mz))), mda=rms(mda), mdc=rms(mdc), err=rms(mdc-mda),
        vxmin=float(np.min(D[19])*3.6), qp=int(np.sum(D[23]<0.5)))
print(f"{'config':>18}{'ey_max':>9}{'ey_rms':>9}{'LTR':>7}{'roll°':>7}{'p°/s':>7}{'Mz_avg':>8}{'Mdact':>8}{'Md_err':>8}{'Vxmin':>7}{'QPfail':>7}")
res={}
for name,f in P:
    r=met(f); res[name]=r
    print(f"{name:>18}{r['ey_max']:9.3f}{r['ey_rms']:9.3f}{r['ltr']:7.3f}{r['phi']:7.2f}"
          f"{r['pmax']:7.1f}{r['mz']:8.0f}{r['mda']:8.0f}{r['err']:8.1f}{r['vxmin']:7.1f}{r['qp']:7d}")
print()
u=res['noComp / uniform']; un=res['noComp / nonunif']
cu=res['Comp25 / uniform']; cn=res['Comp25 / nonunif']
for k in ['ey_max','ey_rms','ltr','phi','mz','err']:
    print(f"  {k:8} noComp {u[k]:8.3f}->{un[k]:8.3f} ({100*(un[k]/u[k]-1):+6.1f}%) | "
          f"Comp {cu[k]:8.3f}->{cn[k]:8.3f} ({100*(cn[k]/cu[k]-1):+6.1f}%) | "
          f"nu: Comp vs noComp {100*(cn[k]/un[k]-1):+6.1f}%")
