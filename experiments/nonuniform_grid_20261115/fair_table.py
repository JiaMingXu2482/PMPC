import numpy as np, scipy.io as sio, json, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
BL='D:/OneDrive - mail.scut.edu.cn/Thesis/Code/Controller/Baseline/PMPC_Results/Baseline_MPC/'
CFG=[('Baseline', BL+'fair_bl_{c}.mat'),
     ('noComp / uniform',   './results/fair_{c}_nc_uni.mat'),
     ('noComp / nonunif',   './results/fair_{c}_nc_nu.mat'),
     ('Comp / uniform',     './results/fair_{c}_c_uni.mat'),
     ('Comp / nonunif',     './results/fair_{c}_c_nu.mat')]
def met(f):
    C=sio.loadmat(f, squeeze_me=False); D=np.asarray(C['D'],float); t=np.ravel(C['t'])
    ey,epsi,phi,p,ltr,mz,sw=D[0],D[1],D[2],D[3],D[4],D[17],D[18]; qp=D[23]; cand=D[24]
    rms=lambda x: float(np.sqrt(np.mean(x**2)))
    o=dict(ey_max=float(np.max(np.abs(ey))), ey_rms=rms(ey), ltr=float(np.max(np.abs(ltr))),
           phi=float(np.degrees(np.max(np.abs(phi)))), pmax=float(np.degrees(np.max(np.abs(p)))),
           swmax=float(np.max(np.abs(sw))), swrms=float(np.sqrt(np.mean(np.diff(sw)**2))/0.01),
           mz=float(np.mean(np.abs(mz))), swrate=float(np.max(np.abs(np.diff(sw)))/0.01),
           qpfail=int(np.sum(qp<0.5)), cand=int(np.sum(cand>0.5)), n=int(qp.size))
    if D.shape[0]>=27:
        o['mda']=rms(D[5]); o['mdc']=rms(D[6]); o['err']=rms(D[6]-D[5])
    else:
        o['mda']=o['mdc']=o['err']=float('nan')
    return o
res={}
for case,label in [('dlc75','DLC 75 km/h'),('sla80','Slalom 80 km/h')]:
    print('='*100); print(label)
    print(f"{'config':>20}{'ey_max':>9}{'ey_rms':>9}{'LTR':>7}{'phi°':>7}{'SW°':>8}{'SWrms°/s':>10}{'Mz_avg':>8}{'Mdact':>8}{'QPfail':>7}{'cand':>6}")
    for name,pat in CFG:
        f=pat.format(c=case)
        r=met(f); res[(case,name)]=r
        print(f"{name:>20}{r['ey_max']:9.4f}{r['ey_rms']:9.4f}{r['ltr']:7.3f}{r['phi']:7.2f}"
              f"{r['swmax']:8.1f}{r['swrms']:10.0f}{r['mz']:8.0f}{r['mda']:8.1f}{r['qpfail']:7d}{r['cand']:6d}")
json.dump({f'{k[0]}|{k[1]}':v for k,v in res.items()}, open('fair_table.json','w'), indent=2)
