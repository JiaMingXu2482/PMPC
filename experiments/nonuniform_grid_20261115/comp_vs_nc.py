import numpy as np, scipy.io as sio, json
R='./results/'
PAIRS=[  # (case, grid, noComp file, Comp file)
 ('DLC70','uni', 'fair_dlc70_nc_uni.mat', 'nu_uniform_c3_70.mat'),
 ('DLC70','nu',  'fair_dlc70_nc_nu.mat',  'nu_nu_c3_70.mat'),
 ('DLC75','uni', 'fair_dlc75_nc_uni.mat', 'fair_dlc75_c_uni.mat'),
 ('DLC75','nu',  'fair_dlc75_nc_nu.mat',  'fair_dlc75_c_nu.mat'),
 ('DLC80','uni', 'fair_dlc80_nc_uni.mat', 'nu_uniform_c3_80.mat'),
 ('DLC80','nu',  'fair_dlc80_nc_nu.mat',  'nu_nu_c3_80.mat'),
 ('SLA80','uni', 'fair_sla80_nc_uni.mat', 'fair_sla80_c_uni.mat'),
 ('SLA80','nu',  'fair_sla80_nc_nu.mat',  'fair_sla80_c_nu.mat'),
]
def met(f):
    C=sio.loadmat(R+f, squeeze_me=False); D=np.asarray(C['D'],float); t=np.ravel(C['t'])
    ey,epsi,phi,p,ltr,mda,mdc,mz,sw=D[0],D[1],D[2],D[3],D[4],D[5],D[6],D[17],D[18]
    rms=lambda x: float(np.sqrt(np.mean(x**2)))
    return dict(ey_max=float(np.max(np.abs(ey))), ey_rms=rms(ey),
        ltr=float(np.max(np.abs(ltr))), phi=float(np.degrees(np.max(np.abs(phi)))),
        sw=float(np.max(np.abs(sw))), swrms=float(np.sqrt(np.mean(np.diff(sw)**2))/0.01),
        mz=float(np.mean(np.abs(mz))), err=rms(mdc-mda),
        qp=int(np.sum(D[23]<0.5)), cand=int(np.sum(D[24]>0.5)))
rows=[]
for case,grid,fn,fc in PAIRS:
    n=met(fn); c=met(fc)
    d=lambda k: 100*(c[k]-n[k])/n[k] if n[k] else float('nan')
    rows.append(dict(case=case, grid=grid, nc=n, c=c,
        d_ey=d('ey_max'), d_eyr=d('ey_rms'), d_sw=d('sw'),
        d_ltr=d('ltr'), d_phi=d('phi'), d_mz=d('mz'), d_err=d('err')))
print(f"{'case':7}{'grid':5}{'nc ey':>8}{'c ey':>8}{'Δey%':>8}{'nc eyr':>8}{'c eyr':>8}{'Δeyr%':>8}"
      f"{'nc SW':>8}{'c SW':>8}{'ΔSW%':>8}{'ΔLTR%':>8}{'Δφ%':>7}{'ΔMz%':>8}{'ΔMd_err%':>10}{'QPfail':>8}")
for r in rows:
    n,c=r['nc'],r['c']
    print(f"{r['case']:7}{r['grid']:5}{n['ey_max']:8.4f}{c['ey_max']:8.4f}{r['d_ey']:+8.1f}"
          f"{n['ey_rms']:8.4f}{c['ey_rms']:8.4f}{r['d_eyr']:+8.1f}"
          f"{n['sw']:8.1f}{c['sw']:8.1f}{r['d_sw']:+8.1f}{r['d_ltr']:+8.1f}{r['d_phi']:+7.1f}"
          f"{r['d_mz']:+8.1f}{r['d_err']:+10.1f}{n['qp']:4d}/{c['qp']:<3d}")
json.dump(rows, open('comp_vs_nc.json','w'), indent=2, default=float)
