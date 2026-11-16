import scipy.io as sio, numpy as np, sys, json
case=sys.argv[1] if len(sys.argv)>1 else 'dlc75'
L=sio.loadmat(f'log_{case}.mat', squeeze_me=True, struct_as_record=False)
R=sio.loadmat(f'replay_{case}.mat', squeeze_me=True, struct_as_record=False)
rec=L['rec']; res=R['res']
def get(s, i):
    v = s[i] if isinstance(s, np.ndarray) else s
    return np.asarray(v.values if hasattr(v,'values') else v, float).ravel()
print(f'== {case} ==')
print(f"{'corner':>7}{'Ia max':>12}{'Ia RMSE':>12}{'Fa max':>12}{'Fa RMSE':>12}{'n':>7}")
out={}
for c in range(1,5):
    ia_l=get(rec.__dict__[f'LogIa{c}'],0)
    fa_l=get(rec.__dict__[f'LogFa{c}'],0)
    ia_r=get(res.ia,c-1)
    fa_r=get(res.fa,c-1)
    n=min(len(ia_l),len(ia_r),len(fa_l),len(fa_r))
    e_ia=ia_r[:n]-ia_l[:n]; e_fa=fa_r[:n]-fa_l[:n]
    out[c]=dict(ia_max=float(np.max(np.abs(e_ia))), ia_rmse=float(np.sqrt(np.mean(e_ia**2))),
                fa_max=float(np.max(np.abs(e_fa))), fa_rmse=float(np.sqrt(np.mean(e_fa**2))), n=int(n))
    print(f"{c:7d}{out[c]['ia_max']:12.4e}{out[c]['ia_rmse']:12.4e}{out[c]['fa_max']:12.2f}{out[c]['fa_rmse']:12.2f}{n:7d}")
json.dump(out, open(f'cmp_replay_{case}.json','w'), indent=2)
