from pathlib import Path
s=Path('work/check_cad_mesh_interference.py').read_text(encoding='utf-8-sig');exec(s.split('N=len(qdata')[0]);from scipy.optimize import least_squares
zero=json.load(open('work/fusion_boundary_ultrafine_metadata.json'))['fine_19']['q_rad'];offs=zero['hip_output']-zero['hip_thigh'];qi=[]
for n in names:
 j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'left_'+n);qi.append(m.jnt_qposadr[j])
base={tuple(v['pair']) for v in check(0)['contacts']};rng=np.random.default_rng(176);out=[]
for i in range(2000):
 a=float(rng.uniform(-np.pi,np.pi));rel=float(-2.52906602731*(i+0.5)/2000);b=a+rel+offs;mujoco.mj_resetData(m,d);d.qpos[qi[0]]=a;d.qpos[qi[1]]=b
 def fun(q):d.qpos[qi[2:]]=q;mujoco.mj_forward(m,d);return d.efc_pos[6:12].copy()
 x=least_squares(fun,np.zeros(4),max_nfev=100,ftol=1e-9,xtol=1e-9,gtol=1e-9);err=float(np.linalg.norm(fun(x.x)));entry={'index':i,'thigh_rad':a,'output_rad':b,'relative_from_zero':rel,'closure':err}
 if err<1e-6:
  qdata['left']['samples'].append({'wheel_relative_z_m':0,'q_rad':dict(zip(names,map(float,np.r_[a,b,x.x])))});hits=[v for v in check(-1)['contacts'] if tuple(v['pair']) not in base];entry['new_intersections']=hits
 out.append(entry)
 if i%50==0:
  Path('work/dense_allowed_progress.json').write_text(json.dumps(out,indent=2),encoding='utf8');print(i,'samples',flush=True)
Path('work/dense_allowed_grid.json').write_text(json.dumps(out,indent=2),encoding='utf8');print('done',flush=True)
