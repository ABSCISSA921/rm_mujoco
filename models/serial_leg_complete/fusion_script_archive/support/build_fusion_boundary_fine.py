import json,mujoco,numpy as np
from scipy.optimize import least_squares
from pathlib import Path
m=mujoco.MjModel.from_xml_path('outputs/mjcf_left_right/robot.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d);parts=['left_thigh','left_output','left_calf','left_longbar','left_midbar','left_shortbar'];jn=['hip_thigh','hip_output','knee','longbar_joint','midbar_joint','shortbar_joint'];qi=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'left_'+n)] for n in jn];initial={}
for n in parts:
 b=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n);initial[n]=(d.xpos[b].copy(),d.xmat[b].reshape(3,3).copy())
w=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'left_wheel');hip=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'left_thigh')
def trans():
 item={}
 for n in parts:
  b=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n);p0,R0=initial[n];R=d.xmat[b].reshape(3,3)@R0.T;p=d.xpos[b]-R@p0;T=np.eye(4);T[:3,:3]=R;T[:3,3]=p*100;item[n]=T.flatten().tolist()
 return item
out={};meta={}; samples=json.load(open('work/zero_pose_kinematic_probe.json'))['left']['samples'];s=samples[115];start=np.array(list(s['q_rad'].values()))

fine=json.load(open('work/fusion_probe_all_metadata.json'));start=np.array(list(fine['fine_5']['q_rad'].values()));lo=fine['fine_5']['z_m'];hi=fine['fine_6']['z_m']
for i,z in enumerate(np.linspace(lo,hi,41)):
 def fun(q):
  d.qpos[qi]=q;mujoco.mj_forward(m,d);v=d.xpos[w]-d.xpos[hip];return np.r_[d.efc_pos[6:12],v[0],v[2]-z]
 res=least_squares(fun,start,max_nfev=200,ftol=1e-12,xtol=1e-12,gtol=1e-12);assert np.linalg.norm(fun(res.x))<1e-7;start=res.x;name='fine_'+str(i);out[name]=trans();meta[name]={'q_rad':dict(zip(jn,map(float,res.x))),'z_m':float(z)}
Path('work/fusion_boundary_ultrafine_transforms.json').write_text(json.dumps(out),encoding='utf8');Path('work/fusion_boundary_ultrafine_metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf8');print('poses',len(out))
