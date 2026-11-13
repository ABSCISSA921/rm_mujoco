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
for z in np.linspace(samples[115]['wheel_relative_z_m'],samples[116]['wheel_relative_z_m'],21):
 def fun(q):
  d.qpos[qi]=q;mujoco.mj_forward(m,d);v=d.xpos[w]-d.xpos[hip];return np.r_[d.efc_pos[6:12],v[0],v[2]-z]
 res=least_squares(fun,start,max_nfev=200,ftol=1e-11,xtol=1e-11,gtol=1e-11);assert np.linalg.norm(fun(res.x))<1e-7;start=res.x;name='fine_'+str(len(meta));out[name]=trans();meta[name]={'q_rad':dict(zip(jn,map(float,res.x))),'z_m':float(z)}
for a in np.linspace(-np.pi,np.pi,25):
 for b in np.linspace(-np.pi,np.pi,25):
  mujoco.mj_resetData(m,d);d.qpos[qi[0]]=a;d.qpos[qi[1]]=b
  def fun(q):d.qpos[qi[2:]]=q;mujoco.mj_forward(m,d);return d.efc_pos[6:12].copy()
  res=least_squares(fun,np.zeros(4),max_nfev=150,ftol=1e-9,xtol=1e-9,gtol=1e-9)
  if np.linalg.norm(fun(res.x))<1e-6:
   name='grid_'+str(len(meta));out[name]=trans();meta[name]={'q_rad':dict(zip(jn,map(float,np.r_[a,b,res.x]))),'wheel_rel_m':(d.xpos[w]-d.xpos[hip]).tolist()}
Path('work/fusion_full_grid_transforms.json').write_text(json.dumps(out),encoding='utf8');Path('work/fusion_full_grid_metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf8');print('poses',len(out))
