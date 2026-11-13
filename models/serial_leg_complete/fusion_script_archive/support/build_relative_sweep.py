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

for direction,end in [('negative',-2.1),('positive',1.2)]:
 mujoco.mj_resetData(m,d);q=np.zeros(4)
 for i,b in enumerate(np.linspace(0,end,int(abs(end)/0.025)+1)):
  d.qpos[qi[0]]=0;d.qpos[qi[1]]=b
  def fun(q):d.qpos[qi[2:]]=q;mujoco.mj_forward(m,d);return d.efc_pos[6:12].copy()
  res=least_squares(fun,q,max_nfev=150,ftol=1e-11,xtol=1e-11,gtol=1e-11);assert np.linalg.norm(fun(res.x))<1e-6;q=res.x;key=direction+'_'+str(i);out[key]=trans();meta[key]={'q_rad':dict(zip(jn,map(float,np.r_[0,b,q]))),'wheel_rel_m':(d.xpos[w]-d.xpos[hip]).tolist()}
Path('work/fusion_relative_sweep_transforms.json').write_text(json.dumps(out),encoding='utf8');Path('work/fusion_relative_sweep_metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf8');print('poses',len(out))
