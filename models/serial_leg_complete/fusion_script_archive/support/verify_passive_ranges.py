import json,numpy as np,mujoco
from scipy.optimize import least_squares
old=mujoco.MjModel.from_xml_path('outputs/mjcf_left_right/robot.xml');new=mujoco.MjModel.from_xml_path('outputs/mjcf_passive_limits_preview/robot_passive_limits.xml');a=mujoco.MjData(old);b=mujoco.MjData(new);z=json.load(open('work/fusion_boundary_ultrafine_metadata.json'))['fine_19']['q_rad']; names=['hip_thigh','hip_output','knee','longbar_joint','midbar_joint','shortbar_joint'];oi=[old.jnt_qposadr[mujoco.mj_name2id(old,mujoco.mjtObj.mjOBJ_JOINT,'left_'+n)] for n in names]; maxerr=0;maxviol=0
for rel in np.linspace(-2.52906602731,0,301):
 mujoco.mj_resetData(old,a);a.qpos[oi[:2]]=[z['hip_thigh'],z['hip_output']+rel]
 def f(q):a.qpos[oi[2:]]=q;mujoco.mj_forward(old,a);return a.efc_pos[6:12].copy()
 sol=least_squares(f,[z[n] for n in names[2:]],max_nfev=100,ftol=1e-11,gtol=1e-11,xtol=1e-11); assert np.linalg.norm(f(sol.x))<1e-7
 vals=np.r_[0,rel,sol.x-np.array([z[n] for n in names[2:]])];mujoco.mj_resetData(new,b)
 for side in ('left','right'):
  for n,v in zip(names,vals):
   j=mujoco.mj_name2id(new,mujoco.mjtObj.mjOBJ_JOINT,side+'_'+n); b.qpos[new.jnt_qposadr[j]]=v
   if new.jnt_limited[j]:maxviol=max(maxviol,float(new.jnt_range[j,0]-v),float(v-new.jnt_range[j,1]))
 mujoco.mj_forward(new,b);maxerr=max(maxerr,float(np.max(abs(b.efc_pos[:12]))))
assert maxviol<1e-7 and maxerr<1e-6,(maxviol,maxerr)
print('301 poses, both legs; max limit violation rad',maxviol,'max closure residual m',maxerr)
