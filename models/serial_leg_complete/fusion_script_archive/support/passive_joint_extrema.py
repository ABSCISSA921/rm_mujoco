import json,numpy as np,mujoco
from scipy.optimize import least_squares, minimize_scalar
from pathlib import Path
m=mujoco.MjModel.from_xml_path('outputs/mjcf_left_right/robot.xml');d=mujoco.MjData(m);z=json.load(open('work/fusion_boundary_ultrafine_metadata.json',encoding='utf8'))['fine_19']['q_rad'];end=json.load(open('work/fusion_relative_fine_metadata.json',encoding='utf8'))['negative_11']['q_rad'];names=['hip_thigh','hip_output','knee','longbar_joint','midbar_joint','shortbar_joint'];idx=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'left_'+n)] for n in names]; delta=end['hip_output']-end['hip_thigh']-(z['hip_output']-z['hip_thigh']); count=0
# Match the same assembly branch at the shortened endpoint, while common hip rotation is held at zero.
def solve(t):
 global count
 mujoco.mj_resetData(m,d);d.qpos[idx[0]]=z['hip_thigh'];d.qpos[idx[1]]=z['hip_output']+t
 def fn(q):
  d.qpos[idx[2:]]=q;mujoco.mj_forward(m,d);return d.efc_pos[6:12].copy()
 x=least_squares(fn,[z[n] for n in names[2:]],max_nfev=100,ftol=1e-12,xtol=1e-12,gtol=1e-12);err=np.linalg.norm(fn(x.x));assert err<1e-7,(t,err); count+=1;return x.x-np.array([z[n] for n in names[2:]])
res=minimize_scalar(lambda t:solve(t)[2],bounds=(delta,0),method='bounded',options={'xatol':1e-12});print('minimum',res.x,res.fun,'calls',count,'closure angles',solve(res.x).tolist());print('end',solve(delta).tolist(),'zero',solve(0).tolist());
Path('work/passive_joint_extrema.json').write_text(json.dumps({'relative_range':[delta,0],'midbar_min_relative':float(res.x),'midbar_min':float(res.fun),'end_angles':solve(delta).tolist(),'zero_angles':solve(0).tolist(),'method':'bounded numerical optimization on selected MuJoCo closure branch; Fusion main-link interference endpoints','limits':'no exact continuous-interference or off-branch proof'},indent=2),encoding='utf8')
