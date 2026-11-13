import mujoco,numpy as np
m=mujoco.MjModel.from_xml_path('outputs/mjcf_passive_limits_preview/robot_passive_limits.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d)
for side in ['left','right']:
 w=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,side+'_wheel');j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,side+'_hip_thigh');hip=d.xanchor[j];print(side,'wheel minus hip',d.xpos[w]-hip,'hip axis',d.xaxis[j]);print('tendon',m.tendon_range[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_TENDON,side+'_leg_relative_stop')])
for q in [0,1,3.14159,6.28318,12.56636,-6.28318]:
 mujoco.mj_resetData(m,d)
 for side in ['left','right']:
  for n in ['hip_thigh','hip_output']:
   j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,side+'_'+n);d.qpos[m.jnt_qposadr[j]]=q
 mujoco.mj_forward(m,d); bad=[i for i in range(d.nefc) if d.efc_type[i] in [mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT,mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON]];print('common rotation',q,'closure',np.max(abs(d.efc_pos[:12])),'active limits',len(bad));assert np.max(abs(d.efc_pos[:12]))<1e-8 and not bad
