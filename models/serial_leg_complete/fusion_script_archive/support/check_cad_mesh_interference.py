import mujoco, numpy as np, fcl, json, time
from pathlib import Path
m=mujoco.MjModel.from_xml_path('outputs/mjcf_left_right/robot.xml'); d=mujoco.MjData(m); qdata=json.loads(Path('work/zero_pose_kinematic_probe.json').read_text());names=['hip_thigh','hip_output','knee','longbar_joint','midbar_joint','shortbar_joint'];objects={}; gi=[]
for g in range(m.ngeom):
 n=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g)
 if '_visual' not in n or m.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:continue
 if n.startswith('right_'):continue
 ix=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[ix]:m.mesh_vertadr[ix]+m.mesh_vertnum[ix]].astype(np.float64).copy(); f=m.mesh_face[m.mesh_faceadr[ix]:m.mesh_faceadr[ix]+m.mesh_facenum[ix]].astype(np.int32).copy();geom=fcl.BVHModel();geom.beginModel(len(v),len(f));geom.addSubModel(v,f);geom.endModel();objects[g]=fcl.CollisionObject(geom);gi.append(g)
print('meshes',len(gi),flush=True)
def check(index):
 mujoco.mj_resetData(m,d); s=qdata['left']['samples'][index]
 for n in names:
  j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'left_'+n);d.qpos[m.jnt_qposadr[j]]=s['q_rad'][n]
 mujoco.mj_forward(m,d)
 for g,o in objects.items():o.setTransform(fcl.Transform(d.geom_xmat[g].reshape(3,3).copy(),d.geom_xpos[g].copy()))
 result=[]
 for i,g in enumerate(gi):
  for h in gi[i+1:]:
   bg=m.geom_bodyid[g];bh=m.geom_bodyid[h]
   if bg==bh:continue
   ng=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g);nh=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,h)
   if not (ng.startswith('left_') or nh.startswith('left_')):continue
   req=fcl.CollisionRequest(num_max_contacts=1,enable_contact=True);res=fcl.CollisionResult();n=fcl.collide(objects[g],objects[h],req,res)
   if n:result.append({'pair':[ng,nh],'depth':float(res.contacts[0].penetration_depth) if res.contacts else None})
 return {'index':index,'z_m':s['wheel_relative_z_m'],'contacts':result}
N=len(qdata['left']['samples']); indices=sorted(set([0,*[round((N-1)*i/20) for i in range(1,21)]]));out=[]
for index in indices:
 x=check(index);out.append(x);print(index,round(x['z_m'],4),len(x['contacts']),flush=True)
Path('work/cad_mesh_interference_probe.json').write_text(json.dumps({'method':'FCL triangle mesh intersection from Fusion visual meshes at vertical axle poses, left side; existing intersections may be intentional, no full CAD motion assembly','samples':out},indent=2),encoding='utf8')

