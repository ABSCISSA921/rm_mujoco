import adsk.core,adsk.fusion,json,traceback,os

def run(context):
 app=adsk.core.Application.get();ui=app.userInterface;work=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\work';out=os.path.join(work,'sprocket_range_audit.json')
 try:
  root=adsk.fusion.Design.cast(app.activeProduct).rootComponent;occs={o.fullPathName:o for o in root.allOccurrences}
  paths={'thigh':'腿总装 v6(镜像):1+腿部分(镜像):1+大腿(镜像):1','output':'腿总装 v6(镜像):1+腿部分(镜像):1+小腿部分(镜像):1+小腿输出端(镜像):1','thigh_sprocket':'baselink:1+中轴部分(镜像):1+二层(镜像):1+大腿链轮(镜像):1','output_sprocket':'baselink:1+中轴部分(镜像):1+三层(镜像):1+小腿链轮(镜像):1','adapter':'baselink:1+中轴部分(镜像):1+三层(镜像):1+转接件(镜像):1'}
  bodies={k:occs[v].bRepBodies.item(0) for k,v in paths.items()};tm=adsk.fusion.TemporaryBRepManager.get();poses=json.load(open(os.path.join(work,'sprocket_range_transforms.json'),encoding='utf8'));poses={'original':None,**poses};res={}
  for label,pose in poses.items():
   moved={}
   for name,b in bodies.items():
    t=tm.copy(b)
    if pose is not None:
     key='left_thigh' if name in ('thigh','thigh_sprocket') else 'left_output';mat=adsk.core.Matrix3D.create();mat.setWithArray(pose[key]);assert tm.transform(t,mat)
    moved[name]=t
   res[label]={}
   for a,b in [('thigh','output_sprocket')]:
    cp=tm.copy(moved[a]);ok=tm.booleanOperation(cp,moved[b],adsk.fusion.BooleanTypes.IntersectionBooleanType);res[label][a+'|'+b]={'ok':bool(ok),'volume_cm3':cp.volume if ok else None}
  with open(out,'w',encoding='utf8') as f:json.dump({'paths':paths,'results':res},f,ensure_ascii=False,indent=2)
  ui.messageBox('Sprocket audit complete.')
 except:
  with open(out+'.error','w',encoding='utf8') as f:f.write(traceback.format_exc())
  ui.messageBox('Sprocket audit failed.')

