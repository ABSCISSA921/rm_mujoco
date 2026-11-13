import adsk.core,adsk.fusion,json,traceback,os,itertools

def run(context):
 app=adsk.core.Application.get();ui=app.userInterface;work=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\work';path=os.path.join(work,'fusion_relative_sweep.json')
 try:
  root=adsk.fusion.Design.cast(app.activeProduct).rootComponent;occs={o.fullPathName:o for o in root.allOccurrences};prefix='腿总装 v6(镜像):1+腿部分(镜像):1+';inner=prefix+'小腿部分(镜像):1+';paths={'left_thigh':prefix+'大腿(镜像):1','left_output':inner+'小腿输出端(镜像):1','left_calf':inner+'小腿(镜像):1','left_longbar':inner+'长杆(镜像):1','left_midbar':inner+'中杆(镜像):1','left_shortbar':inner+'短杆(镜像):1'};bodies={k:occs[v].bRepBodies.item(0) for k,v in paths.items()};tm=adsk.fusion.TemporaryBRepManager.get();poses=json.load(open(os.path.join(work,'fusion_relative_sweep_transforms.json'),encoding='utf8'));data={'results':{}}
  for label,pose in poses.items():
   moved={}
   for k,b in bodies.items():
    t=tm.copy(b);mat=adsk.core.Matrix3D.create();mat.setWithArray(pose[k]);assert tm.transform(t,mat);moved[k]=t
   result={}
   for a,b in itertools.combinations(bodies,2):
    cp=tm.copy(moved[a]);ok=tm.booleanOperation(cp,moved[b],adsk.fusion.BooleanTypes.IntersectionBooleanType);result[a+'|'+b]={'ok':bool(ok),'volume_cm3':cp.volume if ok else None}
   data['results'][label]=result
   if len(data['results'])%10==0 or len(data['results'])==len(poses):
    with open(path,'w',encoding='utf8') as f:json.dump(data,f,ensure_ascii=False)
  ui.messageBox('Relative sweep finished.')
 except:
  with open(path+'.error','w',encoding='utf8') as f:f.write(traceback.format_exc())
  ui.messageBox('Relative sweep failed; details saved.')
