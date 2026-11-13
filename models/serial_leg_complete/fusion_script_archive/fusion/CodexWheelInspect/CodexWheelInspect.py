import adsk.core,adsk.fusion,json,traceback,os

def run(context):
 app=adsk.core.Application.get();ui=app.userInterface;work=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\work';result={};hidden=[]
 try:
  design=adsk.fusion.Design.cast(app.activeProduct);occs=list(design.rootComponent.allOccurrences)
  targets=[o for o in occs if 'P13' in o.fullPathName and 'M0.6 T75' in o.fullPathName and o.component.bRepBodies.count]
  if len(targets)!=2:raise RuntimeError('expected 2 gears, got '+str(len(targets)))
  for o in targets:
   side='b' if '镜像' in o.fullPathName else 'a';fn=os.path.join(work,side+'_wheel_gear75.stl');opt=design.exportManager.createSTLExportOptions(o,fn);opt.isBinaryFormat=True;opt.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium;ok=design.exportManager.execute(opt);result[side]={'path':o.fullPathName,'transform':list(o.transform2.asArray()),'ok':bool(ok),'bytes':os.path.getsize(fn)}
  for o in targets:
   side='b' if '镜像' in o.fullPathName else 'a';parent=o.assemblyContext;hidden.append((o,o.isLightBulbOn));o.isLightBulbOn=False
   try:
    fn=os.path.join(work,side+'_wheel_gears_without75.stl');opt=design.exportManager.createSTLExportOptions(parent,fn);opt.isBinaryFormat=True;opt.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium;result[side]['group_ok']=bool(design.exportManager.execute(opt));result[side]['group_bytes']=os.path.getsize(fn)
   finally:o.isLightBulbOn=hidden[-1][1]
 except:result['error']=traceback.format_exc()
 finally:
  for o,v in hidden:
   try:o.isLightBulbOn=v
   except:result['restore_error']=traceback.format_exc()
 with open(os.path.join(work,'wheel_gear75_result.json'),'w',encoding='utf8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
 ui.messageBox('Wheel gear inspection finished')
