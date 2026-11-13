import adsk.core,adsk.fusion,json,traceback,os

def run(context):
 app=adsk.core.Application.get();ui=app.userInterface
 work=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\work'
 originals=[]; result={}
 try:
  design=adsk.fusion.Design.cast(app.activeProduct); occs=design.rootComponent.allOccurrences
  base=next(o for o in occs if o.fullPathName=='baselink:1')
  targets=[o for o in occs if o.fullPathName.startswith('baselink:1+') and ('大腿链轮' in o.fullPathName or '小腿链轮' in o.fullPathName or ('三层' in o.fullPathName and '转接件' in o.fullPathName))]
  if len(targets)!=6: raise RuntimeError('Expected 6 moving components, got '+str(len(targets)))
  for o in targets:
   originals.append((o,o.isLightBulbOn));o.isLightBulbOn=False
  
  for o in targets:
   if '转接件' in o.fullPathName:
    name=('b' if '镜像' in o.fullPathName else 'a')+'_hip_adapter'
    fname=os.path.join(work,name+'.stl');o.isLightBulbOn=True
    opt=design.exportManager.createSTLExportOptions(o,fname);opt.isBinaryFormat=True;opt.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium;design.exportManager.execute(opt);o.isLightBulbOn=False
    result[name]={'transform':list(o.transform2.asArray()),'path':o.fullPathName,'bytes':os.path.getsize(fname)}
  out=os.path.join(work,'base_no_hip_parts.stl');opt=design.exportManager.createSTLExportOptions(base,out);opt.isBinaryFormat=True;opt.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
  result.update({'ok':bool(design.exportManager.execute(opt)),'targets':[o.fullPathName for o in targets],'bytes':os.path.getsize(out)})
 except:
  result={'error':traceback.format_exc()}
 finally:
  for o,v in originals:
   try:o.isLightBulbOn=v
   except:result['restore_error']=traceback.format_exc()
  with open(os.path.join(work,'hip_adapter_result.json'),'w',encoding='utf8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
  ui.messageBox('Base export finished; visibility restored')
