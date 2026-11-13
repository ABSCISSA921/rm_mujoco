import adsk.core,adsk.fusion,json,traceback,os

def run(context):
 app=adsk.core.Application.get();ui=app.userInterface;out=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\outputs\mjcf_export'
 try:
  design=adsk.fusion.Design.cast(app.activeProduct);occs=design.rootComponent.allOccurrences;found=[]
  for o in occs:
   path=o.fullPathName
   if path.startswith('baselink:1+') and ('大腿链轮' in path or '小腿链轮' in path or '外侧链轮' in path or '内侧链轮' in path):
    name=('b' if '镜像' in path else 'a')+('_thigh_sprocket' if '大腿链轮' in path else '_output_sprocket' if '小腿链轮' in path else '_outer_drive' if '外侧链轮' in path else '_inner_drive');fn=os.path.join(out,'meshes',name+'.stl');opt=design.exportManager.createSTLExportOptions(o,fn);opt.isBinaryFormat=True;opt.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium;ok=design.exportManager.execute(opt);found.append({'name':name,'path':path,'ok':bool(ok),'transform':list(o.transform2.asArray()),'bytes':os.path.getsize(fn) if os.path.exists(fn) else None})
  with open(os.path.join(out,'sprocket_manifest.json'),'w',encoding='utf8') as f:json.dump(found,f,ensure_ascii=False,indent=2)
  ui.messageBox('Sprocket inspection exported')
 except:
  with open(os.path.join(out,'sprocket_error.txt'),'w',encoding='utf8') as f:f.write(traceback.format_exc())
  ui.messageBox('Sprocket export failed')

