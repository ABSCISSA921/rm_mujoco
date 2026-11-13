import adsk.core, adsk.fusion, json, traceback, os

def run(context):
 app=adsk.core.Application.get();ui=app.userInterface;out=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\outputs\mjcf_export'
 try:
  design=adsk.fusion.Design.cast(app.activeProduct); occs={o.fullPathName:o for o in design.rootComponent.allOccurrences}; targets={}
  for side,pre,suf in [('a','腿总装 v9:1',''),('b','腿总装 v6(镜像):1','(镜像)')]:
   ax=pre+'+腿部分'+suf+':1+轴系'+suf+':1+'
   for en,cn,owner in [('output_mid','小腿输出端和中杆轴系','output'),('mid_long','中杆和长杆轴系','midbar'),('thigh_long','大腿和长杆轴系','thigh'),('long_short','长杆和短杆轴系','longbar'),('knee','膝关节轴系','thigh'),('short_calf','短杆和小腿轴系','calf')]:
    targets[side+'_hardware_'+en]=(ax+cn+suf+':1',side+'_'+owner)
   wheel=pre+'+轮部分'+suf+':1+P13减速箱 v2'+suf+':1+'
   for en,cn in [('stator','定子'),('gears','齿轮组'),('cover','防尘盖'),('standard','标准件')]: targets[side+'_wheel_'+en]=(wheel+cn+suf+':1',side+'_calf')
  result={}
  for name,(path,owner) in targets.items():
   if path not in occs: result[name]={'path':path,'owner':owner,'error':'not found'};continue
   try:
    fn=os.path.join(out,'meshes',name+'.stl');o=occs[path]; opt=design.exportManager.createSTLExportOptions(o,fn);opt.isBinaryFormat=True;opt.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
    ok=design.exportManager.execute(opt);result[name]={'path':path,'owner':owner,'ok':bool(ok),'file':fn,'bytes':os.path.getsize(fn) if os.path.exists(fn) else None,'transform':list(o.transform2.asArray())}
   except:result[name]={'path':path,'owner':owner,'error':traceback.format_exc()}
   with open(os.path.join(out,'hardware_manifest.json'),'w',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
  ui.messageBox('Hardware export finished')
 except:
  with open(os.path.join(out,'hardware_error.txt'),'w',encoding='utf-8') as f:f.write(traceback.format_exc())
  ui.messageBox('Hardware export failed')
