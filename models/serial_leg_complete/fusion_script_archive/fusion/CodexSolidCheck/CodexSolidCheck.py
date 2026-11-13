import adsk.core,adsk.fusion,json,traceback,os

def run(context):
 app=adsk.core.Application.get();ui=app.userInterface;work=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\work';path=os.path.join(work,'fusion_scene_full_grid.json')
 try:
  root=adsk.fusion.Design.cast(app.activeProduct).rootComponent;occs={o.fullPathName:o for o in root.allOccurrences};prefix='腿总装 v6(镜像):1+腿部分(镜像):1+';inner=prefix+'小腿部分(镜像):1+'
  paths={'left_thigh':prefix+'大腿(镜像):1','left_output':inner+'小腿输出端(镜像):1','left_calf':inner+'小腿(镜像):1','left_longbar':inner+'长杆(镜像):1','left_midbar':inner+'中杆(镜像):1','left_shortbar':inner+'短杆(镜像):1'};links={k:occs[v].bRepBodies.item(0) for k,v in paths.items()};tm=adsk.fusion.TemporaryBRepManager.get();poses=json.load(open(os.path.join(work,'fusion_scene_full_transforms.json'),encoding='utf8'));wp='腿总装 v6(镜像):1+轮部分(镜像):1+';scene=[]
  def bounds(body):
   b=body.boundingBox;return list(b.minPoint.asArray()),list(b.maxPoint.asArray())
  def overlaps(x,y):return all(x[0][i]<=y[1][i] and y[0][i]<=x[1][i] for i in range(3))
  for name,o in occs.items():
   if not (name.startswith('baselink:1+') or name.startswith(wp)):continue
   if name.startswith('baselink:1+') and ('大腿链轮' in name or '小腿链轮' in name or ('三层' in name and '转接件' in name)):continue
   for b in o.bRepBodies:
    try:
     temp=tm.copy(b);scene.append((name,temp,'wheel' if name.startswith(wp) else 'base',bounds(temp)))
    except:pass
  data={'scene_count':len(scene),'results':{}}
  for label,pose in poses.items():
   moved={};boxes={}
   for k,b in links.items():
    t=tm.copy(b);mat=adsk.core.Matrix3D.create();mat.setWithArray(pose[k]);assert tm.transform(t,mat);moved[k]=t;boxes[k]=bounds(t)
   mat=adsk.core.Matrix3D.create();mat.setWithArray(pose['left_calf']);wheel=[];base=[]
   for name,b,k,box in scene:
    if k=='wheel':
     t=tm.copy(b);assert tm.transform(t,mat);wheel.append((name,t,bounds(t)))
    else:base.append((name,b,box))
   result=[];count=0
   def test(a,b,na,nb):
    cp=tm.copy(a);ok=tm.booleanOperation(cp,b,adsk.fusion.BooleanTypes.IntersectionBooleanType)
    if not ok:result.append({'a':na,'b':nb,'error':True})
    elif cp.volume>1e-9:result.append({'a':na,'b':nb,'volume_cm3':cp.volume})
   for k,a in moved.items():
    for name,b,box in base+([] if k=='left_calf' else wheel):
     if overlaps(boxes[k],box):test(a,b,k,name);count+=1
   for name,a,box in wheel:
    for bn,b,bb in base:
     if overlaps(box,bb):test(a,b,name,bn);count+=1
   data['results'][label]={'tested':count,'overlaps':result}
   if len(data['results'])%5==0 or len(data['results'])==len(poses):
    with open(path,'w',encoding='utf8') as f:json.dump(data,f,ensure_ascii=False)
  ui.messageBox('Full scene scan finished.')
 except:
  with open(path+'.error','w',encoding='utf8') as f:f.write(traceback.format_exc())
  ui.messageBox('Full scene scan failed; details saved.')
