import adsk.core, adsk.fusion, json, traceback, os

def run(context):
    app=adsk.core.Application.get(); ui=app.userInterface
    out=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\work\gimbal_axes.json'
    try:
        root=adsk.fusion.Design.cast(app.activeProduct).rootComponent; data=[]
        for o in root.allOccurrences:
            path=o.fullPathName
            if not (path.startswith('yaw:1+') or path.startswith('pitch (1):1+')): continue
            if not ('推力球' in path or ('pitch:1+' in path and 'pitch右连杆' in path) or '云台臂' in path): continue
            if o.bRepBodies.count==0: continue
            row={'path':path,'cylinders':[]}
            for b in o.bRepBodies:
                for f in b.faces:
                    g=f.geometry
                    if g.objectType=='adsk::core::Cylinder':
                        bb=f.boundingBox
                        row['cylinders'].append({'origin':list(g.origin.asArray()),'axis':list(g.axis.asArray()),'radius':g.radius,'area':f.area,'min':list(bb.minPoint.asArray()),'max':list(bb.maxPoint.asArray())})
            if row['cylinders']: data.append(row)
        with open(out,'w',encoding='utf-8') as f: json.dump(data,f,ensure_ascii=False,indent=2)
        ui.messageBox('Axis inspection saved')
    except:
        with open(out+'.error','w',encoding='utf-8') as f: f.write(traceback.format_exc())
        ui.messageBox('Axis inspection failed')



