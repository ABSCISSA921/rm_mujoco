import adsk.core, adsk.fusion, json, traceback, os

def run(context):
    app=adsk.core.Application.get(); ui=app.userInterface
    out=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\work\fusion_inspect.json'
    os.makedirs(os.path.dirname(out),exist_ok=True)
    def point(v):
        try: return list(v.asArray())
        except: return None
    def infojoint(j):
        d={'name':j.name,'objectType':j.objectType}
        for k in ['occurrenceOne','occurrenceTwo']:
            try:
                v=getattr(j,k); d[k]=v.fullPathName if v else None
            except Exception as e: d[k]=str(e)
        for k in ['jointMotion','geometryOrOriginOne','geometryOrOriginTwo','geometryOne','geometryTwo']:
            try:
                v=getattr(j,k); d[k+'_type']=v.objectType if v else None
                if k=='jointMotion' and v:
                    d['jointType']=v.jointType
                    for a in ['rotationAxisVector','slideDirectionVector']:
                        try: d[a]=point(getattr(v,a))
                        except: pass
                if v:
                    for a in ['origin','primaryAxisVector','secondaryAxisVector','thirdAxisVector']:
                        try: d[k+'_'+a]=point(getattr(v,a))
                        except: pass
            except Exception: pass
        return d
    try:
        design=adsk.fusion.Design.cast(app.activeProduct); root=design.rootComponent
        result={'root':root.name,'occurrences':[],'components':[]}
        for o in root.allOccurrences:
            c=o.component
            result['occurrences'].append({'path':o.fullPathName,'component':c.name,'bodies':c.bRepBodies.count,'transform':list(o.transform2.asArray())})
        for c in design.allComponents:
            d={'name':c.name,'bodies':c.bRepBodies.count,'joints':[],'asBuiltJoints':[]}
            for j in c.joints: d['joints'].append(infojoint(j))
            for j in c.asBuiltJoints: d['asBuiltJoints'].append(infojoint(j))
            result['components'].append(d)
        with open(out,'w',encoding='utf-8') as f: json.dump(result,f,ensure_ascii=False,indent=2)
        ui.messageBox('Inspection saved')
    except:
        with open(out+'.error','w',encoding='utf-8') as f: f.write(traceback.format_exc())
        ui.messageBox('Inspection failed; see work folder')
