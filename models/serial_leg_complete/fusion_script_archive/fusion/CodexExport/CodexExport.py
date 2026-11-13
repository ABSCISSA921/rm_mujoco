import adsk.core, adsk.fusion, json, traceback, os

def run(context):
    app=adsk.core.Application.get(); ui=app.userInterface
    out=r'C:\Users\Hydrogen\Documents\Codex\2026-10-27\https-bbs-robomaster-com-article-1942489\outputs\mjcf_export'
    os.makedirs(os.path.join(out,'meshes'),exist_ok=True)
    try:
        design=adsk.fusion.Design.cast(app.activeProduct); root=design.rootComponent
        targets={'base':'baselink:1','yaw':'yaw:1','pitch':'pitch (1):1'}
        for side,pre,mid in [('a','腿总装 v9:1','腿部分:1'),('b','腿总装 v6(镜像):1','腿部分(镜像):1')]:
            suffix='' if side=='a' else '(镜像)'
            targets[side+'_thigh']=pre+'+'+mid+'+'+'大腿'+suffix+':1'
            for en,cn in [('calf','小腿'),('output','小腿输出端'),('midbar','中杆'),('longbar','长杆'),('shortbar','短杆')]:
                targets[side+'_'+en]=pre+'+'+mid+'+'+'小腿部分'+suffix+':1+'+cn+suffix+':1'
            targets[side+'_wheel']=pre+'+'+'轮部分'+suffix+':1+'+'P13减速箱 v2'+suffix+':1+'+'转子'+suffix+':1'
        occs={o.fullPathName:o for o in root.allOccurrences}
        result={}
        for name,path in targets.items():
            if path not in occs:
                result[name]={'path':path,'error':'occurrence not found'}; continue
            try:
                fn=os.path.join(out,'meshes',name+'.stl')
                options=design.exportManager.createSTLExportOptions(occs[path],fn)
                options.isBinaryFormat=True
                options.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                ok=design.exportManager.execute(options)
                result[name]={'path':path,'ok':bool(ok),'file':fn,'bytes':os.path.getsize(fn) if os.path.exists(fn) else None,'transform':list(occs[path].transform2.asArray())}
            except: result[name]={'path':path,'error':traceback.format_exc()}
            with open(os.path.join(out,'export_manifest.json'),'w',encoding='utf-8') as f: json.dump(result,f,ensure_ascii=False,indent=2)
        ui.messageBox('Geometry export finished. Review manifest.')
    except:
        with open(os.path.join(out,'error.txt'),'w',encoding='utf-8') as f: f.write(traceback.format_exc())
        ui.messageBox('Geometry export failed; review error.txt')
