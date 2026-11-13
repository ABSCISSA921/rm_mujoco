from pathlib import Path
import xml.etree.ElementTree as ET, shutil, json, mujoco
src=Path('outputs/mjcf_limit_guard_candidate');dest=Path('outputs/mjcf_passive_limits_preview');dest.mkdir(exist_ok=True);shutil.copytree(src/'meshes',dest/'meshes',dirs_exist_ok=True);tree=ET.parse(src/'robot_guard_candidate.xml');root=tree.getroot();r=json.load(open('work/passive_joint_extrema.json'));e=r['end_angles'];pad=0.001
ranges={'knee':(e[0],0),'longbar_joint':(min(e[1],0)-pad,max(e[1],0)+pad),'midbar_joint':(r['midbar_min']-pad,max(e[2],0)+pad),'shortbar_joint':(min(e[3],0)-pad,max(e[3],0)+pad)}
for side in ('left','right'):
 for n,(lo,hi) in ranges.items():
  j=root.find('.//joint[@name="'+side+'_'+n+'"]');assert j is not None;j.set('limited','true');j.set('range',f'{lo:.12g} {hi:.12g}')
 for n in ('hip_thigh','hip_output'):
  j=root.find('.//joint[@name="'+side+'_'+n+'"]');assert j.get('limited') not in ('true','1') and 'range' not in j.attrib
ET.indent(root,space='  ');tree.write(dest/'robot_passive_limits.xml',encoding='utf8',xml_declaration=True);m=mujoco.MjModel.from_xml_path(str(dest/'robot_passive_limits.xml'));print('loaded',m.njnt,'ranges',ranges)
