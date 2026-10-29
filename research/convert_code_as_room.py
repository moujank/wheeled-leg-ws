"""Blender batch conversion of an existing local GLB, no generation or source writes."""
import argparse
import json
import sys
from pathlib import Path
import bpy

p=argparse.ArgumentParser()
p.add_argument('--input',required=True,type=Path)
p.add_argument('--output',required=True,type=Path)
a=p.parse_args(sys.argv[sys.argv.index('--')+1:])
a.output.parent.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=str(a.input.resolve()))
mesh_count=sum(o.type=='MESH' for o in bpy.data.objects)
kwargs={'filepath':str(a.output.resolve()),'selected_objects_only':False,
        'export_materials':True,'export_textures':True,'relative_paths':True,
        'root_prim_path':'/World'}
supported={p.identifier for p in bpy.ops.wm.usd_export.get_rna_type().properties}
result=bpy.ops.wm.usd_export(**{k:v for k,v in kwargs.items() if k in supported})
if 'FINISHED' not in result:
    raise RuntimeError(f'USD export failed: {result}')
(a.output.parent/'conversion.json').write_text(json.dumps({'source':str(a.input.resolve()),
    'usd':str(a.output.resolve()),'meshes':mesh_count,'blender':bpy.app.version_string,
    'source_modified':False,'scope':'GLB to USD appearance conversion; collisions added separately'},indent=2),encoding='utf-8')
print('ROOM_CONVERTED '+str(a.output.resolve()),flush=True)
