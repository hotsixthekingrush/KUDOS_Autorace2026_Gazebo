#!/usr/bin/env python3
"""Build portable sign models using the user's unmodified PNG images."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / 'models'

# Front normal +X, upright +Z; UVs preserve readable left/right arrows.
DAE = '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-10-28T00:00:00Z</created><modified>2026-10-28T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="sign-image"><init_from>../materials/textures/sign.png</init_from></image></library_images>
<library_effects><effect id="sign-effect"><profile_COMMON>
<newparam sid="surface"><surface type="2D"><init_from>sign-image</init_from></surface></newparam>
<newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
<technique sid="common"><lambert><ambient><color>0.5 0.5 0.5 1</color></ambient><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse></lambert></technique>
</profile_COMMON></effect></library_effects>
<library_materials><material id="sign-material"><instance_effect url="#sign-effect"/></material></library_materials>
<library_geometries><geometry id="panel"><mesh>
<source id="positions"><float_array id="positions-array" count="12">0.006 -0.12 -0.12 0.006 0.12 -0.12 0.006 0.12 0.12 0.006 -0.12 0.12</float_array><technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="normals"><float_array id="normals-array" count="3">1 0 0</float_array><technique_common><accessor source="#normals-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv-array" count="8">0 0 1 0 1 1 0 1</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="2" material="paint"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="panel-node"><instance_geometry url="#panel"><bind_material><technique_common><instance_material symbol="paint" target="#sign-material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>
'''


def add(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def main():
    for kind in ('A', 'B', 'slow', 'parking'):
        name = 'sign_' + kind
        folder = ROOT / name
        assert (folder / 'materials/textures/sign.png').is_file()
        (folder / 'meshes').mkdir(exist_ok=True)
        (folder / 'meshes/panel.dae').write_text(DAE)
        root = ET.Element('sdf', version='1.6')
        model = add(root, 'model', name=name)
        add(model, 'static', 'true')
        link = add(model, 'link', name='sign_link')
        # Full-height stand, weighted base and panel backing are solid.
        for part, pose, size in (
            ('base', '0 0 0.01 0 0 0', '0.10 0.10 0.02'),
            ('pole', '0 0 0.18 0 0 0', '0.02 0.02 0.36'),
            ('backing', '0 0 0.36 0 0 0', '0.01 0.24 0.24'),
        ):
            for tag in ('visual', 'collision'):
                e = add(link, tag, name=part+'_'+tag)
                add(e, 'pose', pose)
                add(add(add(e, 'geometry'), 'box'), 'size', size)
                if tag == 'visual':
                    material = add(e, 'material')
                    add(material, 'ambient', '0.25 0.25 0.25 1')
                    add(material, 'diffuse', '0.25 0.25 0.25 1')
        front = add(link, 'visual', name='printed_face')
        add(front, 'pose', '0 0 0.36 0 0 0')
        add(add(add(front, 'geometry'), 'mesh'), 'uri', f'model://{name}/meshes/panel.dae')
        ET.indent(root)
        ET.ElementTree(root).write(folder/'model.sdf', encoding='utf-8', xml_declaration=True)
        config = ET.Element('model')
        add(config, 'name', name)
        add(config, 'version', '1.0')
        add(config, 'sdf', 'model.sdf', version='1.6')
        add(config, 'description', 'User image on a 24 cm square sign with stand.')
        ET.ElementTree(config).write(folder/'model.config', encoding='utf-8', xml_declaration=True)
        print('Generated', name)


if __name__ == '__main__':
    main()
