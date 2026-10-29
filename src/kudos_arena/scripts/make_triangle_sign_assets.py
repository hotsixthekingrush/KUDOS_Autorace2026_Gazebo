#!/usr/bin/env python3
"""Triangular sign prisms and UV faces, using unchanged original PNGs."""
from pathlib import Path
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1] / 'models'
NS = 'http://www.collada.org/2005/11/COLLADASchema'
E.register_namespace('', NS)
Q = lambda s: '{'+NS+'}'+s


def main():
    # 30 cm wide backing, 27 cm tall, 1 cm thick; +X is the front.
    yz = [(-.15, -.135), (.15, -.135), (0, .135)]
    vertices = [(x,y,z) for x in (-.005,.005) for y,z in yz]
    faces = [(3,4,5),(2,1,0),(0,1,4),(0,4,3),
             (1,2,5),(1,5,4),(2,0,3),(2,3,5)]
    lines = ['solid triangle_sign']
    for face in faces:
        a,b,c = [vertices[i] for i in face]
        u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
        n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        norm=sum(x*x for x in n)**.5
        lines += ['facet normal '+' '.join(str(x/norm) for x in n),'outer loop']
        lines += ['vertex '+' '.join(map(str,p)) for p in (a,b,c)]
        lines += ['endloop','endfacet']
    lines += ['endsolid triangle_sign']
    for kind in ('pedestrian','bump'):
        folder=ROOT/('sign_'+kind)
        (folder/'meshes/triangle_body.stl').write_text('\n'.join(lines)+'\n')
        tree=E.parse(ROOT/'sign_A/meshes/panel.dae')
        mesh=tree.getroot().find('.//'+Q('mesh'))
        for id_,data,stride in [
            ('positions',[0,-.14,-.125, 0,.14,-.125, 0,0,.125],3),
            ('uv',[0,0, 1,0, .5,1],2)]:
            source=mesh.find(Q('source')+f"[@id='{id_}']")
            array=source.find(Q('float_array'));array.set('count',str(len(data)))
            array.text=' '.join(map(str,data))
            source.find('.//'+Q('accessor')).set('count',str(len(data)//stride))
        triangles=mesh.find(Q('triangles'));triangles.set('count','1')
        triangles.find(Q('p')).text='0 0 0 1 0 1 2 0 2'
        tree.getroot().find('.//'+Q('image')+'/'+Q('init_from')).text=f'../materials/textures/kudos_sign_{kind}.png'
        tree.write(folder/'meshes/triangle_face.dae',encoding='utf-8',xml_declaration=True)
        (folder/'model.config').write_text(f'<model><name>sign_{kind}</name><version>1.0</version><sdf version="1.6">model.sdf</sdf><description>Triangular warning sign asset</description></model>')
        (folder/'model.sdf').write_text(f'<sdf version="1.6"><model name="sign_{kind}"><static>true</static><link name="panel"><visual name="face"><geometry><mesh><uri>model://sign_{kind}/meshes/triangle_face.dae</uri></mesh></geometry></visual></link></model></sdf>')
        print('Generated:',kind)


if __name__ == '__main__':
    main()
