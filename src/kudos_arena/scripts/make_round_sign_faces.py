#!/usr/bin/env python3
"""Circular UV-mapped faces; preserve original PNGs without image editing."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / 'models'
NS = 'http://www.collada.org/2005/11/COLLADASchema'
ET.register_namespace('', NS)
Q = lambda name: '{' + NS + '}' + name


def main():
    positions, uv = [], []
    count = 96
    for i in range(count):
        # CCW in the YZ plane gives a +X front normal.
        for yz in [(0., 0.),
                   (math.cos(2*math.pi*i/count), math.sin(2*math.pi*i/count)),
                   (math.cos(2*math.pi*(i+1)/count), math.sin(2*math.pi*(i+1)/count))]:
            y, z = yz
            positions.extend((0., .112*y, .112*z))
            uv.extend(((y+1)/2, (z+1)/2))
    for kind in ('A', 'B', 'slow', 'parking'):
        folder = ROOT / ('sign_' + kind) / 'meshes'
        tree = ET.parse(folder / 'panel.dae')
        mesh = tree.getroot().find('.//' + Q('mesh'))
        for id_, data, stride in [('positions', positions, 3), ('uv', uv, 2)]:
            source = mesh.find(Q('source') + "[@id='" + id_ + "']")
            array = source.find(Q('float_array'))
            array.set('count', str(len(data)))
            array.text = ' '.join(f'{v:.9f}' for v in data)
            source.find('.//' + Q('accessor')).set('count', str(len(data)//stride))
        triangles = mesh.find(Q('triangles'))
        triangles.set('count', str(count))
        triangles.find(Q('p')).text = ' '.join(f'{i} 0 {i}' for i in range(count*3))
        tree.write(folder / 'round_face.dae', encoding='utf-8', xml_declaration=True)
        print('Generated circular face:', kind)


if __name__ == '__main__':
    main()
