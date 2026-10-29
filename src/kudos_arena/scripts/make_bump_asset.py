#!/usr/bin/env python3
"""Unit trapezoid bump; SDF scale specifies actual length, width and height."""
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'models/speed_bump'
(p/'meshes').mkdir(parents=True,exist_ok=True)
# Cross-section in XZ: 40% ascent, 20% flat, 40% descent.
cross=[(-.5,0),(.5,0),(.1,1),(-.1,1)]
v=[(x,y,z) for y in (-.5,.5) for x,z in cross]
faces=[(0,1,2),(0,2,3),(4,6,5),(4,7,6)]
for i in range(4):
 j=(i+1)%4;faces.extend([(i,i+4,j+4),(i,j+4,j)])
lines=['solid speed_bump']
for face in faces:
 a,b,c=[v[i] for i in face];u=[b[i]-a[i] for i in range(3)];w=[c[i]-a[i] for i in range(3)]
 n=[u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];length=sum(x*x for x in n)**.5
 lines+=['facet normal '+' '.join(str(x/length) for x in n),'outer loop']
 lines+=['vertex '+' '.join(map(str,x)) for x in (a,b,c)];lines+=['endloop','endfacet']
lines+=['endsolid speed_bump'];(p/'meshes/bump.stl').write_text('\n'.join(lines)+'\n')
(p/'model.config').write_text('<model><name>speed_bump</name><version>1.0</version><sdf version="1.6">model.sdf</sdf><description>Scaled ramp asset</description></model>')
(p/'model.sdf').write_text('<sdf version="1.6"><model name="speed_bump"><static>true</static><link name="bump"><visual name="visual"><geometry><mesh><uri>model://speed_bump/meshes/bump.stl</uri><scale>0.3 0.5 0.015</scale></mesh></geometry></visual></link></model></sdf>')
