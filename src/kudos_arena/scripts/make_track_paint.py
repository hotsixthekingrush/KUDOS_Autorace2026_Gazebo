#!/usr/bin/env python3
"""Convert reference-image markings into portable, flat COLLADA meshes."""
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / "models" / "track_paint"


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def write_mesh(mask, output, color):
    height, width = mask.shape
    vertices = []

    # Each horizontal run becomes a rectangle made of two triangles.
    # This avoids creating thousands of separate Gazebo models.
    for row in range(height):
        values = np.pad((mask[row] > 0).astype(np.int8), (1, 1))
        changes = np.diff(values)
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1)

        for start, end in zip(starts, ends):
            x0 = float(start) / width * 10.0
            x1 = float(end) / width * 10.0
            y0 = 6.0 - float(row + 1) / height * 6.0
            y1 = 6.0 - float(row) / height * 6.0
            vertices.extend([
                (x0, y0, 0), (x1, y0, 0), (x1, y1, 0),
                (x0, y0, 0), (x1, y1, 0), (x0, y1, 0),
            ])

    if not vertices:
        raise ValueError(f"No markings found for {output.name}")

    root = ET.Element(
        "COLLADA",
        xmlns="http://www.collada.org/2005/11/COLLADASchema",
        version="1.4.1",
    )
    asset = element(root, "asset")
    element(asset, "created", "2026-10-27T00:00:00Z")
    element(asset, "modified", "2026-10-27T00:00:00Z")
    element(asset, "unit", name="meter", meter="1")
    element(asset, "up_axis", "Z_UP")

    effects = element(root, "library_effects")
    effect = element(effects, "effect", id="paint-effect")
    profile = element(effect, "profile_COMMON")
    technique = element(profile, "technique", sid="common")
    lambert = element(technique, "lambert")
    element(element(lambert, "ambient"), "color", color)
    element(element(lambert, "diffuse"), "color", color)

    materials = element(root, "library_materials")
    material = element(materials, "material", id="paint-material")
    element(material, "instance_effect", url="#paint-effect")

    geometries = element(root, "library_geometries")
    geometry = element(geometries, "geometry", id="paint-geometry")
    mesh = element(geometry, "mesh")

    source = element(mesh, "source", id="positions")
    element(
        source, "float_array",
        " ".join(f"{value:.7f}" for vertex in vertices for value in vertex),
        id="positions-array", count=str(len(vertices) * 3),
    )
    accessor = element(
        element(source, "technique_common"), "accessor",
        source="#positions-array", count=str(len(vertices)), stride="3",
    )
    for axis in ("X", "Y", "Z"):
        element(accessor, "param", name=axis, type="float")

    normal = element(mesh, "source", id="normals")
    element(normal, "float_array", "0 0 1", id="normals-array", count="3")
    accessor = element(
        element(normal, "technique_common"), "accessor",
        source="#normals-array", count="1", stride="3",
    )
    for axis in ("X", "Y", "Z"):
        element(accessor, "param", name=axis, type="float")

    vertex_node = element(mesh, "vertices", id="vertices")
    element(vertex_node, "input", semantic="POSITION", source="#positions")
    triangles = element(
        mesh, "triangles",
        material="paint", count=str(len(vertices) // 3),
    )
    element(
        triangles, "input",
        semantic="VERTEX", source="#vertices", offset="0",
    )
    element(
        triangles, "input",
        semantic="NORMAL", source="#normals", offset="1",
    )
    element(
        triangles, "p",
        " ".join(f"{index} 0" for index in range(len(vertices))),
    )

    scenes = element(root, "library_visual_scenes")
    scene = element(scenes, "visual_scene", id="Scene")
    node = element(scene, "node", id="paint-node")
    instance = element(node, "instance_geometry", url="#paint-geometry")
    bindings = element(element(instance, "bind_material"), "technique_common")
    element(
        bindings, "instance_material",
        symbol="paint", target="#paint-material",
    )
    element(element(root, "scene"), "instance_visual_scene", url="#Scene")

    ET.indent(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    print(f"{output.name}: {len(vertices) // 3} triangles")


def main():
    image = cv2.imread(str(MODEL / "reference" / "track.png"))
    if image is None:
        raise SystemExit("Reference PNG could not be read.")
    if image.shape[:2] != (776, 1286):
        raise SystemExit("Reference image size changed; update the crop first.")

    # Inside the drawing's outer frame; exclude the frame itself.
    image = image[9:770, 9:1277]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    yellow = cv2.inRange(hsv, (10, 90, 90), (45, 255, 255))
    white = cv2.inRange(hsv, (0, 0, 140), (179, 65, 255))

    meshes = MODEL / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    write_mesh(yellow, meshes / "yellow.dae", "1 0.75 0 1")
    write_mesh(white, meshes / "white.dae", "0.95 0.95 0.90 1")


if __name__ == "__main__":
    main()
