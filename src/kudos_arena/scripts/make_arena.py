#!/usr/bin/env python3
"""Generate a Gazebo Classic world from the arena configuration."""

import argparse
import json
import math
import secrets
from pathlib import Path
import xml.etree.ElementTree as ET


def add(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def add_box(world, name, pose, size, rgba, collision=True):
    model = add(world, "model", name=name)
    add(model, "static", "true")
    add(model, "pose", " ".join(map(str, pose)))
    link = add(model, "link", name="link")

    visual = add(link, "visual", name="visual")
    geometry = add(visual, "geometry")
    box = add(geometry, "box")
    add(box, "size", " ".join(map(str, size)))

    material = add(visual, "material")
    add(material, "ambient", rgba)
    add(material, "diffuse", rgba)

    if collision:
        collider = add(link, "collision", name="collision")
        geometry = add(collider, "geometry")
        box = add(geometry, "box")
        add(box, "size", " ".join(map(str, size)))

    return model


def build_world(config):
    sdf = ET.Element("sdf", version="1.6")
    world = add(sdf, "world", name=config["world_name"])
    add(world, "gravity", "0 0 -9.81")

    physics = add(world, "physics", name="default_physics", type="ode")
    add(physics, "max_step_size", "0.001")
    add(physics, "real_time_update_rate", "1000")

    scene = add(world, "scene")
    add(scene, "ambient", "0.6 0.6 0.6 1")
    add(scene, "background", "0.75 0.80 0.85 1")
    add(scene, "shadows", "true")

    # Define lighting directly, avoiding external model downloads.
    light = add(world, "light", name="arena_sun", type="directional")
    add(light, "pose", "0 0 10 0 0 0")
    add(light, "cast_shadows", "true")
    add(light, "diffuse", "0.8 0.8 0.8 1")
    add(light, "specular", "0.2 0.2 0.2 1")
    add(light, "direction", "-0.3 -0.4 -1")

    ground = config["ground"]
    length = float(ground["length"])
    width = float(ground["width"])
    thickness = float(ground["thickness"])
    if min(length, width, thickness) <= 0:
        raise ValueError("Ground dimensions must be positive.")

    add_box(
        world,
        name="arena_ground",
        pose=(length / 2, width / 2, -thickness / 2, 0, 0, 0),
        size=(length, width, thickness),
        rgba=ground["rgba"],
    )

    # Full track paint meshes
    model = add(world, "model", name="track_paint")
    add(model, "static", "true")
    add(model, "pose", "0 0 0.002 0 0 0")
    link = add(model, "link", name="paint_link")

    for color in ("yellow", "white"):
        visual = add(link, "visual", name=f"{color}_paint")
        add(visual, "cast_shadows", "false")
        geometry = add(visual, "geometry")
        mesh = add(geometry, "mesh")
        add(
            mesh, "uri",
            f"model://track_paint/meshes/{color}.dae",
        )

    # Configurable practice cones: collision geometry is visible to ray LiDAR.
    for cone in config.get("cones", []):
        model = add(world, "model", name=cone["name"])
        add(model, "static", "true")
        x = cone["x"] + cone.get("dx", 0.0)
        y = cone["y"] + cone.get("dy", 0.0)
        add(model, "pose", f"{x} {y} 0 0 0 0")
        link = add(model, "link", name="cone_link")
        for kind in ("visual", "collision"):
            part = add(link, kind, name=f"cone_{kind}")
            mesh = add(add(part, "geometry"), "mesh")
            add(mesh, "uri", "model://traffic_cone/meshes/traffic_cone.stl")
            if kind == "visual":
                material = add(part, "material")
                add(material, "ambient", "1 0.25 0.02 1")
                add(material, "diffuse", "1 0.25 0.02 1")

    # Minimal stationary LIMO Pro envelopes; no robot nodes or plugins.
    vehicle_size = config.get("obstacle_vehicle", {})
    for vehicle in config.get("parked_vehicles", []):
        length = float(vehicle_size["length"])
        width = float(vehicle_size["width"])
        height = float(vehicle_size["height"])
        if min(length, width, height) <= 0:
            raise ValueError("Obstacle vehicle dimensions must be positive.")
        color = "0.12 0.40 0.80 1" if vehicle["zone"] == "three_lane" else "0.70 0.72 0.75 1"
        add_box(
            world,
            name=vehicle["name"],
            pose=(vehicle["x"], vehicle["y"], height / 2,
                  0, 0, vehicle.get("yaw", 0.0)),
            size=(length, width, height),
            rgba=color,
            collision=True,
        )

    # Existing supports plus directly attached textured panels.
    support = config["sign_support"]
    h = float(support["pole_height"])
    w = float(support["pole_width"])
    base = float(support["base_size"])
    base_h = float(support["base_height"])
    if min(h, w, base, base_h) <= 0:
        raise ValueError("Sign support dimensions must be positive.")
    for sign in config.get("signs", []):
        add_box(
            world, name=sign["name"] + "_base",
            pose=(sign["x"], sign["y"], base_h / 2, 0, 0, 0),
            size=(base, base, base_h), rgba="0.65 0.65 0.65 1",
        )
        add_box(
            world, name=sign["name"] + "_pole",
            pose=(sign["x"], sign["y"], base_h + h / 2, 0, 0, 0),
            size=(w, w, h), rgba="0.95 0.95 0.95 1",
        )

    branch = config.get("branch_sign", "A")
    if branch not in ("A", "B"):
        raise ValueError("Resolved branch_sign must be A or B")
    for sign in config.get("signs", []):
        kind = branch if sign["type"] == "branch" else sign["type"]
        if kind not in ("A", "B", "slow", "parking", "pedestrian", "bump"):
            raise ValueError(f"Unknown sign: {kind}")
        # Pole front is +w/2. The disk sits wholly in front of it.
        thickness = 0.01
        disk_center = w / 2 + thickness / 2
        panel = add(world, "model", name=sign["name"] + "_panel")
        add(panel, "static", "true")
        add(panel, "pose",
            f"{sign['x']} {sign['y']} {base_h + h} 0 0 {sign['yaw']}")
        link = add(panel, "link", name="panel_link")
        for tag in ("visual", "collision"):
            disk = add(link, tag, name="disk_" + tag)
            if kind in ("pedestrian", "bump"):
                add(disk, "pose", f"{disk_center} 0 0 0 0 0")
                add(add(add(disk, "geometry"), "mesh"), "uri",
                    f"model://sign_{kind}/meshes/triangle_body.stl")
            else:
                add(disk, "pose", f"{disk_center} 0 0 0 {math.pi / 2} 0")
                cylinder = add(add(disk, "geometry"), "cylinder")
                add(cylinder, "radius", "0.12")
                add(cylinder, "length", thickness)
            if tag == "visual":
                backing = add(disk, "material")
                add(backing, "ambient", "0.95 0.95 0.95 1")
                add(backing, "diffuse", "0.95 0.95 0.95 1")
        face = add(link, "visual", name="printed_face")
        add(face, "pose", f"{disk_center + thickness / 2 + 0.0005} 0 0 0 0 0")
        face_file = "triangle_face.dae" if kind in ("pedestrian", "bump") else "round_face.dae"
        add(add(add(face, "geometry"), "mesh"), "uri",
            f"model://sign_{kind}/meshes/{face_file}")
        material = add(face, "material")
        script = add(material, "script")
        add(script, "uri", f"model://sign_{kind}/materials/scripts")
        add(script, "uri", f"model://sign_{kind}/materials/textures")
        add(script, "name", f"Kudos/Sign_{kind}")
        add(face, "cast_shadows", "false")

    bump = config.get("speed_bump")
    if bump:
        length, width, height = (float(bump[k]) for k in ("length", "width", "height"))
        if not (0 < height < 0.024 and length > 0 and width > 0):
            raise ValueError("Bump requires positive dimensions and height below 24 mm")
        model = add(world, "model", name="speed_bump")
        add(model, "static", "true")
        add(model, "pose", f"{bump['x']} {bump['y']} 0 0 0 {bump['yaw']}")
        link = add(model, "link", name="bump_link")
        collision = add(link, "collision", name="ramp_collision")
        mesh = add(add(collision, "geometry"), "mesh")
        add(mesh, "uri", "model://speed_bump/meshes/bump.stl")
        add(mesh, "scale", f"{length} {width} {height}")
        for i in range(10):
            visual = add(link, "visual", name=f"stripe_{i}")
            add(visual, "pose", f"0 {-width/2 + (i+.5)*width/10} 0 0 0 0")
            mesh = add(add(visual, "geometry"), "mesh")
            add(mesh, "uri", "model://speed_bump/meshes/bump.stl")
            add(mesh, "scale", f"{length} {width/10} {height}")
            material = add(visual, "material")
            color = "1 0.8 0 1" if i % 2 == 0 else "0.03 0.03 0.03 1"
            add(material, "ambient", color)
            add(material, "diffuse", color)

    signal = config.get("traffic_light")
    if signal:
        x, y, yaw = (signal[k] for k in ("x", "y", "yaw"))
        z = signal["center_height"]
        depth = signal["body_depth"]
        add_box(world, "traffic_light_base", (x,y,.01,0,0,0), (.10,.10,.02), "0.3 0.3 0.3 1")
        add_box(world, "traffic_light_pole", (x,y,z/2,0,0,0), (.025,.025,z), "0.7 0.7 0.7 1")
        model = add_box(world, "traffic_light", (x,y,z,0,0,yaw),
                        (depth,signal["body_width"],signal["body_height"]), "0.02 0.02 0.02 1")
        link = model.find("link")
        link.set("name", "signal_link")
        for name, offset, rgb in (("RED",.065,(1,.01,.01)),
                                  ("YELLOW",0,(1,.65,.01)),
                                  ("GREEN",-.065,(.01,1,.01))):
            visual = add(link, "visual", name="lamp_" + name)
            add(visual, "pose", f"{depth/2+.002} 0 {offset} 0 {math.pi/2} 0")
            cylinder = add(add(visual, "geometry"), "cylinder")
            add(cylinder, "radius", signal["lamp_radius"])
            add(cylinder, "length", ".004")
            material = add(visual, "material")
            gain = 1 if name == "RED" else .06
            color = " ".join(str(v*gain) for v in rgb) + " 1"
            add(material, "ambient", color)
            add(material, "diffuse", color)
            add(material, "emissive", color if name == "RED" else "0 0 0 1")
        add(model, "plugin", name="traffic_light_driver", filename="libkudos_traffic_light.so")

    gui = add(world, "gui", fullscreen="0")
    camera = add(gui, "camera", name="overview")
    add(camera, "pose", "5 -5 10 0 0.9 1.57079632679")
    add(camera, "view_controller", "orbit")

    return sdf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--branch-sign", choices=("A", "B", "random"))
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    mode = args.branch_sign or config.get("branch_sign", "A")
    config["branch_sign"] = secrets.choice(("A", "B")) if mode == "random" else mode
    print("Branch sign:", config["branch_sign"], flush=True)
    root = build_world(config)
    ET.indent(root, space="  ")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(
        args.output, encoding="utf-8", xml_declaration=True
    )
    print(f"Generated: {args.output.resolve()}")


if __name__ == "__main__":
    main()
