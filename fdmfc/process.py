from context import NonPlanarSurface, ProcessorContext
from extrusion import Extrusion
import os
import re
import open3d
import math
import numpy as np

feature_colors = {
    "Inner wall": [0.0, 0.0, 1.0],
    "Outer wall": [0.0, 1.0, 0.0],
    "Top surface": [1.0, 0.0, 0.0],
    "Ironing": [1.0, 1.0, 0.0],
    "Support": [1.0, 0.0, 1.0],
    "Skirt": [0.0, 1.0, 1.0],
    "Brim": [1.0, 0.5, 0.0],
    "Travel": [0.5, 0.5, 0.5],
}
default_color = [0.5, 0.5, 0.5]

printing_angle = 20
clearance_angle = 20


def parse_simple_args(gcode: str) -> dict:
    return dict(
        map(
            lambda x: (x[0], x[1:].strip()),
            filter(lambda x: x != "", gcode.split(";", maxsplit=1)[0].split(" ")),
        )
    )


def parse_klipper_args(gcode: str) -> dict:
    return dict(map(lambda x: list(map(str.strip, x.split("=", 1))), gcode.split(" ")))


def process_gcode(
    gcode: list[str],
    model_dir: str,
    plate_object: tuple[str, float, float] | None = None,
) -> list[str]:
    ctx = ProcessorContext(gcode, model_dir)
    if plate_object is not None:
        ctx.active_object = load_object(
            ctx, plate_object[0], plate_object[1], plate_object[2]
        )
    is_in_executable = False
    while ctx.gcode_line < len(ctx.gcode):
        if not is_in_executable and ctx.line.startswith(
            ctx.syntax.executable_block_start
        ):
            is_in_executable = True
        elif is_in_executable and ctx.line.startswith(ctx.syntax.executable_block_end):
            break
        elif is_in_executable:
            process_line(ctx)
        ctx.gcode_line += 1

    ctx.gcode[0] = f"; GCodeZAA\n" + ctx.gcode[0]

    line_set = open3d.t.geometry.LineSet()
    line_set.point.positions = open3d.core.Tensor(
        ctx.points, dtype=open3d.core.Dtype.Float32
    )
    line_set.line.indices = open3d.core.Tensor(ctx.lines, dtype=open3d.core.Dtype.Int32)
    line_set.line.colors = open3d.core.Tensor(
        ctx.colors, dtype=open3d.core.Dtype.Float32
    )

    # open3d.visualization.draw_geometries([line_set.to_legacy()])

    return ctx.gcode


def make_path(line_set: open3d.t.geometry.LineSet) -> list[list[int]]:
    lines = line_set.line.indices.numpy()
    paths = []
    while len(lines) > 0:
        path = [lines[0][0].item(), lines[0][1].item()]
        lines = np.delete(lines, 0, 0)
        while len(lines) > 0:
            next = np.where(
                np.logical_or(
                    lines == path[-1],
                    lines == path[0],
                )
            )[0]
            if len(next) != 0:
                if lines[next[0]][0] == path[-1]:
                    path.append(lines[next[0]][1].item())
                elif lines[next[0]][1] == path[-1]:
                    path.append(lines[next[0]][0].item())
                elif lines[next[0]][0] == path[0]:
                    path.insert(0, lines[next[0]][1].item())
                elif lines[next[0]][1] == path[0]:
                    path.insert(0, lines[next[0]][0].item())
                lines = np.delete(lines, next[0], 0)
            else:
                break
        paths.append(path)
    sorted_paths = [paths.pop()]
    vertices = line_set.point.positions.numpy()
    while len(paths) > 0:
        closest_index = 0
        closest_distance = math.inf
        closest_at = 0
        closest_rev = False
        for i, segment in enumerate(paths):
            d = np.linalg.norm(vertices[sorted_paths[0][0]] - vertices[segment[0]])
            if d < closest_distance:
                closest_index = i
                closest_distance = d
                closest_at = 0
                closest_rev = True
            d = np.linalg.norm(vertices[sorted_paths[0][0]] - vertices[segment[-1]])
            if d < closest_distance:
                closest_index = i
                closest_distance = d
                closest_at = 0
                closest_rev = False
            d = np.linalg.norm(vertices[sorted_paths[-1][-1]] - vertices[segment[0]])
            if d < closest_distance:
                closest_index = i
                closest_distance = d
                closest_at = -1
                closest_rev = False
            d = np.linalg.norm(vertices[sorted_paths[-1][-1]] - vertices[segment[-1]])
            if d < closest_distance:
                closest_index = i
                closest_distance = d
                closest_at = -1
                closest_rev = True
        if closest_rev:
            paths[closest_index].reverse()
        if closest_at == 0:
            sorted_paths.insert(0, paths.pop(closest_index))
        else:
            sorted_paths.append(paths.pop(closest_index))

    return sorted_paths


def process_non_planar(ctx: ProcessorContext, line_set: open3d.t.geometry.LineSet):
    paths = make_path(line_set)
    for path in paths:
        for i, point in enumerate(path):
            extrusion = Extrusion(
                p=ctx.last_p,
                x=line_set.point.positions[point][0].item(),
                y=line_set.point.positions[point][1].item(),
                z=line_set.point.positions[point][2].item(),
                e=None,
                f=3000 if i == 0 else None,
                relative=False,
            )
            if i != 0:
                # TODO: replace this shit calculation
                extrusion.e = extrusion.length() * 0.04
            extrusion.meta = "non-planar"
            ctx.last_p = extrusion.pos()
            ctx.extrusion.append(extrusion)


def load_object(
    ctx: ProcessorContext, name: str, x: float, y: float
) -> open3d.t.geometry.RaycastingScene:
    model_path = os.path.join(ctx.model_dir, name)
    mesh = open3d.t.io.read_triangle_mesh(model_path, enable_post_processing=True)
    min_bound = mesh.get_min_bound()
    max_bound = mesh.get_max_bound()
    center = min_bound + (max_bound - min_bound) / 2
    mesh.translate([x - center[0].item(), y - center[1].item(), -min_bound[2].item()])
    mesh.compute_triangle_normals()

    n_z = mesh.triangle["normals"][:, 2]
    mask = (n_z > math.sin(math.radians(90 - printing_angle))).logical_and(n_z > 0)
    masked_mesh = mesh.select_faces_by_mask(mask).to_legacy()
    masked_mesh = masked_mesh.remove_duplicated_triangles()
    masked_mesh = masked_mesh.remove_duplicated_vertices()
    masked_mesh = masked_mesh.merge_close_vertices(0.0001)
    cluster_index, tri_count, area = masked_mesh.cluster_connected_triangles()
    surfaces = []
    masked_mesh = open3d.t.geometry.TriangleMesh.from_legacy(masked_mesh)
    for i in range(len(area)):
        if area[i] < 0.1:
            continue
        surfaces.append(
            masked_mesh.select_faces_by_mask(np.asarray(cluster_index) == i)
        )
    for surface in surfaces:
        min_bound = surface.get_min_bound()
        max_bound = surface.get_max_bound()
        min_dir = min_bound[1].item()
        max_dir = max_bound[1].item()
        line_count = math.ceil((max_dir - min_dir) / 0.4)
        line_set = surface.slice_plane(
            open3d.core.Tensor([0, 0, 0]),
            open3d.core.Tensor([0, 1, 0]),
            [min_dir + i * 0.4 for i in range(line_count)],
        )
        line_set.line.colors = open3d.core.Tensor(
            np.full((line_set.line.indices.shape[0], 3), feature_colors["Top surface"]),
            dtype=open3d.core.Dtype.Float32,
        )
        z = line_set.point.positions[:, 2][line_set.line.indices.flatten()]
        top_surface_max_z = z.max().item() if len(line_set.line.indices) != 0 else 0
        top_surface_min_z = (
            z.min().item() if len(line_set.line.indices) != 0 else math.inf
        )
        top_surface = line_set if len(line_set.line.indices) != 0 else None

        legacy = surface.to_legacy()
        edges = np.asarray(legacy.get_non_manifold_edges(allow_boundary_edges=False))

        if len(edges) != 0:
            boundary = open3d.t.geometry.LineSet()
            boundary.point.positions = surface.vertex["positions"]
            boundary.line.indices = open3d.core.Tensor(
                edges, dtype=open3d.core.Dtype.Int32
            )
            boundary.line.colors = open3d.core.Tensor(
                np.full((len(edges), 3), feature_colors["Outer wall"]),
                dtype=open3d.core.Dtype.Float32,
            )
            z = boundary.point.positions[:, 2][boundary.line.indices.flatten()]
            boundary_max_z = z.max().item()
            boundary_min_z = z.min().item()
        else:
            boundary = None
            boundary_max_z = 0
            boundary_min_z = math.inf
        if top_surface is not None or boundary is not None:
            target = NonPlanarSurface(
                min_z=min(boundary_min_z, top_surface_min_z),
                max_z=max(boundary_max_z, top_surface_max_z),
                boundary=boundary,
                surface=top_surface,
                geometry=surface.vertex["positions"].numpy(),
                withheld=[],
            )
            ctx.non_planar_surfaces.append(target)

    # open3d.visualization.draw_geometries([surface.to_legacy() for surface in surfaces])
    scene = open3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh)

    return scene


def process_line(ctx: ProcessorContext):
    write_back = ""
    ctx.extrusion = []
    ctx.gcode[ctx.gcode_line] = ctx.line.strip() + "\n"

    if ctx.line.startswith("G0 ") or ctx.line.startswith("G1 "):
        args = parse_simple_args(ctx.line)
        ctx.extrusion.append(
            Extrusion(
                p=ctx.last_p,
                x=float(args["X"]) if "X" in args else None,
                y=float(args["Y"]) if "Y" in args else None,
                z=float(args["Z"]) if "Z" in args else None,
                e=float(args["E"]) if "E" in args else None,
                f=float(args["F"]) if "F" in args else None,
                relative=ctx.relative_positioning,
            )
        )
        ctx.points.append(list(ctx.extrusion[-1].pos()))
        if len(ctx.points) > 1:
            ctx.lines.append([len(ctx.points) - 2, len(ctx.points) - 1])
            ctx.colors.append(
                feature_colors.get(ctx.line_type, default_color)
                if ctx.extrusion[-1].e is not None
                else feature_colors["Travel"]
            )
    elif ctx.line.startswith("G2 "):
        # TODO: cw arc move
        pass
    elif ctx.line.startswith("G3 "):
        # TODO: ccw arc move
        pass
    elif ctx.line.startswith(ctx.syntax.line_type):
        line_type = ctx.line.removeprefix(ctx.syntax.line_type).strip()
        if (
            ctx.line_type == ctx.syntax.line_type_outer_wall
            and line_type != ctx.syntax.line_type_outer_wall
        ):
            ctx.gcode[ctx.gcode_line] = (
                "T1\n" + "".join(ctx.outer_wall_gcode) + "T0\n" + ctx.line
            )
            ctx.outer_wall_gcode = []
        ctx.line_type = line_type
    elif ctx.line.startswith(ctx.syntax.layer_change):
        ctx.layer += 1
        ctx.line_type = ctx.syntax.line_type_inner_wall  # doesn't get emitted properly
    elif ctx.line.startswith(ctx.syntax.z):
        ctx.z = float(ctx.line.removeprefix(ctx.syntax.z))
    elif ctx.line.startswith(ctx.syntax.height):
        ctx.height = float(ctx.line.removeprefix(ctx.syntax.height))
    elif ctx.line.startswith(ctx.syntax.width):
        ctx.width = float(ctx.line.removeprefix(ctx.syntax.width))
    elif ctx.line.startswith(ctx.syntax.wipe_start):
        ctx.wipe = True
    elif ctx.line.startswith(ctx.syntax.wipe_end):
        ctx.wipe = False
    elif ctx.line.startswith("M82"):
        ctx.relative_extrusion = False
    elif ctx.line.startswith("M83"):
        ctx.relative_extrusion = True
    elif ctx.line.startswith("G90"):
        ctx.relative_positioning = False
    elif ctx.line.startswith("G91"):
        ctx.relative_positioning = True
    elif ctx.line.startswith("G92"):
        args = parse_simple_args(ctx.line)
        if "E" in args:
            ctx.last_e = float(args["E"])
        if "X" in args or "Y" in args or "Z" in args:
            ctx.last_p = (
                float(args.get("X", ctx.last_p[0])),
                float(args.get("Y", ctx.last_p[1])),
                float(args.get("Z", ctx.last_p[2])),
            )
        pass
    # elif ctx.line.startswith("M73"):
    #    args = parse_simple_args(ctx.line)
    #    if "P" in args:
    #        ctx.progress_percent = float(args["P"])
    #    if "R" in args:
    #        ctx.progress_remaining_minutes = float(args["R"])
    elif ctx.line.startswith("EXCLUDE_OBJECT_DEFINE"):
        args = parse_klipper_args(ctx.line.removeprefix("EXCLUDE_OBJECT_DEFINE "))
        name = args["NAME"]
        x, y = map(float, args["CENTER"].split(","))

        # ctx.exclude_object[name] = load_object(
        #    ctx, re.sub(r"\.stl_.*$", ".stl", name), x, y
        # )
    elif ctx.line.startswith("EXCLUDE_OBJECT_START"):
        # args = parse_klipper_args(ctx.line.removeprefix("EXCLUDE_OBJECT_START "))
        # ctx.active_object = ctx.exclude_object[args["NAME"]]
        pass
    elif ctx.line.startswith("EXCLUDE_OBJECT_END"):
        # ctx.active_object = None
        pass

    if (
        ctx.line_type == ctx.syntax.line_type_outer_wall
        and len(ctx.extrusion) == 1
        and ctx.extrusion[0].e is not None
        and ctx.extrusion[0].z is None
    ):
        p = min(1, ctx.z / 30)
        e = ctx.extrusion[0].e
        ctx.extrusion[0].z = ctx.z - ctx.height * p
        ctx.extrusion[0].e = e * p
        ctx.gcode[ctx.gcode_line] = f"G1 {ctx.extrusion[0]}\n"
        ctx.extrusion[0].z = ctx.z
        ctx.extrusion[0].e = e * (1 - p)
        ctx.outer_wall_gcode.append(f"G1 {ctx.extrusion[0]}\n")
    elif ctx.line_type == ctx.syntax.line_type_outer_wall:
        ctx.outer_wall_gcode.append(ctx.gcode[ctx.gcode_line])
