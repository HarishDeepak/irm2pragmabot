"""
mesh_gen.py — generate the calibration cube mesh for FoundationPose.

Run on Alonnisos (or the laptop, since trimesh has no GPU dependency):
    python3 mesh_gen.py --side 0.061 --out cube.obj --edge-stripe

CRITICAL: --side is in METERS, not cm. FoundationPose does not auto-scale —
a mismatched mesh silently produces a wrong-scale pose. Measure the cube
with calipers before running this.

--edge-stripe builds the cube as 6 independent faces (real per-face solid
colors, not an image texture) with a stripe on one face's edge — this is
what actually breaks the 24-fold rotational symmetry (see CLAUDE.md).
Without it, the mesh is symmetric and FoundationPose's pose estimate will
be ambiguous by design, not by bug. --face-texture is the old approach —
kept only for reference, confirmed broken (paints all 6 faces identically
via trimesh's default box UVs, see apply_face_texture()'s docstring).
"""

import argparse

import numpy as np
import trimesh

# normal, u, v per face such that u x v == normal (outward, right-hand rule) —
# needed because each face below is built independently (not from a shared-
# vertex primitive box), which is what actually lets one face carry different
# per-triangle colors than the other 5. See build_cube_with_edge_stripe.
_FACES = {
    "+z": (np.array([0, 0, 1.0]), np.array([1, 0, 0.0]), np.array([0, 1, 0.0])),
    "-z": (np.array([0, 0, -1.0]), np.array([0, 1, 0.0]), np.array([1, 0, 0.0])),
    "+x": (np.array([1, 0, 0.0]), np.array([0, 1, 0.0]), np.array([0, 0, 1.0])),
    "-x": (np.array([-1, 0, 0.0]), np.array([0, 0, 1.0]), np.array([0, 1, 0.0])),
    "+y": (np.array([0, 1, 0.0]), np.array([0, 0, 1.0]), np.array([1, 0, 0.0])),
    "-y": (np.array([0, -1, 0.0]), np.array([1, 0, 0.0]), np.array([0, 0, 1.0])),
}


def build_cube(side_m: float) -> trimesh.Trimesh:
    """Axis-aligned cube of the given side length, in meters, centered at origin."""
    return trimesh.creation.box(extents=[side_m, side_m, side_m])


def _quad(center, u, v, half_u, half_v):
    """4 CCW-from-outside vertices of a rectangle centered at `center`,
    spanning +/-half_u along u and +/-half_v along v."""
    return np.array([
        center - u * half_u - v * half_v,
        center + u * half_u - v * half_v,
        center + u * half_u + v * half_v,
        center - u * half_u + v * half_v,
    ])


def build_cube_with_edge_stripe(side_m: float, stripe_face: str = "+z",
                                 stripe_frac: float = 0.3,
                                 cube_color=(120, 200, 140, 255),
                                 stripe_color=(225, 222, 210, 255)) -> trimesh.Trimesh:
    """
    Cube built as 6 INDEPENDENT faces (no shared vertices across faces) so
    each face can carry its own solid color — this is what actually breaks
    the 24-fold symmetry, unlike apply_face_texture()'s image-UV approach
    below, which (per its own docstring) paints every face identically on
    trimesh's default box UVs.

    `stripe_face` gets a solid-color stripe along its +v edge, occupying
    the outer `stripe_frac` of that face's depth (matches a real strip of
    tape stuck along one edge, e.g. off-white tape on a green 3D-printed
    cube) — everything else on that face, and all 5 other faces, stay
    `cube_color`. An edge stripe (not a centered mark) is what makes all
    4 in-plane rotations of that face visually distinct from each other,
    on top of already telling this face apart from the other 5 plain ones.
    """
    half = side_m / 2.0
    verts_list = []
    faces_list = []
    colors_list = []
    offset = 0

    for key, (normal, u, v) in _FACES.items():
        center = normal * half
        if key == stripe_face:
            # main region: from -half to (half - stripe_width) along v
            stripe_width = stripe_frac * side_m
            main_half_v = (side_m - stripe_width) / 2.0
            main_center = center - v * (stripe_width / 2.0)
            stripe_center = center + v * (half - stripe_width / 2.0)

            quad = _quad(main_center, u, v, half, main_half_v)
            verts_list.append(quad)
            faces_list.append(np.array([[0, 1, 2], [0, 2, 3]]) + offset)
            colors_list += [cube_color] * 2
            offset += 4

            quad = _quad(stripe_center, u, v, half, stripe_width / 2.0)
            verts_list.append(quad)
            faces_list.append(np.array([[0, 1, 2], [0, 2, 3]]) + offset)
            colors_list += [stripe_color] * 2
            offset += 4
        else:
            quad = _quad(center, u, v, half, half)
            verts_list.append(quad)
            faces_list.append(np.array([[0, 1, 2], [0, 2, 3]]) + offset)
            colors_list += [cube_color] * 2
            offset += 4

    vertices = np.concatenate(verts_list, axis=0)
    faces = np.concatenate(faces_list, axis=0)
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.visual = trimesh.visual.ColorVisuals(
        mesh, face_colors=np.array(colors_list, dtype=np.uint8)
    )
    return mesh


def apply_face_texture(mesh: trimesh.Trimesh, texture_path: str) -> trimesh.Trimesh:
    """
    Apply an image texture to the mesh's +Z face only, so exactly one face
    is visually distinct — this is what breaks the symmetry FoundationPose
    would otherwise be unable to resolve.

    NOTE: trimesh's per-face UV control on a primitive box is limited; for
    a marker sticker/ArUco tag, the simplest robust approach in practice is
    often to model the cube as 6 separate quads with independent UVs rather
    than rely on trimesh.creation.box's default UV layout. This function
    gives a starting point — inspect the exported .obj in Blender/MeshLab
    before trusting it, and adjust the UV mapping by hand if the marker
    doesn't land cleanly on one face.
    """
    from PIL import Image

    texture_image = Image.open(texture_path)
    material = trimesh.visual.texture.SimpleMaterial(image=texture_image)

    # Default box UVs from trimesh unwrap each face into the same [0,1]^2
    # square repeated per face — good enough to get a marker visible on
    # every face for a first pass, but means the marker appears on ALL
    # faces, not just one. For true single-face marking, edit the exported
    # .obj's UV coordinates directly, or construct the box from 6
    # independently-UV'd quads. Flagged here rather than silently assumed
    # correct — verify visually before running calibration.
    uv = mesh.visual.uv if mesh.visual.uv is not None else np.zeros((len(mesh.vertices), 2))
    mesh.visual = trimesh.visual.TextureVisuals(uv=uv, material=material)
    return mesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", type=float, required=True,
                         help="Cube side length in METERS (e.g. 0.06 for 6cm)")
    parser.add_argument("--out", type=str, default="cube.obj",
                         help="Output mesh path (.obj)")
    parser.add_argument("--face-texture", type=str, default=None,
                         help="DEPRECATED — paints all 6 faces identically, "
                              "does not break symmetry. Use --edge-stripe.")
    parser.add_argument("--edge-stripe", action="store_true",
                         help="Build with a solid-color edge stripe on one "
                              "face (6 independent faces, real per-face "
                              "colors) — use this, not --face-texture.")
    parser.add_argument("--stripe-face", default="+z",
                         choices=list(_FACES.keys()))
    parser.add_argument("--stripe-frac", type=float, default=0.3,
                         help="Fraction of that face's depth the stripe covers")
    args = parser.parse_args()

    if args.side > 1.0:
        print(f"WARNING: --side={args.side} looks like it might be in cm, not "
              f"meters. A {args.side}m cube is huge. Did you mean --side "
              f"{args.side / 100:.4f}?")

    if args.edge_stripe:
        mesh = build_cube_with_edge_stripe(
            args.side, stripe_face=args.stripe_face, stripe_frac=args.stripe_frac
        )
        status = f"edge-striped on {args.stripe_face}, {args.stripe_frac:.0%} of face depth"
    elif args.face_texture:
        mesh = build_cube(args.side)
        mesh = apply_face_texture(mesh, args.face_texture)
        status = "textured, verify UV placement visually (KNOWN BROKEN — see warning above)"
    else:
        mesh = build_cube(args.side)
        status = "PLAIN, symmetric — fix before use"
        print("WARNING: no --edge-stripe given. This mesh is fully symmetric "
              "and FoundationPose's orientation estimate will be ambiguous "
              "(see CLAUDE.md, 'Cube symmetry blocker'). Mark one face before "
              "calibration, not after.")

    mesh.export(args.out)
    print(f"Wrote {args.out} — side {args.side}m ({status})")


if __name__ == "__main__":
    main()
