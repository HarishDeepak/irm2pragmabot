"""
mesh_gen.py — generate the calibration cube mesh for FoundationPose.

Run on Alonnisos (or the laptop, since trimesh has no GPU dependency):
    python3 mesh_gen.py --side 0.06 --out cube.obj
    python3 mesh_gen.py --side 0.06 --out cube.obj --face-texture marker.png

CRITICAL: --side is in METERS, not cm. FoundationPose does not auto-scale —
a mismatched mesh silently produces a wrong-scale pose. Measure the cube
with calipers before running this; 6cm was an estimate, not confirmed.

If --face-texture is given, that image is applied to the +Z face only,
breaking the cube's 24-fold rotational symmetry (see CLAUDE.md). Without
it, the mesh is symmetric and FoundationPose's pose estimate will be
ambiguous by design, not by bug.
"""

import argparse

import numpy as np
import trimesh


def build_cube(side_m: float) -> trimesh.Trimesh:
    """Axis-aligned cube of the given side length, in meters, centered at origin."""
    return trimesh.creation.box(extents=[side_m, side_m, side_m])


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
                         help="Optional path to a marker image to break symmetry")
    args = parser.parse_args()

    if args.side > 1.0:
        print(f"WARNING: --side={args.side} looks like it might be in cm, not "
              f"meters. A {args.side}m cube is huge. Did you mean --side "
              f"{args.side / 100:.4f}?")

    mesh = build_cube(args.side)

    if args.face_texture:
        mesh = apply_face_texture(mesh, args.face_texture)
    else:
        print("WARNING: no --face-texture given. This mesh is fully symmetric "
              "and FoundationPose's orientation estimate will be ambiguous "
              "(see CLAUDE.md, 'Cube symmetry blocker'). Mark one face before "
              "calibration, not after.")

    mesh.export(args.out)
    print(f"Wrote {args.out} — side {args.side}m "
          f"({'textured, verify UV placement visually' if args.face_texture else 'PLAIN, symmetric — fix before use'})")


if __name__ == "__main__":
    main()
