"""
mask_sampling.py — pick a point to act on inside a 2D detection mask.

WHY THIS LIVES IN calibration/
------------------------------
Same reason `mask_to_pointcloud.py` does: three Python environments in this
project share no dependencies (GraspGen pins torch==2.1.0/py3.10,
Grounded-SAM-2 needs torch>=2.3.1, and the ROS 2 node runs on system
python3.10). Anything that must be callable from more than one of them has
to be PURE NUMPY and live outside all three package trees. `calibration/`
is already that shared shelf, and this module is imported by:

  - `calibration/test_fps.py`                 (CLI, any python with cv2)
  - `pragmabot_bridge/live_perception.py`     (ROS 2 env, no cv2/torch)

Putting it inside `pragmabot_bridge/` would make it unimportable from the
CLI without installing the ament package; putting it in `src/pragmabot/`
would drop a perception helper into the VLM planner package, most of which
is on the never-modify list.

WHAT IT DOES
------------
`farthest_point_sampling()` picks N spread-out pixels inside a binary mask.
That alone is not enough to choose a PLACEMENT point: FPS maximises spread,
so its candidates are pulled toward the mask's extremes and it will happily
return a corner of a plate. Releasing an object over the rim of a surface
is how a "successful" place ends with the object on the floor.

`select_placement_pixel()` fixes that with two layers, in this order:

  1. Restrict FPS to the mask's INTERIOR CORE — the set of pixels at least
     `min_interior_px` away from any boundary. This is computed by
     `interior_distance()`, which iterates `mask_to_pointcloud.erode_mask`
     (a 3x3 cross kernel, so repeated erosion is exactly the L1 /
     city-block distance to the nearest non-mask pixel). Reused, not
     reimplemented — the same erosion that removes stereo flying pixels
     from the object cloud defines "away from the edge" here.
  2. Among the FPS candidates, keep the one with the LARGEST interior
     distance. FPS supplies diversity across the surface; this second pass
     spends that diversity on safety rather than on spread for its own
     sake.

If the interior core is empty (a thin or sliver-shaped surface), the margin
is relaxed in halving steps and the margin actually used is reported back,
so the caller can put it in the reason string the planner reflects on
rather than silently placing on an edge.
"""

import numpy as np

from mask_to_pointcloud import erode_mask

# Default number of spread-out candidates to consider. Small enough that the
# per-candidate scoring is free, large enough that the interior maximum is
# not decided by one lucky draw.
DEFAULT_N_CANDIDATES = 16

# Default interior margin in pixels. At our ZED's 1280x720 and a ~0.9 m
# working distance, 1 px is roughly 1.7 mm on the table, so 8 px is ~1.4 cm
# of clearance from the visible rim of the surface — about a finger width,
# and comfortably larger than the mask's own boundary uncertainty.
DEFAULT_MIN_INTERIOR_PX = 8

# Cap on the erosion iterations used to build the distance map. Anything
# deeper than this is "very interior" and does not need to be distinguished.
MAX_INTERIOR_PX = 64


def farthest_point_sampling(mask: np.ndarray, n_points: int = 8) -> np.ndarray:
    """Select n_points spread-out pixels from a binary mask.
    Returns an (N, 2) array of [u, v] = [col, row] pixel coordinates."""
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        raise ValueError("Mask is empty — nothing to sample from.")

    n_points = min(n_points, len(xs))
    points = np.column_stack([xs, ys]).astype(np.int32)

    # start at the point closest to the mask's centroid
    centroid = points.mean(axis=0)
    first_idx = np.argmin(np.sum((points - centroid) ** 2, axis=1))

    selected = [int(first_idx)]
    min_dist_sq = np.sum((points - points[first_idx]) ** 2, axis=1).astype(np.float64)

    for _ in range(1, n_points):
        next_idx = int(np.argmax(min_dist_sq))
        selected.append(next_idx)
        new_dist_sq = np.sum((points - points[next_idx]) ** 2, axis=1)
        min_dist_sq = np.minimum(min_dist_sq, new_dist_sq)

    return points[selected]


def interior_distance(mask: np.ndarray, max_px: int = MAX_INTERIOR_PX) -> np.ndarray:
    """L1 distance in pixels from each mask pixel to the nearest edge.

    0 outside the mask; 1 for a pixel touching the boundary; k for a pixel
    that survives k-1 further erosions. Saturates at `max_px`.

    Built by iterating `mask_to_pointcloud.erode_mask`, whose 3x3 cross
    kernel makes repeated erosion equivalent to a city-block distance
    transform. Work is confined to the mask's bounding box — on a 1280x720
    frame a tabletop surface occupies a small fraction of the image, and
    eroding the full frame 64 times would cost ~100x more for an identical
    result.
    """
    mask = np.asarray(mask, dtype=bool)
    dist = np.zeros(mask.shape, dtype=np.int32)
    if not mask.any():
        return dist

    ys, xs = np.nonzero(mask)
    # +1 border so the crop's own edge is treated as background, exactly as
    # erode_mask's zero-padding would treat the image border.
    y0, y1 = max(int(ys.min()) - 1, 0), min(int(ys.max()) + 2, mask.shape[0])
    x0, x1 = max(int(xs.min()) - 1, 0), min(int(xs.max()) + 2, mask.shape[1])

    cur = mask[y0:y1, x0:x1]
    sub = cur.astype(np.int32)
    for _ in range(max_px - 1):
        cur = erode_mask(cur, 1)
        if not cur.any():
            break
        sub += cur

    dist[y0:y1, x0:x1] = sub
    return dist


def select_placement_pixel(mask: np.ndarray,
                           valid: np.ndarray = None,
                           n_candidates: int = DEFAULT_N_CANDIDATES,
                           min_interior_px: int = DEFAULT_MIN_INTERIOR_PX):
    """Choose one pixel to place onto, biased away from the surface's edges.

    Args:
        mask: (H, W) bool — the placement surface, from GroundedSAM.
        valid: optional (H, W) bool — pixels with usable depth. Candidates
            are drawn only from `mask & valid`, so a pixel whose depth the
            stereo matcher dropped is never chosen and then rejected later.
        n_candidates: how many FPS candidates to score.
        min_interior_px: required L1 distance from the mask boundary.

    Returns:
        (u, v, info). `info` carries the numbers the caller should put in
        the reason string: `interior_px` (the chosen pixel's distance from
        the edge), `margin_px` (the margin actually enforced, which is
        lower than `min_interior_px` if the core had to be relaxed),
        `relaxed` (bool), `n_candidates` and `n_pixels`.

    Raises:
        ValueError if `mask & valid` is empty — there is nothing to choose.
    """
    mask = np.asarray(mask, dtype=bool)
    usable = mask if valid is None else (mask & np.asarray(valid, dtype=bool))
    if not usable.any():
        raise ValueError(
            "no pixel is both inside the placement mask and has valid depth")

    dist = interior_distance(usable)

    # Relax the margin in halving steps rather than failing outright: a
    # narrow surface (a strip of table, a saucer seen edge-on) still has a
    # best available pixel, and reporting the reduced margin is more useful
    # to the planner than refusing to place at all.
    margin = int(min_interior_px)
    relaxed = False
    while margin > 1 and not (dist >= margin).any():
        margin //= 2
        relaxed = True
    core = dist >= margin
    if not core.any():
        core = usable
        margin = 1
        relaxed = True

    candidates = farthest_point_sampling(core, n_points=n_candidates)

    # FPS gave spread; spend it on safety. The interior distance is both the
    # "how far from the edge" and the "how much local mask support" measure
    # asked for — a pixel deep inside the mask is by construction surrounded
    # by mask on every side.
    scores = dist[candidates[:, 1], candidates[:, 0]]
    best = int(np.argmax(scores))
    u, v = int(candidates[best, 0]), int(candidates[best, 1])

    return u, v, {
        "interior_px": int(scores[best]),
        "margin_px": margin,
        "relaxed": relaxed,
        "n_candidates": int(len(candidates)),
        "n_pixels": int(usable.sum()),
        "candidates": candidates,
    }
