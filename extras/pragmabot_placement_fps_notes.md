# Chat Transcript: Placement &amp; FPS &amp; Fig 4

## User

How to implement the placement aspect mentioned here? I did not understand what they do with the FPS for placement, and how it relates to Fig 4

## Aurelle

For **placement**, PRAGMABOT uses FPS to turn a segmented destination region into a small set of candidate placement points, then asks the VLM to choose the most suitable one.

The process is:

1. **Choose the object and skill.**  
   The VLM decides that the next skill is `place`, and identifies the destination object or region—for example, “place the apple on the plate.”

2. **Segment the destination.**  
   A grounding-and-segmentation module produces a binary mask for the plate. The mask contains all image pixels that belong to the possible placement surface.

3. **Generate candidate points with FPS.**  
   Instead of treating every pixel in the plate mask as a possible placement location, they use **farthest-point sampling (FPS)** to select a small, spatially distributed set of points from the mask. FPS works approximately as follows:

   ```text
   candidates = {an initial point}
   repeat:
       add the mask point whose distance to
       the nearest selected point is largest
   ```

   Thus, the selected points cover the plate rather than clustering together. For example, a plate mask might be reduced to candidate points near its center, left side, right side, and so on.

4. **Render the candidates on the image.**  
   Each candidate is drawn as a numbered region or marker—e.g. candidate 1, candidate 2, candidate 3—over the original camera image.

5. **Ask the VLM to select one.**  
   The VLM examines the annotated image and chooses the candidate that best satisfies the task. For “place the apple on the plate,” it might choose a central point with enough clearance from the rim or from other objects.

6. **Convert that choice into a robot action.**  
   The selected image location is used as the target location for the low-level placement controller. The paper does not spell out the exact camera-to-robot geometric transformation, but that conversion must ultimately produce a 3D target pose for the arm.

The important distinction is that FPS is **not itself deciding where the robot should place the object**. It only creates a compact, diverse set of possible locations. The VLM performs the semantic selection.

### How this relates to Figure 4

Figure 4 illustrates the general annotation mechanism for different skills:

- For **grasping**, the system segments the object and presents candidate grasp locations—for example, selecting the stick rather than the meat on a drumstick.
- For **pushing**, it creates candidate goal locations or goal masks, and the VLM selects where the pushed object should end up.
- For **placing**, it applies FPS inside the segmented destination mask to create candidate placement locations, then the VLM selects among them.

So the visual logic is:

```text
RGB image
   ↓
segment destination object or surface
   ↓
FPS selects diverse candidate locations
   ↓
annotated image with numbered candidates
   ↓
VLM chooses a candidate
   ↓
place controller executes at that location
```

The paper states that FPS is used for placing to generate candidate locations from the segmented mask, while for pushing it generates candidate goal masks; in both cases, the VLM evaluates the annotations and selects the appropriate location. [Image Annotation](https://www.alphaxiv.org/abs/2507.16713?page=5)

A simple implementation would therefore be:

```python
def placement_target(rgb_image, destination_text):
    mask = segment(rgb_image, destination_text)

    # Optional: erode the mask to avoid the boundary
    mask = morphological_erosion(mask)

    points = farthest_point_sample(mask_pixels(mask), n=8)

    annotated = draw_numbered_points(rgb_image, points)

    chosen_index = vlm_choose(
        annotated,
        prompt="Choose the safest location to place the object."
    )

    pixel = points[chosen_index]
    target_3d = backproject_to_3d(pixel, depth_image)
    execute_place(target_3d)
```

The erosion step is not described in the paper, but it is a sensible practical addition: raw segmentation masks often include the object boundary, whereas a placement controller usually needs clearance from the edge. The paper’s actual contribution is the combination of **segmentation → FPS candidate generation → VLM selection**, which lets the VLM reason about semantic placement without requiring it to output a precise continuous coordinate directly.

## User

now write the code for each of these functions given the RGB and depth image.

## Aurelle

A working placement pipeline needs **four inputs beyond the RGB image**: a destination segmentation method, camera intrinsics, the RGB-to-robot transform, and a robot controller. The code below implements the rest of the pipeline and leaves the VLM and robot-specific pieces as explicit interfaces.

```python
# placement_pipeline.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence

import cv2
import numpy as np


# ---------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------

@dataclass
class CameraIntrinsics:
    """
    Pinhole camera intrinsics.

    fx, fy: focal lengths in pixels
    cx, cy: principal point in pixels
    depth_scale: converts the depth image's stored value to metres.
                 For example:
                   uint16 depth in millimetres -> depth_scale=0.001
                   float32 depth already in metres -> depth_scale=1.0
    """
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float = 1.0


@dataclass
class PlacementTarget:
    pixel: tuple[int, int]       # (u, v), image coordinates
    camera_xyz: np.ndarray       # [x, y, z] in camera coordinates, metres
    robot_xyz: np.ndarray        # [x, y, z] in robot coordinates, metres
    mask: np.ndarray              # final destination mask
    annotated_image: np.ndarray
    candidate_pixels: np.ndarray  # shape: [N, 2], each row is [u, v]


# ---------------------------------------------------------------------
# 1. Destination segmentation
# ---------------------------------------------------------------------

def segment_destination(
    rgb: np.ndarray,
    text: str,
    segmenter: Callable[[np.ndarray, str], np.ndarray],
) -> np.ndarray:
    """
    Run an external open-vocabulary segmentation model.

    Parameters
    ----------
    rgb:
        H x W x 3 uint8 RGB image.
    text:
        Destination description, e.g. "plate".
    segmenter:
        Function wrapping Grounded SAM, SAM, or another segmenter.
        It must return either:
          - a boolean mask of shape H x W, or
          - a uint8 mask with values 0 and 255.

    Returns
    -------
    Boolean mask of shape H x W.
    """
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("rgb must have shape H x W x 3")

    mask = segmenter(rgb, text)

    if mask.shape != rgb.shape[:2]:
        raise ValueError(
            f"Segmenter returned {mask.shape}, expected {rgb.shape[:2]}"
        )

    return mask.astype(bool)


def simple_color_segmentation(
    rgb: np.ndarray,
    lower_rgb: Sequence[int],
    upper_rgb: Sequence[int],
) -> np.ndarray:
    """
    A simple debugging segmenter.

    This is not a replacement for Grounded SAM. It is useful for testing
    the remainder of the pipeline when the destination has a known colour.
    """
    lower = np.asarray(lower_rgb, dtype=np.uint8)
    upper = np.asarray(upper_rgb, dtype=np.uint8)

    mask = cv2.inRange(rgb, lower, upper)
    return mask > 0


# ---------------------------------------------------------------------
# 2. Mask cleanup
# ---------------------------------------------------------------------

def clean_mask(
    mask: np.ndarray,
    erosion_radius: int = 5,
    min_component_area: int = 500,
) -> np.ndarray:
    """
    Remove small components and optionally erode the boundary.

    Erosion is useful for placement because the segmentation boundary may
    include the rim or edge of a container. It creates a safer interior
    region, but it also makes the available region smaller.
    """
    mask_u8 = (mask.astype(np.uint8) * 255)

    # Remove small connected components.
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask_u8, connectivity=8
    )

    cleaned = np.zeros_like(mask_u8)

    for label in range(1, num_labels):
        area = stats[label, cv2.CC_STAT_AREA]
        if area >= min_component_area:
            cleaned[labels == label] = 255

    if not np.any(cleaned):
        raise ValueError("No sufficiently large destination region found")

    # Shrink the mask away from boundaries.
    if erosion_radius > 0:
        kernel_size = 2 * erosion_radius + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        cleaned = cv2.erode(cleaned, kernel)

    if not np.any(cleaned):
        raise ValueError(
            "Mask disappeared after erosion; reduce erosion_radius"
        )

    return cleaned > 0


# ---------------------------------------------------------------------
# 3. Farthest-point sampling
# ---------------------------------------------------------------------

def farthest_point_sampling(
    mask: np.ndarray,
    n_points: int = 8,
    initial_point: Optional[tuple[int, int]] = None,
    random_seed: int = 0,
) -> np.ndarray:
    """
    Select spatially distributed pixels from a binary mask.

    Returns
    -------
    points:
        Integer array with shape [N, 2], where each row is [u, v].

    Notes
    -----
    This is FPS over mask pixels. For very large masks, stride can be used
    to reduce the number of pixels before sampling.
    """
    ys, xs = np.nonzero(mask)

    if len(xs) == 0:
        raise ValueError("Cannot sample from an empty mask")

    n_points = min(n_points, len(xs))

    points = np.column_stack([xs, ys]).astype(np.int32)

    rng = np.random.default_rng(random_seed)

    # Start at the mask point closest to the mask centroid unless specified.
    if initial_point is None:
        centroid = points.mean(axis=0)
        first_idx = np.argmin(np.sum((points - centroid) ** 2, axis=1))
    else:
        initial = np.asarray(initial_point, dtype=np.float32)
        first_idx = np.argmin(np.sum((points - initial) ** 2, axis=1))

    selected_indices = [int(first_idx)]

    # Distance from each point to its nearest selected point.
    min_dist_sq = np.sum(
        (points - points[first_idx]) ** 2,
        axis=1,
    ).astype(np.float64)

    for _ in range(1, n_points):
        next_idx = int(np.argmax(min_dist_sq))
        selected_indices.append(next_idx)

        new_dist_sq = np.sum(
            (points - points[next_idx]) ** 2,
            axis=1,
        )

        min_dist_sq = np.minimum(min_dist_sq, new_dist_sq)

    return points[selected_indices]


# ---------------------------------------------------------------------
# 4. Annotated image shown to the VLM
# ---------------------------------------------------------------------

def draw_numbered_candidates(
    rgb: np.ndarray,
    mask: np.ndarray,
    candidate_pixels: np.ndarray,
    radius: int = 8,
) -> np.ndarray:
    """
    Draw the destination mask and numbered candidate points.

    Coordinates are expected as [u, v] = [x, y].
    """
    annotated = rgb.copy()

    # Draw the mask boundary.
    contours, _ = cv2.findContours(
        (mask.astype(np.uint8) * 255),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    cv2.drawContours(
        annotated,
        contours,
        contourIdx=-1,
        color=(0, 255, 0),
        thickness=2,
    )

    for index, (u, v) in enumerate(candidate_pixels):
        u, v = int(u), int(v)

        cv2.circle(
            annotated,
            center=(u, v),
            radius=radius,
            color=(0, 0, 255),
            thickness=-1,
        )

        # White number with black outline for readability.
        label = str(index + 1)

        cv2.putText(
            annotated,
            label,
            org=(u + radius + 3, v - radius - 3),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.7,
            color=(0, 0, 0),
            thickness=4,
            lineType=cv2.LINE_AA,
        )

        cv2.putText(
            annotated,
            label,
            org=(u + radius + 3, v - radius - 3),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.7,
            color=(255, 255, 255),
            thickness=2,
            lineType=cv2.LINE_AA,
        )

    return annotated


# ---------------------------------------------------------------------
# 5. VLM candidate selection
# ---------------------------------------------------------------------

def choose_candidate_with_vlm(
    annotated_rgb: np.ndarray,
    instruction: str,
    candidate_pixels: np.ndarray,
    vlm_selector: Callable[[np.ndarray, str, int], int],
) -> int:
    """
    Call an external VLM selector.

    The VLM wrapper must return a zero-based candidate index.

    Example instruction:
        "Place the object safely on the plate. Choose a location
         away from the plate rim and other objects."

    The wrapper should be constrained to return one integer from 0 to N-1.
    """
    selected_index = vlm_selector(
        annotated_rgb,
        instruction,
        len(candidate_pixels),
    )

    if not isinstance(selected_index, (int, np.integer)):
        raise TypeError("vlm_selector must return an integer index")

    selected_index = int(selected_index)

    if not 0 <= selected_index < len(candidate_pixels):
        raise ValueError(
            f"VLM selected {selected_index}, but valid indices are "
            f"0 through {len(candidate_pixels) - 1}"
        )

    return selected_index


def choose_center_candidate(
    candidate_pixels: np.ndarray,
    mask: np.ndarray,
) -> int:
    """
    Deterministic fallback when no VLM is available.

    Selects the candidate closest to the mask centroid. This does not
    understand obstacles or semantics.
    """
    ys, xs = np.nonzero(mask)
    centroid = np.array([xs.mean(), ys.mean()])

    distances = np.sum(
        (candidate_pixels.astype(np.float32) - centroid) ** 2,
        axis=1,
    )

    return int(np.argmin(distances))


# ---------------------------------------------------------------------
# 6. Depth lookup
# ---------------------------------------------------------------------

def robust_depth_at_pixel(
    depth: np.ndarray,
    pixel: tuple[int, int],
    depth_scale: float = 1.0,
    window_radius: int = 3,
    min_depth_m: float = 0.05,
    max_depth_m: float = 10.0,
) -> float:
    """
    Estimate depth at a pixel using the median of a local valid-depth patch.
    """
    u, v = pixel
    h, w = depth.shape[:2]

    if not (0 <= u < w and 0 <= v < h):
        raise ValueError("Pixel lies outside the depth image")

    x0 = max(0, u - window_radius)
    x1 = min(w, u + window_radius + 1)
    y0 = max(0, v - window_radius)
    y1 = min(h, v + window_radius + 1)

    patch = depth[y0:y1, x0:x1].astype(np.float32) * depth_scale

    valid = patch[
        np.isfinite(patch)
        & (patch >= min_depth_m)
        & (patch <= max_depth_m)
    ]

    if len(valid) == 0:
        raise ValueError(f"No valid depth near pixel {(u, v)}")

    return float(np.median(valid))


# ---------------------------------------------------------------------
# 7. Back-project pixel + depth to 3D camera coordinates
# ---------------------------------------------------------------------

def backproject_to_3d(
    pixel: tuple[int, int],
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """
    Convert image pixel (u, v) and depth z into camera-frame XYZ.

    Camera convention:
        +X: right
        +Y: down
        +Z: forward
    """
    u, v = pixel
    z = float(depth_m)

    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy

    return np.array([x, y, z], dtype=np.float64)


# ---------------------------------------------------------------------
# 8. Transform camera coordinates into robot coordinates
# ---------------------------------------------------------------------

def transform_point(
    point_camera: np.ndarray,
    T_robot_camera: np.ndarray,
) -> np.ndarray:
    """
    Transform a 3D point from camera coordinates to robot coordinates.

    T_robot_camera is a 4 x 4 homogeneous transform:
        p_robot_h = T_robot_camera @ [x, y, z, 1]
    """
    if T_robot_camera.shape != (4, 4):
        raise ValueError("T_robot_camera must have shape (4, 4)")

    point_h = np.r_[point_camera, 1.0]
    transformed_h = T_robot_camera @ point_h

    return transformed_h[:3] / transformed_h[3]


# ---------------------------------------------------------------------
# 9. Complete placement target computation
# ---------------------------------------------------------------------

def compute_placement_target(
    rgb: np.ndarray,
    depth: np.ndarray,
    destination_text: str,
    instruction: str,
    segmenter: Callable[[np.ndarray, str], np.ndarray],
    intrinsics: CameraIntrinsics,
    T_robot_camera: np.ndarray,
    vlm_selector: Optional[
        Callable[[np.ndarray, str, int], int]
    ] = None,
    n_candidates: int = 8,
    erosion_radius: int = 5,
    min_component_area: int = 500,
) -> PlacementTarget:
    """
    Full pipeline:

        RGB
          -> destination segmentation
          -> mask cleanup and erosion
          -> FPS candidate generation
          -> numbered image
          -> VLM selection
          -> depth lookup
          -> 3D back-projection
          -> camera-to-robot transform
    """
    if depth.shape[:2] != rgb.shape[:2]:
        raise ValueError("RGB and depth image dimensions must match")

    # 1. Segment the destination surface/object.
    raw_mask = segment_destination(
        rgb=rgb,
        text=destination_text,
        segmenter=segmenter,
    )

    # 2. Remove noise and avoid the boundary.
    mask = clean_mask(
        raw_mask,
        erosion_radius=erosion_radius,
        min_component_area=min_component_area,
    )

    # 3. Generate spatially distributed candidate placement points.
    candidate_pixels = farthest_point_sampling(
        mask=mask,
        n_points=n_candidates,
    )

    # 4. Create the image shown to the VLM.
    annotated = draw_numbered_candidates(
        rgb=rgb,
        mask=mask,
        candidate_pixels=candidate_pixels,
    )

    # 5. Select a candidate.
    if vlm_selector is None:
        selected_index = choose_center_candidate(
            candidate_pixels=candidate_pixels,
            mask=mask,
        )
    else:
        selected_index = choose_candidate_with_vlm(
            annotated_rgb=annotated,
            instruction=instruction,
            candidate_pixels=candidate_pixels,
            vlm_selector=vlm_selector,
        )

    selected_pixel = tuple(
        int(x) for x in candidate_pixels[selected_index]
    )

    # 6. Get a robust depth estimate around the selected pixel.
    depth_m = robust_depth_at_pixel(
        depth=depth,
        pixel=selected_pixel,
        depth_scale=intrinsics.depth_scale,
    )

    # 7. Back-project into camera coordinates.
    camera_xyz = backproject_to_3d(
        pixel=selected_pixel,
        depth_m=depth_m,
        intrinsics=intrinsics,
    )

    # 8. Transform into robot coordinates.
    robot_xyz = transform_point(
        point_camera=camera_xyz,
        T_robot_camera=T_robot_camera,
    )

    return PlacementTarget(
        pixel=selected_pixel,
        camera_xyz=camera_xyz,
        robot_xyz=robot_xyz,
        mask=mask,
        annotated_image=annotated,
        candidate_pixels=candidate_pixels,
    )


# ---------------------------------------------------------------------
# 10. Robot execution interface
# ---------------------------------------------------------------------

def execute_place(
    target: PlacementTarget,
    robot_place_controller: Callable[[np.ndarray], None],
    approach_height_m: float = 0.10,
    release_height_m: float = 0.02,
) -> None:
    """
    Execute placement through a robot-specific controller.

    This function assumes robot_xyz is expressed in a frame where +Z is
    upward. If your robot uses another convention, transform the target
    before calling this function.
    """
    target_xyz = target.robot_xyz.copy()

    approach_xyz = target_xyz.copy()
    approach_xyz[2] += approach_height_m

    release_xyz = target_xyz.copy()
    release_xyz[2] += release_height_m

    # The actual controller must implement:
    #   move_to(approach_xyz)
    #   move_to(release_xyz)
    #   open_gripper()
    #   move_to(approach_xyz)
    #
    # Here the callback represents that robot-specific sequence.
    robot_place_controller(
        np.stack([approach_xyz, release_xyz], axis=0)
    )
```

A minimal example of the interfaces is:

```python
import numpy as np
import cv2

# -------------------------------------------------------------
# Example segmenter
# -------------------------------------------------------------

def my_segmenter(rgb: np.ndarray, text: str) -> np.ndarray:
    """
    Replace this with Grounded SAM or another open-vocabulary segmenter.

    This toy implementation assumes the destination is green.
    """
    return simple_color_segmentation(
        rgb,
        lower_rgb=(0, 80, 0),
        upper_rgb=(150, 255, 150),
    )


# -------------------------------------------------------------
# Example VLM adapter
# -------------------------------------------------------------

def my_vlm_selector(
    annotated_rgb: np.ndarray,
    instruction: str,
    n_candidates: int,
) -> int:
    """
    Replace this with your VLM API call.

    The VLM should see `annotated_rgb` and return one number from
    1 through n_candidates. This adapter converts it to zero-based indexing.
    """

    cv2.imwrite("placement_candidates.png", annotated_rgb)

    print(instruction)
    print(f"Candidate image saved with {n_candidates} candidates.")

    # Temporary deterministic choice:
    # choose candidate 1 in the displayed image.
    vlm_answer_one_based = 1

    return vlm_answer_one_based - 1


# -------------------------------------------------------------
# Camera calibration
# -------------------------------------------------------------

intrinsics = CameraIntrinsics(
    fx=615.0,
    fy=615.0,
    cx=320.0,
    cy=240.0,
    depth_scale=0.001,  # uint16 depth image stored in millimetres
)

# Obtain this from camera-robot extrinsic calibration.
T_robot_camera = np.eye(4)


# -------------------------------------------------------------
# Run placement target generation
# -------------------------------------------------------------

# rgb: H x W x 3 uint8 RGB image
# depth: H x W uint16 depth image
#
# rgb, depth = camera.capture()

target = compute_placement_target(
    rgb=rgb,
    depth=depth,
    destination_text="plate",
    instruction=(
        "Place the manipulated object safely on the plate. "
        "Choose a location away from the plate rim and other objects."
    ),
    segmenter=my_segmenter,
    intrinsics=intrinsics,
    T_robot_camera=T_robot_camera,
    vlm_selector=my_vlm_selector,
    n_candidates=8,
    erosion_radius=5,
)

print("Selected pixel:", target.pixel)
print("Camera-frame target [m]:", target.camera_xyz)
print("Robot-frame target [m]:", target.robot_xyz)


# -------------------------------------------------------------
# Robot-specific execution
# -------------------------------------------------------------

def my_robot_place_controller(waypoints: np.ndarray) -> None:
    approach_xyz = waypoints[0]
    release_xyz = waypoints[1]

    print("Move above target:", approach_xyz)
    print("Move to release pose:", release_xyz)
    print("Open gripper")
    print("Move back up")


execute_place(
    target=target,
    robot_place_controller=my_robot_place_controller,
)
```

The part corresponding directly to the paper’s placement procedure is:

```python
raw_mask = segment_destination(rgb, "plate", segmenter)
mask = clean_mask(raw_mask)
candidate_pixels = farthest_point_sampling(mask, n_points=8)
annotated = draw_numbered_candidates(rgb, mask, candidate_pixels)
selected_index = choose_candidate_with_vlm(
    annotated,
    "Choose a safe placement location on the plate.",
    candidate_pixels,
    vlm_selector,
)
```

In the paper, FPS is applied to the segmented placement region to create candidate locations, while the VLM chooses between those candidates. [Placement Candidates](https://www.alphaxiv.org/abs/2507.16713?page=5)

Two details are essential in a real robot implementation:

- **Segmentation alone is not enough.** The depth pixel must correspond to the actual supporting surface. For a plate, the selected depth may come from the object being placed, the plate rim, or a background surface, so depth validation and collision checking should be added.
- **`T_robot_camera` must be calibrated.** Back-projection produces a point in camera coordinates; the robot cannot use it until it is transformed into the robot or arm base frame.
