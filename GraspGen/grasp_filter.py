import yaml
import numpy as np

from scipy.spatial.transform import Rotation

data = yaml.safe_load(open("/tmp/cube_grasps.yml"))
grasps = data["grasps"]

DOWN = np.array([0.0, 0.0, -1.0])

results = []
for name, g in grasps.items():
    pos = np.array(g["position"])
    conf = g["confidence"]

    q = g["orientation"]
    # file gives w then xyz — scipy wants (x, y, z, w), so reorder
    quat_xyzw = [q["xyz"][0], q["xyz"][1], q["xyz"][2], q["w"]]
    R = Rotation.from_quat(quat_xyzw).as_matrix()

    approach = R[:, 2]                       # local Z axis, in world/object frame
    alignment = float(np.dot(approach, DOWN))

    results.append((name, conf, alignment, pos, R))

# top-down = alignment close to 1.0 (within ~25 degrees)
top_down = [r for r in results if r[2] > 0.90]
top_down.sort(key=lambda r: -r[1])           # sort by confidence

print(f"{len(top_down)} / {len(results)} grasps are top-down\n")
for name, conf, align, pos, R in top_down[:3]:
    print(f"{name}: confidence={conf:.3f}  alignment={align:.3f}  position(cube frame)={pos}")
# --- convert best top-down grasp to robot base frame ---
CUBE_CENTRE = np.array([0.497, -0.005, 0.0275])
TCP_OFFSET_Z = 0.108   # confirm with: ros2 run tf2_ros tf2_echo fr3_hand fr3_hand_tcp

best_name, best_conf, best_align, best_pos, best_R = top_down[0]

grasp_pos_base = CUBE_CENTRE + best_pos
tcp_target = grasp_pos_base.copy()
tcp_target[2] -= TCP_OFFSET_Z

hover = tcp_target.copy()
hover[2] += 0.10

print("grasp_pos_base:", grasp_pos_base)
print("tcp_target:    ", tcp_target)
print("hover:         ", hover)
X_AXIS = np.array([1.0, 0.0, 0.0])
Y_AXIS = np.array([0.0, 1.0, 0.0])

face_parallel = []
for name, conf, align, pos, R in top_down:
    finger_axis = R[:, 0]
    finger_axis = finger_axis / np.linalg.norm(finger_axis)

    dot_x = abs(np.dot(finger_axis, X_AXIS))
    dot_y = abs(np.dot(finger_axis, Y_AXIS))
    best_axis_alignment = max(dot_x, dot_y)   # 1.0 = perfectly face-parallel, 0.707 = 45° (corner)

    if best_axis_alignment > 0.95:            # within ~18 degrees of a face
        face_parallel.append((name, conf, align, best_axis_alignment, pos, R))

face_parallel.sort(key=lambda r: -r[1])

print(f"{len(face_parallel)} / {len(top_down)} top-down grasps are also face-parallel\n")
for name, conf, align, axis_align, pos, R in face_parallel[:3]:
    print(f"{name}: confidence={conf:.3f}  down_align={align:.3f}  face_align={axis_align:.3f}  pos={pos}")
