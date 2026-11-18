# Sensors: topics, frames and calibration

## Topics & frames

Sensors use **role-based names** (topic namespaces and TF frames aligned):

| Role | Point cloud | Images | TF frame(s) |
|------|-------------|--------|-------------|
| **navcam** | `/navcam/points` (+ `_filtered`) | `/navcam/{left,right}/image_rect` (+ `compressed`) | `navcam_optical_frame`, `navcam_right_optical_frame` |
| **loccam** | `/loccam/points` (+ `_filtered`) | `/loccam/rgb/image_raw`\|`image_rect`, `/loccam/{left,right}/image_rect`, `/loccam/stereo/image_raw` | `loccam_*` |
| **lidar** | `/lidar/points` (+ `_filtered`) | — | `lidar_frame` |

Both camera clouds carry RGB. The voxel filter republishes every `*/points` as
`*/points_filtered` (default leaf **0.01 m**, live-settable:
`ros2 param set /voxel_filter leaf_size <m>`).

**Mounting** — `lidar_frame` shares the `laser_frame` pose on `base_link`; the navcam
stereo pair is centered 0.35 m above it (baseline midpoint on the lidar x,y).

**Rates** — kept low so the large clouds don't saturate RViz/Zenoh: lidar **1 Hz**,
loccam/navcam **2 fps**. Raw `*/points` are tens of MB per message (loccam ≈ 30 MB), so
prefer `*/points_filtered` for visualization. The OAK IR dot projector is off
(`camera.i_laser_dot_brightness: 0`) — re-enable for better depth on low-texture surfaces.

Other telemetry:

- **Odometry** — `/odom` (component-odometry, Ackermann model; also publishes the
  `odom→base_footprint` TF) and `/controller/odom` (the controller's own wheel odometry).
- **IMUs** — `/controller/imu/data` (base controller, frame `imu`) and the external WIT IMU
  under `/imu_wit/` (`data_raw`, `mag`, `temperature`, `orientation`).
- **Joint feedback** — `/controller/joint_states` (real wheel encoders + steering servos),
  consumed by `robot_state_publisher` and `component-odometry`.

## Camera calibration & projection

Every image stream is paired with a `sensor_msgs/CameraInfo` topic (`.../camera_info`)
carrying the matrices needed to project between 3D and pixels:

- **`k`** — 3×3 intrinsics `[fx 0 cx; 0 fy cy; 0 0 1]` (for **raw** images).
- **`d`** — distortion coefficients (`distortion_model`); empty when the image is already rectified.
- **`r`** — 3×3 rectification rotation.
- **`p`** — 3×4 projection matrix (for **rectified** images). Project an optical-frame point
  `X` to a pixel with `[u v w]ᵀ = p · [X Y Z 1]ᵀ`, then `(u/w, v/w)`. For the right camera of
  a stereo pair, `p[3] = −fx · baseline`, so `baseline = −p[3] / fx`.

Rule of thumb: use **`p`** with the `image_rect` streams (and the depth-aligned streams);
use **`k` + `d`** only if you consume a raw/unrectified image.

| CameraInfo topic | Res | Model | fx / fy | cx / cy | Optical frame | Use with |
|------------------|-----|-------|---------|---------|---------------|----------|
| `/navcam/left/camera_info` | 1024×768 | plumb_bob (rectified, `d=[]`) | 650.17 | 510.67 / 380.71 | `navcam_optical_frame` | `/navcam/left/image_rect`, `/navcam/points` (reference) |
| `/navcam/right/camera_info` | 1024×768 | plumb_bob | 650.17 | 510.67 / 380.71 | `navcam_right_optical_frame` | `/navcam/right/image_rect` — `p[3]=−156.16` → baseline **0.240 m** |
| `/loccam/rgb/camera_info` | 1280×720 | rational_polynomial | 560.24 | 625.39 / 355.07 | `loccam_rgb_camera_optical_frame` | `/loccam/rgb/image_raw` (raw, has `d`) / `image_rect` |
| `/loccam/stereo/camera_info` | 1280×720 | rational_polynomial | 560.24 | 625.39 / 355.07 | `loccam_rgb_camera_optical_frame` | `/loccam/stereo/image_raw` (depth, RGB-aligned) + `/loccam/points` |
| `/loccam/left/camera_info` | 1280×720 | rational_polynomial | 566.00 | 634.97 / 358.40 | `loccam_left_camera_optical_frame` | `/loccam/left/image_rect` — `p[3]=−42.39` → baseline **0.075 m** |
| `/loccam/right/camera_info` | 1280×720 | rational_polynomial | 563.62 | 671.86 / 378.97 | `loccam_right_camera_optical_frame` | `/loccam/right/image_rect` |

For depth→3D, the point clouds (`/navcam/points`, `/loccam/points`) are already deprojected
using these matrices; for image overlays, project into the reference camera
(`/navcam/left/camera_info` for navcam, `/loccam/rgb/camera_info` for the loccam RGB-aligned
depth). Intrinsics scale with resolution — re-read `camera_info` if you change a camera's
resolution rather than hardcoding these values.

### Depth images

The per-pixel depth ("depth matrix") lives in these `sensor_msgs/Image` topics. **Watch the
encoding — the two cameras differ**: navcam depth is float **metres**, loccam depth is
uint16 **millimetres**.

| Depth topic | Encoding | Units | Res | Frame | Deproject with |
|-------------|----------|-------|-----|-------|----------------|
| `/navcam/depth/image_raw` | `32FC1` | **metres** (float32; 0/NaN = invalid) | 1024×768 | `navcam_optical_frame` | `/navcam/left/camera_info` |
| `/loccam/stereo/image_raw` | `16UC1` | **millimetres** (uint16; 0 = invalid) | 1280×720 | `loccam_rgb_camera_optical_frame` | `/loccam/stereo/camera_info` (RGB-aligned) |
| `/navcam/disparity/image_raw` | `16UC1` | disparity, **not** depth | 1024×768 | `navcam_optical_frame` | depth = `fx · baseline / disparity` (fx 650.17, baseline 0.240 m) |

Deproject a depth pixel `(u, v)` with the paired `camera_info` intrinsics:
`Z = depth(u,v)` (convert mm→m for loccam), `X = (u − cx)·Z/fx`, `Y = (v − cy)·Z/fy`. The
result is in that topic's optical frame; use TF to move it into `base_link`/`odom`.
