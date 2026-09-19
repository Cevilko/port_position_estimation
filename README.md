# Port-insertion dataset generation (diplomski)

Generates a synthetic vision dataset for **robotic fibre-optic port insertion**
in Isaac Sim, delivered as ROS 2 bags.

A UR5e carrying an SFP fibre plug is posed at random in front of a NIC card on a
task board. Poses are kept only when **both SFP port entrances are genuinely
visible to the centre camera** — in frustum, facing it, and unoccluded — and
each accepted pose is recorded as three camera images plus resolved transforms.
Because TF gives the world pose of both port entrances and of every camera,
every sample is labelled automatically; no box is ever drawn by hand.

The dataset trains a YOLO26 detector, and the second half of the pipeline
consumes it: detections from three cameras are triangulated into 3D port
positions with covariance, and scored against the transform tree.

The task is the Intrinsic [AI for Industry Challenge](https://www.intrinsic.ai/events/ai-for-industry-challenge);
the toolkit it comes from is at `~/aic`.

## Quickstart

To generate a dataset, two terminals — the recorder must be up first.

```bash
./run.sh build                      # once, and after any change under ros_ws/src/

# terminal 1 -- must be running before terminal 2
./run.sh recorder

# terminal 2
./run.sh sample --samples 20 --seed 0
```

Then turn a recorded bag into inspectable samples:

```bash
./run.sh extract                    # newest bag in rosbags/ -> rosbag_samples/
./run.sh yolo --drop-inconsistent   # -> yolo_dataset/ (images + labels)
./run.sh train pretrained=yolo26s.pt
```

And to run the trained detector against a live scene, one terminal each:

```bash
./run.sh sample --samples 0         # scene only, no randomization
./run.sh detect                     # images   -> detections
./run.sh triangulate                # detections -> 3D poses + covariance
./run.sh error                      # poses    -> distance from /tf truth
./run.sh stop                       # none of these exit on their own
```

`./run.sh` with no arguments lists every step. **Read [CLAUDE.md](CLAUDE.md)
before running anything** — it carries the version, interpreter and
startup-order facts that are not guessable from the code, and the known sharp
edges.

## How it fits together

```
isaacsim/scene.usd                    UR5e + 3 cameras + ROS 2 OmniGraph
        │                             (binary; see docs/scene_contract.yaml)
        │
isaacsim/randomize_visible_joints.py  rejection-samples arm and fixture poses
        │                             until both SFP entrances are visible to
        │                             the centre camera -- in frustum, facing
        │                             it, unoccluded -- then fires the scene's
        │                             ROS2 Service Client node
        │  /record_rosbag  (std_srvs/Trigger)
        ▼
ros_ws/src/bag_recorder_node          snapshots 3x(image + camera_info) + /tf
        │                             into an mcap bag on each call
        ▼
rosbags/<name>/                       the recorded dataset (gitignored)
        │
scripts/extract_rosbag_samples.py     every recorded frame: three images, the
        ▼                             CameraInfo, and world-resolved TF for the
rosbag_samples/<name>/                cameras and both ports, as JPEG + YAML
        │
scripts/export_yolo_dataset.py        projected boxes -> an Ultralytics dataset
        ▼
yolo_dataset/ -> ./run.sh train -> runs/<name>/weights/best.pt
        │
yolo_detector_node -> port_triangulator_node -> port_error_node
                                      live detection, triangulation, scoring
```

Two documents cover this in depth:
**[docs/pipeline.md](docs/pipeline.md)** walks the whole flow stage by stage —
what each stage receives, the maths it applies, what it emits and how to check
it. **[docs/components.md](docs/components.md)** is the per-component reference:
what each script and node consumes, produces, and gets wrong if you hold it
incorrectly.

The trigger goes through an OmniGraph node inside the USD stage rather than
from Python because Isaac Sim's bundled Python 3.11 cannot host Jazzy's
Python 3.12 `rclpy` — see CLAUDE.md.

## Layout

| Path | What |
|---|---|
| `run.sh` | every pipeline step; handles sourcing and interpreter choice |
| `CLAUDE.md` | versions, interpreters, startup order, known sharp edges |
| `isaacsim/` | the USD scene and the pose sampler that drives it |
| `ros_ws/src/bag_recorder_node/` | the `/record_rosbag` Trigger service |
| `ros_ws/src/yolo_detector_node/` | runs the detector on live camera topics |
| `ros_ws/src/port_triangulator_node/` | multi-view detections → 3D pose + covariance |
| `ros_ws/src/port_error_node/` | estimates vs `/tf` truth, paired one-to-one |
| `scripts/extract_rosbag_samples.py` | bag → labelled samples |
| `scripts/project_port_bboxes.py` | transforms + intrinsics → 2D boxes |
| `scripts/export_yolo_dataset.py` | samples → Ultralytics dataset |
| `scripts/dump_scene_contract.py` | scene → `docs/scene_contract.yaml` |
| `docs/components.md` | **what every script and node does** |
| `docs/dataset.md`, `docs/training.md` | how the dataset and model were produced |
| `docs/scene_contract.yaml` | **generated**: what the scene publishes, in readable form |
| `tests/` | pytest for every pure function in the pipeline |
| `rviz/dipl.rviz` | RViz layout for the three camera streams |
| `rosbags/` | recorded bags (gitignored — large) |
| `rosbag_samples/` | extracted samples (committed — small, and the provenance record) |

## Published topics

From `docs/scene_contract.yaml`, which is regenerated from the scene itself:

| Topic | Type |
|---|---|
| `/center_camera/image`, `/left_camera/image`, `/right_camera/image` | `sensor_msgs/Image`, 1152×1024 `rgb8` |
| `/center_camera/camera_info`, `/left_camera/camera_info`, `/right_camera/camera_info` | `sensor_msgs/CameraInfo`, fx=fy=997.66, cx=576, cy=512 |
| `/tf` | `tf2_msgs/TFMessage` — arm links, the three `*_camera_optical` frames, and both `sfp_port_*_entrance` frames |
| `/record_rosbag` | `std_srvs/Trigger` (service, provided by `bag_recorder_node`) |

Added by the perception nodes when they are running:

| Topic | Type |
|---|---|
| `<camera>/detections` | `vision_msgs/Detection2DArray` |
| `<camera>/detections_image` | `sensor_msgs/Image` — annotated, viewable in RViz |
| `/port_triangulator_node/port_<n>/pose` | `geometry_msgs/PoseWithCovarianceStamped`, world frame |
| `/port_error_node/port_<n>/error` | `std_msgs/Float64` — metres from the true pose |

## Verifying a change

```bash
./run.sh check
```

Runs the tests and confirms `docs/scene_contract.yaml` still matches the scene.
Everything else in the pipeline needs Isaac Sim or a GPU, so this is the only
cheap check — keep new logic in pure functions so it stays that way.

## Known limitations

These are real and currently unfixed; CLAUDE.md has the detail.

- A trigger fired before all seven topics have arrived is rejected cleanly but
  still loses that sample, as does an image stamped newer than the newest `/tf`
  (~1% of poses).
- Port identity is positional, not semantic: the triangulator's `port_0` matches
  the true `sfp_port_0_entrance` about half the time. Distinguishing them needs
  a third landmark or temporal tracking.
- The trained detector's recall is capped by the two side cameras, which the
  sampler never vets for occlusion — 0.997 on the centre camera, 0.888 and
  0.843 on the sides.
- The scene references assets by absolute path into `~/IsaacLab`, three of
  which are already dead. It is not portable to another machine as-is.
- Bags hold one frame per accepted pose, not a continuous stream, so frame rate
  is whatever the sampler triggered at.
