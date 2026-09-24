# Port position estimation

Estimates the 3D position of SFP fibre-optic ports from three robot-mounted
cameras, in simulation. A UR5e in Isaac Sim is posed at random in front of a NIC
card; every accepted pose is recorded, automatically labelled from the transform
tree, and used to train a YOLO26 detector. At run time the detector's boxes from
three cameras are triangulated into world-frame positions with covariance, and
scored against ground truth.

**Measured end to end: median 0.43 mm position error**, detector mAP50 0.919,
~6 ms/frame inference. See [docs/training.md](docs/training.md) and
[docs/dataset.md](docs/dataset.md) for how those were obtained and what they do
and do not mean.

The task comes from Intrinsic's
[AI for Industry Challenge](https://www.intrinsic.ai/events/ai-for-industry-challenge).

---

## What you get, and what you do not

**Included in this repository:**

| | |
|---|---|
| All source: sampler, recorder, extractor, labeller, exporter, 4 ROS 2 nodes | |
| **The trained detector** | `runs/sfp_yolo26s_p2/weights/best.pt` (20 MB) |
| Its full training configuration and per-epoch record | `args.yaml`, `results.csv` |
| A readable proxy for the simulation scene | `docs/scene_contract.yaml` |

**Not included, and why:**

| | |
|---|---|
| `isaacsim/scene.usd` (9.1 MB) | binary USDC, gitignored. **Without it you cannot run the simulation half** — see below |
| The dataset (2757 images) and bags (9.9 GB) | regenerate with the pipeline; too large for git |
| `yolo26s.pt`, `yolo26n.pt` | Ultralytics' own weights, already public at [ultralytics/assets](https://github.com/ultralytics/assets/releases) `v8.4.0` |

**If you clone this without the scene**, you can still: run the full test suite,
run the trained detector on your own images or camera topics, and run the
triangulation and scoring nodes against any source of `sensor_msgs/Image`. You
cannot regenerate the dataset, retrain from scratch, or refresh the scene
contract. Ask the author for `scene.usd` if you need those.

---

## Prerequisites

This project is pinned to specific versions, and mismatches fail in confusing
ways rather than loudly. [CLAUDE.md](CLAUDE.md) explains each one.

**Hardware**

- An NVIDIA GPU. Training was done on an RTX 5090 (Blackwell, `sm_120`), which
  **requires a CUDA 12.8+ build of torch** — older builds will not run on it.
- ~32 GB VRAM for training at the default settings; inference needs ~1 GB.
- Disk: a 1000-episode run produces ~10 GB of bag plus ~324 MB extracted.

**Software**

| | Version | Notes |
|---|---|---|
| Ubuntu | 24.04 | |
| ROS 2 | **Jazzy** | Python 3.12 — this matters, see below |
| Isaac Sim | **5.1.0** | only needed for the simulation half |
| Python (system) | 3.12 | ships with ROS; runs the extractor and tests |
| torch | 2.9.1+cu130 | in a venv, see installation |
| ultralytics | 8.4.155 | ships YOLO26 |
| OpenUSD | 25.11 | only to regenerate the scene contract |

**Four Python interpreters are involved and picking the wrong one is the most
common failure here.** `./run.sh` selects the right one for every step — prefer
it to calling scripts directly. The short version:

| Interpreter | For |
|---|---|
| `~/isaacsim/python.sh` (3.11) | the sampler, inside Isaac Sim |
| `/usr/bin/python3` (3.12) | extractor, labeller, exporter, tests |
| `~/.venv/bin/python` (3.12) | anything touching torch — training, detector |
| OpenUSD venv | the scene contract dump |

Note the detector node runs under the **venv**, not the system interpreter, and
that works only because ROS 2 Jazzy and the venv are both Python 3.12 — rclpy's
`cp312` extension modules import cleanly under it. Isaac Sim's bundled 3.11 does
not get that luxury, which is why the simulation talks to ROS through an
OmniGraph node instead of `rclpy`.

---

## Installation

```bash
git clone https://github.com/Cevilko/port_position_estimation.git
cd port_position_estimation
```

**1. ROS 2 Jazzy** — follow the
[official instructions](https://docs.ros.org/en/jazzy/Installation.html), then:

```bash
sudo apt install ros-jazzy-vision-msgs ros-jazzy-rosbag2-storage-mcap
```

`vision_msgs` is required by the detector and triangulator; the mcap storage
plugin is what bags are written with.

**2. A venv with torch and ultralytics.** Anything importing torch runs here,
never under the system interpreter:

```bash
python3 -m venv ~/.venv
~/.venv/bin/pip install --index-url https://download.pytorch.org/whl/cu130 torch
~/.venv/bin/pip install ultralytics==8.4.155
```

Override the location with `TRAIN_PYTHON=/path/to/python` if you put it
elsewhere. Check it found your GPU:

```bash
~/.venv/bin/python -c "import torch; print(torch.cuda.get_device_name(0))"
```

**3. Build the ROS 2 workspace:**

```bash
./run.sh build
```

**4. Isaac Sim 5.1** — only for the simulation half. Install to `~/isaacsim`, or
point `ISAAC_PYTHON` at its `python.sh`. You also need `isaacsim/scene.usd`,
which is not in this repository.

**Verify the install:**

```bash
./run.sh check      # 129 tests + scene-contract freshness
```

The contract check reports `SKIP` without `scene.usd`; the tests should all
pass regardless.

---

## Usage

### Run the trained detector (no Isaac Sim needed)

The shipped model detects one class, `sfp_port`. On images or a directory:

```bash
~/.venv/bin/yolo detect predict \
    model=runs/sfp_yolo26s_p2/weights/best.pt \
    source=<image-or-directory> imgsz=1152 save=True
```

**Always pass `imgsz=1152`.** The ports are ~17 px at native resolution; the
ultralytics default of 640 shrinks them to ~9 px and the detector will look far
worse than it is.

### Run the live perception chain

Four terminals, or background them and use `./run.sh stop` to end them all.
Each needs a source of `sensor_msgs/Image` — Isaac Sim, a recorded bag, or a
real camera.

```bash
./run.sh detect         # images      -> vision_msgs/Detection2DArray
./run.sh triangulate    # detections  -> PoseWithCovarianceStamped, world frame
./run.sh error          # estimates   -> distance from /tf truth, per port
./run.sh rviz           # raw + annotated streams side by side
```

Two things that will otherwise cost you an afternoon:

- **`use_sim_time` must match whoever opened the scene.** It defaults to `true`,
  correct when `./run.sh sample` is driving (it adds a `/clock` publisher at
  runtime). A hand-opened Isaac Sim or a bag played without `--clock` publishes
  no `/clock`, and then the node still detects but its timers never fire — no
  status output and, worse, no warning when it stops receiving images. Pass
  `-p use_sim_time:=false`.
- **Nothing started in the background stops by itself.** The detector alone
  holds ~1 GB of VRAM indefinitely. `./run.sh stop` ends them;
  `./run.sh stop --dry-run` lists them first.

### Regenerate the dataset (needs Isaac Sim + `scene.usd`)

Two terminals — **the recorder must be running before the sampler starts**, or
poses are accepted and silently produce no data.

```bash
# terminal 1
./run.sh recorder

# terminal 2
./run.sh sample --samples 1000 --headless     # ~2h; ~12 attempts per accepted pose
```

Then, after stopping the recorder so rosbag2 writes its `metadata.yaml`:

```bash
./run.sh extract                              # bag  -> rosbag_samples/
./run.sh yolo --drop-inconsistent             #      -> yolo_dataset/
./run.sh train pretrained=yolo26s.pt epochs=100
```

`--drop-inconsistent` matters: without it, an image keeps its label for one port
while a second, visible-but-unlabelable port is left as background.

### Every command

| Step | Does |
|---|---|
| `build` | colcon build the ROS 2 workspace |
| `recorder` | serve `/record_rosbag`, writing an mcap bag per trigger |
| `sample` | randomise poses in Isaac Sim, trigger a recording per accepted one |
| `extract` | bag → images, transforms and intrinsics on disk |
| `bbox` | project the ports into one frame as 2D boxes (`--annotate` to draw) |
| `yolo` | extracted frames → an Ultralytics dataset |
| `train` | train a detector; defaults tuned for ~17 px objects |
| `detect` | run the detector on live camera topics |
| `triangulate` | multi-view detections → 3D pose with covariance |
| `error` | score estimates against `/tf`, paired one-to-one |
| `rviz` | open RViz with the project layout |
| `contract` | regenerate `docs/scene_contract.yaml` from the scene |
| `check` | tests + contract freshness — run before committing |
| `stop` | stop everything this script starts |
| `info`, `topics` | inspect a bag / list live topics |

`./run.sh` with no arguments prints the same list with full help.

---

## How it fits together

```
isaacsim/scene.usd                    UR5e + 3 cameras + ROS 2 OmniGraph
        │                             (binary; see docs/scene_contract.yaml)
        │
scripts/randomize_visible_joints.py   rejection-samples arm and fixture poses
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

No bounding box is ever drawn by hand: labels are projected from the recorded
transforms and the ports' known aperture size.

---

## Documentation

| | |
|---|---|
| [CLAUDE.md](CLAUDE.md) | **read before running anything** — versions, interpreters, startup order, and every sharp edge found so far |
| [docs/pipeline.md](docs/pipeline.md) | the whole flow stage by stage, with the maths |
| [docs/components.md](docs/components.md) | what each script and node does |
| [docs/dataset.md](docs/dataset.md) | how the dataset was built, and its caveats |
| [docs/training.md](docs/training.md) | the training run and what its score means |
| [docs/scene_contract.yaml](docs/scene_contract.yaml) | **generated** — what the scene publishes, in readable form |

---

## Published topics

From the scene:

| Topic | Type |
|---|---|
| `/{center,left,right}_camera/image` | `sensor_msgs/Image`, 1152×1024 `rgb8` |
| `/{center,left,right}_camera/camera_info` | `sensor_msgs/CameraInfo`, fx=fy=997.66, cx=576, cy=512 |
| `/tf` | `tf2_msgs/TFMessage` — arm links, camera optical frames, both port entrances |
| `/record_rosbag` | `std_srvs/Trigger` (served by `bag_recorder_node`) |

From the perception nodes:

| Topic | Type |
|---|---|
| `<camera>/detections` | `vision_msgs/Detection2DArray` |
| `<camera>/detections_image` | `sensor_msgs/Image` — annotated, viewable in RViz |
| `/port_triangulator_node/port_<n>/pose` | `geometry_msgs/PoseWithCovarianceStamped`, world frame |
| `/port_error_node/port_<n>/error` | `std_msgs/Float64` — metres from the true pose |

RViz has no `Detection2DArray` display, so `rviz/dipl.rviz` shows the annotated
image topics instead.

---

## Known limitations

- **Port identity is positional, not semantic.** The triangulator's `port_0`
  matches the true `sfp_port_0_entrance` about half the time. Telling them apart
  needs a third landmark or temporal tracking.
- **Detector recall is capped by the two side cameras**, which the sampler never
  vets for occlusion: 0.997 on the centre camera, 0.888 and 0.843 on the sides.
  A labelling artifact, not a model weakness.
- **The fibre cable is switched off in the scene** (`/UR5e_gripper/cable` is
  deactivated) and was for the whole pipeline, so no render contains one and the
  occlusion test never rejected a pose it would have blocked.
- **Covariance is deliberately pessimistic** — the default `pixel_sigma=1.5`
  over-states uncertainty roughly 8x against measured error.
- **Every number here comes from one room, one fixture and one renderer**, with
  validation drawn from the same run as training. They say the geometry is right;
  they say nothing about a real camera.
- The scene references assets by absolute path into `~/IsaacLab`, three of which
  are already dead. It is not portable between machines as-is.

---

## Licence

This project uses [Ultralytics](https://github.com/ultralytics/ultralytics)
YOLO26, which is licensed **AGPL-3.0**. `yolo_detector_node` imports it, and
`runs/sfp_yolo26s_p2/weights/best.pt` was trained from Ultralytics' `yolo26s.pt`
weights, so the trained model and its full configuration are distributed here
with the source rather than held back.

> **Relicensing to AGPL-3.0 is in progress.** Some package manifests still
> declare Apache-2.0 and are being updated; treat AGPL-3.0 as the intended
> licence for this work. The three files under
> `ros_ws/src/bag_recorder_node/test/` are Copyright 2015 Open Source Robotics
> Foundation and remain under their original Apache-2.0 terms.
