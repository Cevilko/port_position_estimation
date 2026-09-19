# CLAUDE.md

Machine- and version-specific truth for this workspace. Read this before
running anything; most of it is not discoverable from the code.

## Versions — get these wrong and nothing works

| | |
|---|---|
| Isaac Sim | **5.1.0** at `~/isaacsim`, interpreter `~/isaacsim/python.sh` |
| ROS 2 | **Jazzy**, `/opt/ros/jazzy/setup.bash` |
| OS | Ubuntu 24.04 |

**There is also `~/isaacsim-6.0` (a 6.0.1 source build) on this machine. This
project does not use it.** 6.0 splits the ROS 2 bridge into
`isaacsim.ros2.{core,nodes,ui}`, moves the Core API to
`isaacsim.core.experimental.*`, and deprecates the direct prim-path inputs on
`ROS2PublishTransformTree` / `ROS2PublishJointState` that this scene relies on.
Do not mix its docs, paths or skills into this project.

`~/isaacsim-5.1.0` **does not exist**, despite what some docstrings in this
repo still claim. The 5.1 install is the unsuffixed `~/isaacsim`.

## Three Pythons, and picking the wrong one is the most common failure

| Interpreter | Version | Has | Use for |
|---|---|---|---|
| `~/isaacsim/python.sh` | 3.11 | Isaac Sim, `omni.*`, `pxr` **only after Kit boots** | `scripts/randomize_visible_joints.py` |
| `/usr/bin/python3` | 3.12 | `cv2`, `pytest`, ROS 2 Jazzy | the extractor, the tests |
| `~/miniconda3/bin/python3` | 3.13 | none of the above | **nothing here** — it is first on `PATH`, so it is what you get if you type `python3` without thinking |

`./run.sh` picks the right one for each step. Prefer it over calling these
directly.

**Isaac Sim's Python cannot host `rclpy`.** Isaac Sim 5.1 bundles Python 3.11
and NVIDIA compiles its internal ROS 2 libraries against 3.11; Jazzy on Ubuntu
24.04 is built against 3.12. Importing the system `rclpy` inside Kit crashes the
process. This is a hard version mismatch, not a flaky environment. The scene
works around it by calling `/record_rosbag` through a **ROS2 Service Client
OmniGraph node inside the USD stage** rather than from Python — see
`ros2_service_client` in `docs/scene_contract.yaml`.

## Startup order is load-bearing

1. `./run.sh build` — once, after any change under `ros_ws/src/`
2. `./run.sh recorder` — in its own terminal, and **before** step 3
3. `./run.sh sample --samples N` — fires `/record_rosbag` per accepted pose

**Source ROS 2 before Isaac Sim starts, never after.** The bridge resolves
`librmw` at extension load; if Jazzy is not in the environment when Kit boots,
the bridge comes up with zero topics and no error worth reading. `run.sh`
handles this.

If the recorder is not running, the sampler still accepts poses and reports the
service call as failed, silently producing no data.

## What the sampler randomizes

Two things, independently, per attempt:

- **The arm pose** -- six joints, uniform within `--joint-span` of a nominal
  pose. The cameras are mounted on the arm, so this is what moves the viewpoint.
- **The NIC card fixture** -- `/base_visual`, `/nic_card_mount_visual`,
  `/nic_card_visual` and `/sc_port_visual` are bolted together in reality, so
  they are posed as one rigid group: the same offset (`--port-pos-span`) and the
  same yaw (`--port-yaw-span`) applied to all four, yawing about
  `--port-pivot-prim`. **Yaw about world +z only** -- any other rotation tips
  the fixture off the table. None of the four has physics in its subtree, so
  this is a transform change with no collider to keep in sync, and the port
  entrance frames (children of the card) follow automatically into `/tf`.
  `--no-randomize-ports` restores the old fixed-fixture behaviour.

Both are vetted together by the acceptance test, so an accepted sample is one
where that arm pose sees those ports.

**The scene fights the sampler, and silently.** `/Graphs/Position_Controller`
drives the arm to one fixed joint command on every playback tick. Setting joint
positions without stopping it means the arm is pulled straight back: barely
visible when the capture is immediate, total once the capture happens after a
hold. The symptom is a camera that hardly moves however wide `--joint-span` is —
it was 9 mm of travel across 20 episodes — together with joints that never
settle. The sampler now disables every `IsaacArticulationController` node after
reading the nominal pose from it (order matters: disable first and the arm sags,
and every sample centres on the sag). `--keep-position-controller` opts out.

**Occlusion is tested against the rendered depth, not physics.** A physics
raycast would be useless: the whole scene has 11 colliders, none on the gripper
and none on the card, so a ray hits nothing. The sampler instead attaches a
small `distance_to_camera` render product to the sampling camera and compares
measured depth against the distance to each port's aperture centre and corners.
The test is one-sided on purpose — a port entrance is a hole, so an
unobstructed sample reads the *inside* of the cage and comes back farther than
the entrance; only depth clearly nearer means something is in the way.
`--no-occlusion-check` disables it, `--occlusion-tolerance` sets the margin.
On a fixed seed it rejects poses nothing else catches: accepted 6, out of
frustum 114, facing away 16, occluded 7.

**In frustum is not visible.** A port entrance is an opening in one face of the
cage, so once the camera passes the plane of that face it sees the back of the
card while the port's origin is still inside the frustum. With wide sampling the
camera reaches below the ports (they sit at z≈1.297) and this happened in 3 of
20 episodes, labelling boxes over blank card. Acceptance now also requires each
port's outward normal to be within `--max-view-angle` (70°) of the direction to
the camera.

## Things that will bite you

- **`bag_recorder_node` writes to a relative, timestamped `rosbag_<stamp>` uri**,
  one per `BagRecorderNode` instance, so repeat runs no longer collide the way a
  fixed name did (rosbag2 refuses to open a bag directory that already exists).
  `./run.sh recorder` runs from `rosbags/`, so bags land in `rosbags/`; the node
  logs the absolute path it opened. `./run.sh extract`, `info` and `yolo` all
  default to the newest bag, and a bag is only "newest" once it has a
  `metadata.yaml` — rosbag2 writes that on close, so stop the recorder before
  extracting or you will silently get the previous bag.
- **Stopping the recorder means signalling the node, not `ros2 run`.**
  `./run.sh recorder` ends in `exec ros2 run bag_recorder_node ...`, and
  `ros2 run` launches the node as a *child*. SIGINT to the `ros2 run` pid leaves
  the node alive, the writer open and `metadata.yaml` unwritten — the bag then
  looks like it is still being recorded and `extract` silently takes the
  previous one. Signal the process whose argv is
  `ros_ws/install/bag_recorder_node/lib/bag_recorder_node`, then wait for
  `metadata.yaml` to appear before extracting.
- **`pgrep -f` and `pkill -f` match the shell you are running them from.** The
  pattern appears in your own command line, so `pgrep -f bag_recorder_node`
  reports a hit when nothing is running, and `pkill -f` kills the wrapper shell
  issuing the kill. Both have wasted time here: once killing the wrong process,
  once reporting a phantom recorder, and once hanging a wait-loop that was
  waiting on itself. Match on something more specific (the install path), or
  filter with `ps -eo pid,args | grep <pat> | grep -v grep`, and treat a bare
  name match as unreliable.
- **A trigger fired before every topic has arrived records nothing.**
  `bag_recorder_node` checks all seven subscriptions first and rejects the
  `Trigger` with `success=False` and the list of missing topics, so this is a
  clean failure rather than a crash — but the sample is still lost. Watch the
  recorder's log for `Nothing recorded`.
- **Transforms are resolved at the captured image's stamp, not latched.** The
  cameras ride the arm and the arm never fully stops (the sampler reports
  0.1–0.2 rad/s at capture), so the newest `/tf` describes a different pose than
  the newest image. `bag_recorder_node` keeps a tf2 buffer and looks up
  `world -> frame` at `center_image.header.stamp`, which took projected-box
  error from ~9 px down to ~0.7 px. This needs the recorder's clock to match the
  sim-time stamps: `./run.sh recorder` passes `use_sim_time:=true`, and the
  sampler adds a `ROS2PublishClock` node to the scene graph at runtime because
  the scene itself publishes no `/clock`.
- **The recorded `/tf` is synthesised, not the raw tree.** It holds exactly the
  `world -> X` transforms for the tracked frames at that instant, already
  resolved. Do not expect the full articulation chain in a recorded bag.
- **Each camera now publishes its own `frame_id`** — `center_camera_optical`,
  `left_camera_optical`, `right_camera_optical` — set on the `ROS2CameraHelper`
  and `ROS2CameraInfoHelper` nodes and matching the camera's TF frame, so an
  image says which camera produced it. **Bags recorded before this change (e.g.
  `rosbags/rosbag_0`, and anything under `rosbag_samples/rosbag_0/`) still carry
  `frame_id: sim_camera` for all three**, which is ambiguous; identify cameras
  in those by topic, not `frame_id`. `extract_rosbag_samples.py` is unaffected
  either way — it resolves cameras through `TF_FRAME_ALIASES` against `/tf`.
- **A verification heuristic tuned at one scale lies when the scale changes.**
  The aperture detector used to check label quality needs filters that scale
  with the projected box. With fixed pixel limits it silently dropped close-up
  ports — five ports 56–65 px wide read as "not visible" when they were
  perfectly visible and correctly boxed. That looked like a pipeline fault and
  was not. Wide randomization makes boxes range ~10–65 px, so any check with
  hard-coded pixel limits will misreport.
- **The record gate fires spuriously, so frame count is not episode count.**
  Two causes, both sub-1% and neither worth chasing. The warm-up pulse the
  sampler fires at startup to initialise the service client is usually a no-op
  but has produced a real capture (21 recordings for 20 poses). And the gate
  itself sometimes fires twice for one pose: over 1000 episodes, 991 captures
  came from 1000 accepted poses *after* 12 were lost to tf2, meaning 3 extra
  triggers. Duplicates are byte-identical in pose and internally consistent;
  strays land at poses that were never accepted, with the camera pointed away
  from the card, and export as correct empty-label negatives. Nothing
  downstream may assume one frame per episode.
- **tf2 refuses to extrapolate, and that costs the occasional sample.** If an
  image's stamp is newer than the newest `/tf`, the lookup fails and the
  recording is rejected with `Nothing recorded`. Correct behaviour — better than
  a mislabelled frame. Measured at **1.2%** over 1000 episodes (12 lost). A
  20-episode run once lost 2 and suggested 10%; that was small-sample noise, so
  budget ~1% and do not size a run around the pessimistic figure.
- **`export_yolo_dataset.py` appends, it does not clean.** Filenames are
  prefixed by bag name, so a second export lands alongside the first. That is
  useful for accumulating runs into one dataset and a trap if you expected a
  replacement: delete `yolo_dataset/` first when you want only the latest run.
- **Export with `--drop-inconsistent`, or you teach the detector that a port is
  background.** A port too small (`--min-pixels`, default 8) or clipped by the
  edge cannot be labelled, so its box is dropped — but the image was still
  written, and if the *other* port in it labelled fine, that picture now calls
  the same object both foreground and background. It hit 76 images in a
  991-frame run. The flag skips those images instead, and leaves pure negatives
  alone: an image where no port projects is legitimately empty and worth
  keeping. Off by default so old exports reproduce; the current dataset was
  built with it.
- **Never edit `ros_ws/install/`.** Rebuild with `./run.sh build`.
- **The USD scene references assets by absolute path into `~/IsaacLab`,** and
  three of those references are already dead — see `unresolved_references` in
  `docs/scene_contract.yaml`. The scene is not portable to another machine as-is.

## Training a detector

A **fourth** interpreter, on top of the three above: `~/.venv/bin/python` (3.12)
is the only one with torch — `2.9.1+cu130`, whose arch list includes `sm_120`.
That matters: the RTX 5090 is Blackwell, and an older CUDA build of torch will
not run on it. `ultralytics` (8.4.155, which ships YOLO26) lives there too.
`./run.sh train` uses it via `TRAIN_PYTHON`.

`./run.sh train` deliberately does **not** source ROS 2 — Jazzy puts its own
`cv2` and `numpy` on `PYTHONPATH`, which shadow the venv's and break the
training imports.

Its defaults are all tuned around one fact: **the ports are ~17 px** (median,
measured over 991 frames).

| Default | Value | Why |
|---|---|---|
| `model` | `yolo26s-p2.yaml` | P2 adds a stride-4 level; at stride 8 a 17 px box spans ~2 cells |
| `imgsz` | 1152 | native width. At 640 the ports shrink to ~9 px |
| `batch` | -1 | auto-fit. A P2 head at 1152 holds a 288x288 stride-4 map, so the stock `batch=16` (tuned at 640, no P2) is not the same ask |
| `scale` | 0.25 | stock 0.5 rescales 50-150%, pushing boxes under the 8 px export floor |
| `mosaic` | 0.4 | stock 1.0 tiles four images, roughly halving object size — on top of `scale` |

Each is overridable per-run (`./run.sh train batch=8`) or by env var
(`YOLO_BATCH`, `YOLO_SCALE`, `YOLO_MOSAIC`, `YOLO_MODEL`, `YOLO_IMGSZ`).

**The default `model=` is an architecture file, so training starts from random
weights.** `pretrained=True` is the ultralytics default but is a no-op here: it
only loads anything when it is a *path*, or disables loading when `False`. The
smoke tests that scored 0.995 used `yolo26n.pt`, which *was* pretrained, so
switching to a P2 config silently dropped that. To get both a P2 head and COCO
initialisation, pass the weights explicitly — matching layers transfer, the
rest stay random:

```bash
./run.sh train pretrained=yolo26s.pt epochs=100
```

### Where the results go, and how to use them

`project` defaults to `$REPO/runs` and ultralytics picks the run name, so a
plain `./run.sh train` lands in **`runs/train/`** (then `train2`, `train3`, ...;
the smoke tests used explicit names and sit in `runs/smoke_*`). `runs/` is
gitignored. Inside:

| Path | What |
|---|---|
| `weights/best.pt` | **the model you want** — best val mAP50-95 |
| `weights/last.pt` | final epoch; resume with `resume=True` |
| `results.csv` | per-epoch losses and metrics |
| `args.yaml` | every setting the run used, including the defaults above |
| `confusion_matrix.png`, `*_curve.png` | val diagnostics |
| `val_batch*_pred.jpg` | predictions vs labels, the fastest sanity check |

Using `best.pt` afterwards, all through `~/.venv/bin/yolo` (never a ROS-sourced
shell):

```bash
# score it again, or against a different dataset
~/.venv/bin/yolo detect val model=runs/train/weights/best.pt data=yolo_dataset/data.yaml imgsz=1152

# run it on images
~/.venv/bin/yolo detect predict model=runs/train/weights/best.pt source=<img-or-dir> imgsz=1152 save=True

# export for deployment (ONNX/TensorRT)
~/.venv/bin/yolo export model=runs/train/weights/best.pt format=onnx imgsz=1152
```

**Always pass `imgsz=1152` at inference.** The checkpoint records it, but a
caller that omits it gets 640, which shrinks a 17 px port to ~9 px and the
detector will look far worse than it is.

**A P2 model cannot be fully initialised from stock weights.** Passing
`pretrained=yolo26s.pt` transfers only ~40% of tensors (`Transferred 360/902`)
because the P2 branch renumbers layers. Still worth passing, but it is not
COCO initialisation and does not compare to a `.pt`-to-matching-architecture
load.

Trained result, 2026-09-18: **mAP50 0.919, mAP50-95 0.869** in 44 minutes,
3 ms/image inference. **`docs/training.md` has the full write-up** — the run
itself, the log lines that look alarming and are not (AutoBatch's OOM probes,
a permanent `val/cls_loss=nan`), one printed metric that is wrong, and why
recall sits at 0.913. Short version of that last one: acceptance vets
`center_camera` only, so the side cameras carry correct-but-unlearnable labels
for occluded ports. Per-camera recall is **center 0.997**, left 0.888, right
0.843. It is a labelling artifact, not a model weakness, and not a
small-object problem — recall barely varies with box size.

Earlier smoke tests on the 60-image set (`yolo26n.pt` 0.995 in 2.8 min,
`yolo26n-p2.yaml` 0.995 in 3.2 min) were proof the chain runs, not results —
train and val there were near-duplicate views of one fixture position.

**How the current dataset was built, and what is wrong with it, is in
`docs/dataset.md`** — provenance, the exact sampler flags, yield and rejection
counts, box-size distribution, and the caveats that matter before you believe a
trained number. Read it before training on `yolo_dataset/` or regenerating it.

## Running the detector on live topics

`./run.sh detect` subscribes to the three camera topics, runs `best.pt` on each
frame and publishes `vision_msgs/Detection2DArray` on `<camera>/detections`,
plus an annotated `<camera>/detections_image`. Measured at **6 ms/frame** on
replayed bag data, so the camera is the bottleneck, not the model.

**`ros2 run` works for all of these, but only because the detector ships a
wrapper instead of a console script.** A setuptools `console_scripts` entry
point gets the shebang of the interpreter colcon built with — here
`/usr/bin/python3`, which has no torch — so `ros2 run yolo_detector_node
yolo_detector_node` used to import rclpy fine and then die on
`import ultralytics`. The interpreter was the *only* problem: the identical
installed module runs under the venv without modification. So
`yolo_detector_node` declares `scripts=['scripts/yolo_detector_node']` rather
than an entry point, and that wrapper execs `TRAIN_PYTHON` (default
`~/.venv/bin/python`). `port_triangulator_node` and `port_error_node` need no
wrapper — they never import torch and run under the system interpreter.

Any of this works only because Jazzy and the venv are **both Python 3.12**, so
rclpy's cp312 extension modules import cleanly under the venv. It is exactly
the coincidence Isaac Sim (3.11) does not enjoy.

**`cv_bridge` is not used, and must not be.** It is compiled against NumPy 1.x;
under the venv's NumPy 2.x, importing it does not raise — **it segfaults the
process**. `yolo_detector_node/image_convert.py` replaces it with a numpy
reshape. That is strictly less code for the 8-bit encodings a camera actually
publishes, and it is covered by tests. Two traps it handles that a naive
reshape does not: the scene publishes **`rgb8`**, which must be reversed to the
BGR ultralytics expects, and `step` is a row stride that may exceed
`width * channels`, so rows must be sliced individually or the image shears.
A third: the unpacked array must be **copied**, not just made contiguous —
`np.ascontiguousarray` returns the same read-only array when the data is
already contiguous, and `predict()` letterboxes in place.

**RViz cannot display `Detection2DArray`, so view the annotated image
instead.** Stock RViz has no such display, and the official
`ros-jazzy-vision-msgs-rviz-plugins` package (not installed here) only ships
**3D** displays — `Detection3DArray`, `Detection3D`, `BoundingBox3D`,
`BoundingBox3DArray`. There is no 2D equivalent, because a pixel-space box has
no place in a 3D scene. `rviz/dipl.rviz` therefore carries three extra Image
displays on `<camera>/detections_image`, which needs no plugin at all.
`ros-jazzy-vision-msgs-layers` overlays `Detection2DArray` on an image if you
want the real message rendered, but it is an **rqt** plugin, not an RViz one.
To get boxes into the 3D view properly, publish `Detection3DArray`: the port
aperture is a known 12.2 x 7.15 mm, so depth follows from the projected box
size and the intrinsics.

**Nothing started in the background stops by itself.** The detector, the
recorder, the sampler and RViz are all launched detached: they survive the
terminal that started them, get reparented to init, and keep their resources
until killed. The detector is the expensive one -- it holds **~1 GB of VRAM**
and runs inference on every frame for as long as something publishes.
`./run.sh stop` ends them (SIGINT first, so rclpy shuts down cleanly and
rosbag2 still writes `metadata.yaml`, then TERM for anything that ignored it);
`./run.sh stop --dry-run` lists them without killing anything. It matches on
full command lines through `ps`, never `pgrep -f`, for the self-match reason
above.

**`use_sim_time` must match who opened the scene.** `./run.sh detect` defaults
it to true, which is right when the sampler is driving -- the sampler adds a
`ROS2PublishClock` node at runtime. **The saved scene publishes no `/clock`**,
so an Isaac Sim opened by hand needs
`./run.sh detect -p use_sim_time:=false`. Get this wrong and the node still
detects, because callbacks use the message's own header stamp, but its clock
never advances, so its timers never fire: no throughput reports and, worse, no
"not receiving images" warning when something really is broken.

**Default QoS is RELIABLE, matching the scene.** `bag_recorder_node` receives
frames with a plain depth-1 reliable subscription, so the publisher is
reliable; a `BEST_EFFORT` subscriber would match nothing and sit silent. Pass
`-p best_effort:=true` for a camera that publishes best-effort. The node warns
every 10 s while it has received no images, which is what that failure looks
like from the outside.

## Triangulating the ports in 3D

`./run.sh triangulate` turns the per-camera detections into world-frame
positions with covariance, one `geometry_msgs/PoseWithCovariance` per port on
`/port_triangulator_node/port_<n>/pose`. It needs `./run.sh detect` running.

Measured over the 991-frame bag, detections straight from `best.pt`:

| | |
|---|---|
| ports localised | 1809 of 1982 (91%) |
| position error | **median 0.43 mm**, mean 0.92, p90 1.47, max 40.7 |
| reprojection residual | median 0.11 px |
| views used | 3-view 1244, 2-view 565 |

**The correspondence problem is the hard part, not the triangulation.** The
detector emits an unlabelled `sfp_port` per box, so nothing says which box in
one camera is which port in another. The node enumerates every assignment of
at most one detection per camera, triangulates each, scores it by reprojection
error, and greedily keeps the cheapest non-conflicting ones — preferring a
3-view fit over a lower-error 2-view one, because two views can always be made
to fit. Each detection is consumed once, so two ports cannot collapse onto one.

**Port ids are positional, not semantic.** Accepted points are sorted by
coordinate, so `port_0` and `port_1` swap if the fixture yaws far enough.
They are not tied to `sfp_port_0_entrance` / `sfp_port_1_entrance`.

**`pixel_sigma` is the covariance's only real knob, and the default is
deliberately pessimistic.** Covariance scales with its square. The default 1.5
px yields a median predicted sigma of (1.27, 2.63, 4.34) mm against a median
actual error of 0.43 mm — a median Mahalanobis distance of **0.18** where a
calibrated 3-dof estimate would sit near 1.5. In other words the node reports
roughly 8x more uncertainty than it has. Calibrating it means
`-p pixel_sigma:=0.2`, which the measured 0.11 px residual supports (corrected
for the fit's degrees of freedom, true noise is ~0.16-0.22 px). The default
stays conservative because these numbers come from a noise-free renderer and
an in-distribution detector; on real hardware they will not hold, and
over-stating confidence is the worse failure.

**Orientation is not estimated.** A triangulated point has none, so the
quaternion is identity and the rotation block of the covariance is set to
`orientation_variance` (1e6) rather than a small number that would imply a
measured rotation.

**The message is `PoseWithCovarianceStamped`**, in `world_frame` (default
`world`). The unstamped `PoseWithCovariance` carries neither the frame the
position is in nor the instant it describes, which is useless in a scene where
the cameras ride a moving arm; `-p publish_stamped:=false` still gives the bare
type if something demands it.

**Each port is stamped with the newest detection that went into it** -- not the
first camera's, and not the whole synchronised set's. Two reasons: a fused
estimate did not exist before its last input did, and a consumer resolving TF
at that stamp is then asking for a time the tree has already reached instead of
one it would have to extrapolate to, which tf2 refuses to do. Ports in the same
cycle can therefore carry different stamps when they were fitted from different
cameras.

## Scoring the estimates against the truth

`./run.sh error` reads the true port poses from `/tf`, takes the estimates from
`port_triangulator_node`, and publishes the distance per port on
`/port_error_node/port_<n>/error` as a `std_msgs/Float64` in metres. Over a
replayed bag it reports a median of **0.55 mm**, agreeing with the 0.43 mm
measured offline.

**The pairing is one-to-one, and that is the whole point.** Letting each port
take its nearest estimate independently lets both score against the *same*
estimate, which makes the reported error look better exactly when the
estimator is doing worse: one good estimate gets counted twice and the missing
one never gets counted. The node solves an assignment minimising total
distance instead, so each estimate is spent once, and a port with nothing left
to pair against is published as **unmatched** rather than borrowing its
neighbour's. Minimising the total rather than going greedy per port also makes
the result independent of the order the ports happen to be listed in.

**Truth lookups are all-or-nothing.** If any port's transform is missing at
that stamp, nothing is scored for that set, rather than scoring one port
against a transform from a different instant.

**Replaying a bag a second time silently stops the chain.** `ros2 bag play
--clock` restarts sim time from the beginning, so the clock jumps *backwards*
while the running nodes' tf2 buffers still hold data stamped in what is now the
future. Lookups stop matching and the node keeps reporting the same frozen
counts with no error. Restart the nodes (`./run.sh stop`, then start them
again) whenever you restart the bag.

**`docs/pipeline.md` walks the whole flow in detail** — sampling, recording,
extraction, projected labels, dataset export, training, live detection,
triangulation and validation, with the conventions and the maths that connect
them. Read it before changing any stage's contract with the next.

## Reading the scene without launching Isaac Sim

`isaacsim/scene.usd` is binary USDC and gitignored. Its tracked, readable proxy
is **`docs/scene_contract.yaml`** — camera prims and intrinsics, render
resolutions, every ROS topic and service the graph names, the arm joints, the
port entrance frames. Read that first; it answers most scene questions.

Regenerate it with `./run.sh contract` after **any** change to the scene, and
commit the result. `./run.sh check` fails if it is stale.

The dump needs OpenUSD but not Isaac Sim, and uses the pre-built OpenUSD 25.11
at `~/usd_root/python-usd-venv/bin/python`. Isaac Sim's `python.sh` will *not*
work — `pxr` only becomes importable after the Kit app boots.

## Verifying a change

```bash
./run.sh check      # pytest + scene-contract freshness. Run this before committing.
```

Only the pure functions in `scripts/extract_rosbag_samples.py` are covered
(quaternion maths, TF chain resolution, image unpacking, path emission).
Everything else needs the simulator. When you add logic to the extractor, add
it as a pure function and test it in `tests/` — that is the only layer where a
change can be checked cheaply.

## Context that lives outside this repo

- `~/aic` — the Intrinsic **AI for Industry Challenge** toolkit. This is the
  competition the task comes from; `docs/` there describes the task board, the
  scoring and the interfaces.
- `~/IsaacLab/etf_robotics_aic` — the Isaac Lab version of the same task, and
  the origin of the scene's asset references.
- `~/lerobot`, `~/podaci_aic_hdf5` — downstream policy-learning work.
- A more mature, spec-driven rebuild of this scene exists at
  <https://github.com/Cevilko/aic_isaacsim_ros>. It was vendored into this
  workspace and has been removed. It targets Isaac Sim **6.0**, so its paths
  and its `python_server`/8226 remote workflow do not apply here — but its
  `docs/ros2-bridge.md` is worth reading, and its `/clock`, `/joint_states` and
  `/aic/cheat/*` topics would fix the clock and frame_id problems above. Its
  four documented OmniGraph "gotchas" are undocumented by NVIDIA and unverified
  on 5.1; treat them as folklore until tested.

## Conventions

- Don't commit binaries. `*.usd`, `*.usda`, `*.mcap` and `rosbags/` are ignored
  on purpose; `git-lfs` is installed if you decide otherwise.
- `docs/scene_contract.yaml` is generated. Never hand-edit it.
- Keep `rosbag_samples/` paths repo-relative — they are committed, so an
  absolute `/home/...` in them is wrong the moment the repo moves. There is a
  test for this.
