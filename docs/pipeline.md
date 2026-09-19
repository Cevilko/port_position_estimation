# How the pipeline works, stage by stage

A detailed walk from a rendered frame to a validated 3D position. Each stage
says what it receives, what it does to it, what it emits, and how you check it
worked.

`docs/components.md` is the per-component reference; this is the data flow
between them. `CLAUDE.md` has the environment facts you need before running
anything.

## Conventions that run through everything

**Coordinate frames.** Every camera has an *optical* frame — `z` forward, `x`
right, `y` down — which is the convention `camera_info`'s `K` already assumes,
so intrinsics apply to camera-frame points with no axis permutation. The port
entrance frames have local `+y` as the outward normal, with the aperture lying
in the local `x`/`z` plane. World is `world`.

**A transform from TF is a pose, not a projection.** `world -> camera` from
`/tf` maps camera coordinates *into* the world: `X_world = R @ X_cam + t`.
Projecting needs its inverse. Getting this backwards produces confident,
plausible, wrong answers everywhere downstream, which is why it has a test.

**One instant per sample.** The cameras ride a moving arm, so "the newest
image" and "the newest transform" describe different robot poses. Every stage
resolves transforms at a *specific* stamp rather than taking the latest. This
is the single most consequential idea in the pipeline: ignoring it cost ~9 px
of label error, and fixing it brought that to ~0.7 px.

---

## Stage 1 — Pose sampling

**In:** `isaacsim/scene.usd`. **Out:** a `/record_rosbag` service call per
accepted pose. **Run:** `./run.sh sample --samples N --headless`

Each attempt draws an arm pose (six joints, uniform within `--joint-span` of
nominal) and a fixture pose (the NIC card, its mount, base and SC port moved as
one rigid group: a shared translation within `--port-pos-span` and a shared yaw
within `--port-yaw-span` about world `+z`). The arm carries the cameras, so the
first moves the viewpoint and the second moves the target.

The pose is then vetted by **three** independent tests, all against the centre
camera. A pose is accepted only if **both** ports pass **all three**:

1. **In frustum** — the port origin lies inside `Gf.Camera.frustum`.
2. **Facing the camera** — the port's outward normal is within
   `--max-view-angle` (70°) of the direction to the camera. Necessary because a
   port entrance is a hole in one face of a cage: once the camera passes the
   plane of that face it sees the *back* of the card while the origin is still
   inside the frustum. Without this, 3 of 20 episodes labelled blank card.
3. **Not occluded** — a small `distance_to_camera` render product is attached
   to the camera, and the measured depth at the aperture centre and corners is
   compared with the distance to those points. The test is deliberately
   *one-sided*: a port is a hole, so an unobstructed sample reads the inside of
   the cage and comes back *farther* than the entrance. Only depth clearly
   *nearer* means something is in the way. A physics raycast would be useless
   here — the scene has 11 colliders, none on the gripper or the card.

Yield on the 1000-pose run: 7971 attempts, rejecting 6340 out of frustum, 342
facing away, 289 occluded. Tests 2 and 3 rejected 631 poses the frustum test
alone would have passed.

Once accepted, the sampler waits `--record-hold-seconds`, waits for the arm to
stop moving, then pulses a Branch gate that drives the stage's OmniGraph
service client. **The trigger cannot come from Python**: Isaac Sim 5.1 bundles
Python 3.11 and Jazzy is built for 3.12, so importing `rclpy` inside Kit
crashes the process.

---

## Stage 2 — Recording

**In:** the trigger, plus live camera and `/tf` topics. **Out:** an mcap bag.
**Run:** `./run.sh recorder` (must already be running)

On each `Trigger`, `bag_recorder_node` writes seven messages under **one
identical bag timestamp**: three images, three `camera_info`s, and one `/tf`.

Two rules make the output trustworthy:

**All-or-nothing.** If any of the six sensor messages has not arrived, the call
is rejected with `success=False` and the list of missing topics. A partial
frame is never written.

**Transforms are resolved at the captured image's stamp.** The node keeps a
tf2 buffer and looks up `world -> frame` at
`center_image.header.stamp` for each tracked frame — the three camera optical
frames and both port entrances. The `/tf` it writes is therefore *synthesised*:
exactly those `world -> X` transforms, already resolved, not the raw
articulation tree. If any lookup fails the whole recording is rejected.

That last rule costs samples: tf2 refuses to extrapolate, so an image stamped
newer than the newest `/tf` is dropped. Measured at **1.2%** over 1000 poses.
That is the right trade — a dropped sample beats a mislabelled one.

This needs the recorder's clock to match sim-time stamps, so `./run.sh
recorder` passes `use_sim_time:=true`, and the sampler adds a
`ROS2PublishClock` node to the scene graph at runtime because the scene itself
publishes no `/clock`.

---

## Stage 3 — Extraction

**In:** a bag. **Out:** `rosbag_samples/<bag>/`. **Run:** `./run.sh extract`

Because all of one trigger's messages share a bag timestamp, that timestamp is
the grouping key. The extractor streams the bag, emits each complete group as a
frame, and drops it — no accumulation.

```
rosbag_samples/<bag>/
  MANIFEST.yaml                       frame count, and every path it wrote
  camera_info/<camera>_camera_info.yaml     once per camera
  frames/00000/
    frame.yaml                        index, bag_timestamp_ns, all paths
    images/<camera>_image.jpg         the decoded render
    images/<camera>_image.yaml        header stamp, frame_id, encoding, step
    tf_world_transforms/world_to_<frame>.yaml
```

A transform file holds the `world -> child` translation and quaternion, with
the stamp it was resolved at. Note `frame.yaml` records both
`requested_frame` (`center_camera`) and `recorded_child_frame_id`
(`center_camera_optical`) — the extractor resolves cameras through `/tf`
aliases, so it is unaffected by the frame naming.

`camera_info` is written once because intrinsics never change:
`fx = fy = 997.66`, `cx = 576`, `cy = 512`, 1152×1024, `plumb_bob` with zero
distortion.

**The newest bag is the newest directory containing `metadata.yaml`**, which
rosbag2 writes on close — so stop the recorder before extracting, or you
silently get the previous bag.

**Check it:** frame count should equal captures, and captures equal accepted
poses minus tf rejections. It will not equal episode count exactly; the record
gate occasionally fires twice.

---

## Stage 4 — Labelling by projection

**In:** a frame directory. **Out:** 2D boxes. **Run:**
`./run.sh bbox <frame> --annotate out.jpg`

No box is ever drawn by hand. For each port:

1. Build the aperture rectangle in the **entrance frame**: four corners at
   `(±w/2, 0, ±h/2)`, with `w = 12.20 mm`, `h = 7.15 mm`. The entrance prims
   carry a pose but **no geometry**, so these constants were measured off the
   rendered openings — across unoccluded frames the apertures come out at
   12.20 ± 0.35 by 7.14 ± 0.28 mm, tighter than the 13.4 × 8.5 mm nominal SFP
   cage mouth.
2. Corners to world with that frame's `world -> port` transform.
3. World to camera with the **inverse** of `world -> camera`.
4. Camera to pixels through `K`.
5. The box is the axis-aligned extent of the four projected corners — tight for
   a fronto-parallel port, a slight over-estimate as it turns away.

Everything in steps 2–4 comes from the frame's own files, so the label is
exactly as good as the transform/image alignment from Stage 2. Measured against
detected apertures: **0.70 px centre error, IoU 0.891**.

**Check it:** `--annotate` and look. A scale-aware check only — a verification
heuristic with fixed pixel limits once reported five perfectly good close-up
labels as failures.

---

## Stage 5 — Dataset export

**In:** extracted frames. **Out:** `yolo_dataset/`. **Run:**
`./run.sh yolo <bag> --drop-inconsistent`

```
yolo_dataset/
  images/{train,val}/<bag>_<frame>_<camera>.jpg
  labels/{train,val}/<bag>_<frame>_<camera>.txt     class cx cy w h, normalised
  data.yaml
```

**One class, `sfp_port`.** The two ports are pixel-identical; which is which is
a question about position in the scene that the transforms already answer
exactly. `--per-port` splits them if you want the detector to try.

**Split by frame index, never by image.** The three cameras see one instant
from rigidly connected viewpoints, so an image-level random split puts
near-duplicates on both sides and inflates the validation score. `--val-every 5`
gives 20% of *frames* to val; the 1000-frame export had 792 train frames and
199 val with **zero overlap**.

Two filters run per box:

- **too small** (`--min-pixels`, default 8) or **clipped by the image edge** —
  a partly-visible port has unknown true extent, so training on the clipped
  extent teaches the wrong size.
- **`--drop-inconsistent`** then skips the whole *image* if a port was dropped
  this way. Without it the image is kept, and if the *other* port labelled
  fine, that picture asserts the same object is both foreground and background.
  It hit 76 images in 991 frames.

An image with no boxes gets an **empty label file**, which is a legitimate
negative — not the same as a missing file. 210 of them in the current set,
almost all side cameras that legitimately see no port.

---

## Stage 6 — Training

**In:** `yolo_dataset/`. **Out:** `runs/<name>/weights/best.pt`. **Run:**
`./run.sh train pretrained=yolo26s.pt epochs=100`

Every default is set by one fact: **the ports are ~17 px** (median, measured
over 991 frames).

| Default | Value | Why |
|---|---|---|
| `model` | `yolo26s-p2.yaml` | P2 adds a stride-4 level; at stride 8 a 17 px box spans ~2 cells |
| `imgsz` | 1152 | native width; at 640 the ports become ~9 px |
| `batch` | -1 | auto-fit — a P2 head at 1152 holds a 288×288 stride-4 map and the stock 16 OOMs |
| `scale` | 0.25 | stock 0.5 rescales 50–150%, pushing boxes under the 8 px export floor |
| `mosaic` | 0.4 | stock 1.0 tiles four images, halving object size on top of `scale` |

`./run.sh train` deliberately does **not** source ROS: Jazzy puts its own `cv2`
and `numpy` on `PYTHONPATH`, shadowing the venv's and breaking the imports.

**A P2 model cannot be fully initialised from stock weights** — the P2 branch
renumbers layers, so `pretrained=yolo26s.pt` transfers 360 of 902 tensors. Worth
passing, but it is not COCO initialisation.

Result: mAP50 0.919, mAP50-95 0.869, 44 minutes, 3 ms/image. Recall of 0.913 is
**a labelling artifact, not a model weakness** — acceptance vets only the centre
camera, so the side cameras carry correct-but-unlearnable labels for occluded
ports. Per-camera recall is centre 0.997, left 0.888, right 0.843.

Full detail in `docs/training.md`, including one printed metric that is wrong.

---

## Stage 7 — Live detection

**In:** camera topics. **Out:** `<camera>/detections`. **Run:** `./run.sh detect`

Each image is unpacked to a BGR array, run through `best.pt`, and published as
`vision_msgs/Detection2DArray` with the **image's own header copied through**,
so a consumer can align detections with `/tf` at that instant. An annotated
image goes to `<camera>/detections_image`, which RViz can display directly —
RViz has no `Detection2DArray` display, and the official `vision_msgs` plugin
ships only 3D ones.

`cv_bridge` is not used: built against NumPy 1.x, it **segfaults** under the
venv's NumPy 2.x. The replacement handles three things a naive reshape gets
wrong — the scene publishes `rgb8` and ultralytics wants BGR; `step` is a row
stride that may exceed `width × channels`, which shears the image if ignored;
and the array must be *copied*, since `np.ascontiguousarray` returns the same
read-only buffer when data is already contiguous and `predict()` letterboxes in
place.

~6 ms/frame. Pass `imgsz=1152` at inference or a 17 px port becomes 9 px.

---

## Stage 8 — Triangulation

**In:** per-camera `camera_info` + `detections` + `/tf`. **Out:**
`PoseWithCovarianceStamped` per port. **Run:** `./run.sh triangulate`

The three detection streams are synchronised on their stamps, and each camera's
extrinsic is looked up at *its own* detection's stamp.

**Projection matrix.** `P = K @ [R' | -R't]`, the transpose inverting the
camera's pose in the world.

**Correspondence, which is the actual work.** The detector emits an unlabelled
`sfp_port` per box, so nothing says which box in one camera is which port in
another. The node enumerates every assignment of at most one detection per
camera, triangulates each, scores it by RMS reprojection error, discards
anything above `max_reprojection_error`, and greedily keeps the cheapest
non-conflicting ones — **preferring more views over lower error**, because two
views can always be made to fit. Each detection is consumed once, so two ports
cannot collapse onto one.

**Triangulation.** The DLT: each view contributes two rows
`u·P₃ − P₁` and `v·P₃ − P₂`; the point is the null space of the stack, via SVD.

**Covariance.** First-order propagation of isotropic pixel noise: with `J` the
2×3 Jacobian of the projection at the solution, the Fisher information is
`Σ JᵀJ / σ²` and the covariance is its inverse. Geometry dominates — a narrow
baseline gives a long thin ellipsoid along the viewing direction, which the
tests pin explicitly. Orientation is never estimated, so the rotation block is
set to `orientation_variance` (1e6) rather than a small number implying a
measurement.

**Each port is stamped with the newest detection that fed it** — a fused
estimate did not exist before its last input did, and a consumer resolving TF
at that stamp is asking for a time the tree has reached rather than one it must
extrapolate to.

Measured: **median 0.43 mm**, 1809 of 1982 ports localised, median reprojection
residual 0.11 px, 1244 three-view fits to 565 two-view.

---

## Stage 9 — Validation

**In:** estimates + `/tf` truth. **Out:** `~/port_<n>/error`. **Run:**
`./run.sh error`

For each synchronised set, the true port positions are looked up at the
estimate's stamp — **all-or-nothing across ports**, so nothing is scored if any
truth is missing. Then a **one-to-one assignment** minimising total distance
pairs truths to estimates, and each port's distance is published as a
`std_msgs/Float64` in metres.

**Why the constraint matters.** If each port took its nearest estimate
independently, both could score against the same one — which makes the reported
error look *better* the worse the estimator is, since a single good estimate
gets counted twice while the port actually missed is never counted at all. A
port left without a partner is reported **unmatched** instead of borrowing its
neighbour's. Minimising the total rather than going greedy also removes
dependence on the order the ports are listed in.

Reports median 0.55 mm live, agreeing with the 0.43 mm measured offline.

### Verifying the whole chain

| Level | Command | What it proves |
|---|---|---|
| Cheap | `./run.sh check` | 129 unit tests + scene contract freshness |
| Labels | `./run.sh bbox <frame> --annotate` | the projection is aligned |
| Model | `yolo detect val model=best.pt imgsz=1152` | in-distribution accuracy |
| End to end | `detect` + `triangulate` + `error` over a replayed bag | metric position error against truth |

**Replaying a bag twice silently freezes the chain.** `ros2 bag play --clock`
restarts sim time from zero, so the clock jumps *backwards* while the nodes'
tf2 buffers still hold future-stamped data. Counts stall with no error.
Restart the nodes whenever you restart the bag.

---

## What the numbers do not say

Every measurement above comes from one room, one fixture, one renderer, and a
validation split drawn from the same 1000-pose run as training. They establish
that the geometry is right and the chain is sound. They say nothing about a
different room, different lighting, or a real camera — and the covariance is
calibrated against noise-free renders, which is why `pixel_sigma` defaults to
an over-estimate. See the caveats in `docs/dataset.md` and `docs/training.md`.
