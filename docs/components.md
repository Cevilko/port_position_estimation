# What every script and node does

A reference for the moving parts of this repo: each one's job, what it consumes
and produces, how to run it, and the thing about it that is not obvious. For
*why* the environment is the way it is (versions, interpreters, startup order)
read `CLAUDE.md` first; for the dataset and the trained model see
`docs/dataset.md` and `docs/training.md`.

Everything is launched through `./run.sh <step>`, which picks the right
interpreter and sources ROS 2 where it is needed. Run `./run.sh` with no
arguments for the list.

## The pipeline, end to end

```
 isaacsim/scene.usd                     UR5e + 3 cameras + ROS 2 OmniGraph
          |
 randomize_visible_joints.py            pick a pose both ports are VISIBLE from
          |  /record_rosbag (Trigger)
 bag_recorder_node                      snapshot 3x(image+info) + resolved /tf
          |
 rosbags/<stamp>/                       recorded bag (gitignored, ~10 MB/frame)
          |
 extract_rosbag_samples.py              bag -> frames/ of JPEG + YAML
          |
 project_port_bboxes.py                 TF + intrinsics -> 2D boxes
          |
 export_yolo_dataset.py                 frames -> yolo_dataset/ (images+labels)
          |
 ./run.sh train                         -> runs/<name>/weights/best.pt
          |
 yolo_detector_node                     live images -> Detection2DArray
          |
 port_triangulator_node                 multi-view detections -> 3D + covariance
          |
 port_error_node                        estimate vs /tf truth -> distance
```

The first half builds a dataset; the second half consumes the model it trains.

---

# Scripts

## `isaacsim/randomize_visible_joints.py` — the sampler
`./run.sh sample [args]` · Isaac Sim's Python 3.11

Opens the scene, randomises the arm pose and the NIC-card fixture, and keeps
only poses from which **both SFP port entrances are genuinely visible** to the
centre camera. Each accepted pose fires `/record_rosbag` through an OmniGraph
service-client node in the stage.

Acceptance is three tests, not one: inside the camera frustum, the port's
outward normal within `--max-view-angle` of the camera, and not occluded —
the last measured against a small `distance_to_camera` render product, because
the scene has no colliders on the gripper or card for a raycast to hit. On a
1000-pose run that rejected 6340 out of frustum, 342 facing away and 289
occluded.

| | |
|---|---|
| Key args | `--samples N`, `--headless`, `--seed`, `--joint-span`, `--port-yaw-span` |
| **`--samples 0`** | open and play the scene, randomise nothing — for live testing |
| Produces | service calls; the bag is written by the recorder |

It also disables the scene's `Position_Controller`, which would otherwise pull
the arm straight back to one fixed pose and cap camera travel at ~9 mm.

## `scripts/extract_rosbag_samples.py` — bag to files
`./run.sh extract [bag]` · system Python 3.12

Turns a recorded bag into inspectable files. All of one trigger's messages
share a bag timestamp, which is what groups them into a frame.

```
rosbag_samples/<bag>/
  frames/00000/images/<camera>_image.jpg + .yaml
  frames/00000/tf_world_transforms/world_to_<frame>.yaml
  camera_info/<camera>_camera_info.yaml      (once — intrinsics do not change)
  MANIFEST.yaml
```

Defaults to the newest bag, where "newest" means the newest directory holding a
`metadata.yaml` — rosbag2 writes that on close, **so stop the recorder before
extracting** or you silently get the previous bag.

## `scripts/project_port_bboxes.py` — geometry to 2D boxes
`./run.sh bbox <frame> [--annotate out.jpg]` · system Python 3.12

Projects the SFP apertures into one extracted frame as axis-aligned boxes,
using that frame's own transforms and the camera intrinsics. This is the
labeller: no human ever draws a box in this project.

The entrance prims carry a pose but **no geometry**, so the aperture size is a
constant measured off the renders (12.20 × 7.15 mm), not read from the USD.
`--annotate` draws the result on the image, which is the quickest way to see
whether labels are sane.

## `scripts/export_yolo_dataset.py` — frames to a training set
`./run.sh yolo [bag] [--drop-inconsistent]` · system Python 3.12

Writes the Ultralytics layout: `images/{train,val}`, `labels/{train,val}`,
`data.yaml`. One class, `sfp_port` — the two ports are pixel-identical, so
telling them apart is a geometry question the transforms already answer
exactly. `--per-port` splits them into two classes if you want to try anyway.

Splits **by frame index, never by image**: the three cameras see one instant
from rigidly connected viewpoints, so a random split would put near-duplicates
on both sides and inflate the validation score.

Two traps worth knowing: it **appends** rather than replaces, so delete
`yolo_dataset/` for a clean export; and **use `--drop-inconsistent`**, or an
image keeps its label for one port while a second, visible-but-unlabelable port
is left as background.

## `scripts/dump_scene_contract.py` — scene to readable YAML
`./run.sh contract` · needs OpenUSD, not Isaac Sim

`isaacsim/scene.usd` is binary and gitignored, so nothing in a diff shows what
the scene publishes. This writes the tracked, readable proxy at
`docs/scene_contract.yaml` — cameras and intrinsics, topics, services, joints,
port frames. Re-run after any scene change; `./run.sh check` fails if it is
stale.

---

# ROS 2 nodes

All four live in `ros_ws/src/`. The last three **cannot be started with
`ros2 run`**: colcon gives their console scripts a `/usr/bin/python3` shebang,
and the detector needs torch, which only the venv has. Use `./run.sh`.

## `bag_recorder_node` — record on demand
`./run.sh recorder` · system Python 3.12

Serves `/record_rosbag` (`std_srvs/Trigger`). On each call it snapshots the
three images, the three `camera_info`s and a `/tf` message into an mcap bag.

**All-or-nothing**: if any of the seven inputs is missing the call is rejected
with the list of what was absent, rather than writing a partial frame. The
`/tf` it writes is *synthesised* — the `world -> X` transforms for the tracked
frames, resolved from a tf2 buffer **at the captured image's own stamp**, not
the newest available. That one change took projected-box error from ~9 px to
~0.7 px, because the cameras ride a moving arm.

Each instance opens its own `rosbag_<timestamp>` directory, so repeat runs no
longer collide.

## `yolo_detector_node` — detect ports in live images
`./run.sh detect [args]` · venv Python 3.12 (torch)

Subscribes to each camera's `image`, runs `best.pt`, publishes
`vision_msgs/Detection2DArray` on `<camera>/detections` and an annotated frame
on `<camera>/detections_image`. ~6 ms per frame, so the camera is the
bottleneck.

**`cv_bridge` is deliberately not used**: it is built against NumPy 1.x and
*segfaults* under the venv's NumPy 2.x. `image_convert.py` replaces it with a
numpy reshape that handles the three things a naive one gets wrong — `rgb8`
needs reversing to BGR, `step` is a row stride that can exceed
`width × channels`, and the result must be copied because `predict()`
letterboxes in place.

QoS defaults to RELIABLE, matching the scene; a BEST_EFFORT subscriber would
match nothing and sit silent.

## `port_triangulator_node` — detections to 3D
`./run.sh triangulate [args]` · venv Python 3.12

Takes each camera's `camera_info` and `detections` plus `/tf`, and publishes
one `geometry_msgs/PoseWithCovarianceStamped` per port on
`~/port_<n>/pose`, in the world frame. Measured against ground truth:
**median 0.43 mm**, 91% of ports localised.

The triangulation is a textbook DLT; the work is **correspondence**. The
detector emits unlabelled boxes, so nothing says which box in one camera is
which port in another. Every assignment of at most one detection per camera is
triangulated, scored by reprojection error, and the cheapest non-conflicting
ones are kept — preferring a 3-view fit over a lower-error 2-view one, because
two views can always be made to fit.

Covariance is first-order propagation of pixel noise, `inv(sum(J'J)/sigma^2)`.
The default `pixel_sigma=1.5` **over-states uncertainty roughly 8x**;
`pixel_sigma:=0.2` calibrates it against measured residuals. The pessimistic
default stands because the measurement came from a noise-free renderer.

**Port ids are positional, not semantic** — sorted by coordinate, so they match
the true `sfp_port_0/1` identity only ~51% of the time, i.e. a coin flip.

## `port_error_node` — score the estimates
`./run.sh error [args]` · venv Python 3.12

Reads the true port poses from `/tf`, takes the triangulated estimates, and
publishes the distance per port on `~/port_<n>/error` as a `std_msgs/Float64`
in metres. Reports a median of 0.55 mm over a replayed bag.

**The pairing is one-to-one and that is the point.** If each port took its
nearest estimate independently, both could score against the same one — which
makes the error look *better* the worse the estimator is, since one good
estimate gets counted twice and the port that was actually missed never gets
counted. It solves an assignment minimising total distance instead, and a port
left without a partner is reported unmatched rather than borrowing its
neighbour's.

---

# Tests

`./run.sh check` runs all of them plus the scene-contract freshness check. 129
tests, covering only what can be checked without a simulator — which is why new
logic belongs in pure functions.

| File | Covers |
|---|---|
| `test_extract_rosbag_samples.py` | quaternions, TF chains, image unpacking, path emission |
| `test_project_port_bboxes.py` | the projection and box arithmetic |
| `test_export_yolo_dataset.py` | label normalisation, the split rule, the inconsistency filter |
| `test_image_convert.py` | the `cv_bridge` replacement — encodings, row stride, writability |
| `test_triangulation.py` | DLT recovery, covariance behaviour, correspondence matching |
| `test_assignment.py` | one-to-one pairing, especially two ports never sharing an estimate |
