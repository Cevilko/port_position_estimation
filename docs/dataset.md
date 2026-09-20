# The SFP port detection dataset

A record of how the current `yolo_dataset/` was produced, what is in it, and
what is wrong with it. Written after the 1000-episode run of 2026-09-18; the
numbers below are that run's, and re-running the pipeline will not reproduce
them exactly (the sampler is unseeded unless you pass `--seed`).

## Provenance

| | |
|---|---|
| Bag | `rosbags/rosbag_20260918_162421_582567` (9.8 GiB, 6937 messages) |
| Extraction | `rosbag_samples/rosbag_20260918_162421_582567` (991 frames, 2973 images) |
| Dataset | `yolo_dataset/` (2757 images, 5074 boxes) |
| Wall clock | 2h11m for 1000 accepted poses, headless |

Regenerate with, in this order and in separate terminals:

```bash
./run.sh build                                   # only after ros_ws/src changes
./run.sh recorder                                # leave running
./run.sh sample --samples 1000 --headless        # ~2h11m
# stop the recorder (SIGINT the NODE, see below) so metadata.yaml is written
./run.sh extract rosbags/<bag>
./run.sh yolo rosbag_samples/<bag> --drop-inconsistent
```

Every sampler argument was left at its default, which is what the dataset's
diversity rests on:

| Flag | Value | What it controls |
|---|---|---|
| `--joint-span` | 0.8 rad | arm pose spread about nominal, per joint |
| `--port-pos-span` | (0.06, 0.06, 0.0) m | NIC card fixture translation |
| `--port-yaw-span` | π | fixture yaw -- full 360 deg about world +z |
| `--max-view-angle` | 70 deg | how far a port may turn from the camera |
| `--occlusion-tolerance` | 0.005 m | depth margin for the occlusion test |
| `--settle-timeout` | 0.5 s | settle before the acceptance test |
| `--record-hold-seconds` | 2.0 s | hold after acceptance, before capture |

## Yield

Rejection sampling did most of the work: **7971 attempts for 1000 accepted
poses**, an 87% rejection rate.

| Outcome | Count |
|---|---|
| accepted | 1000 |
| out of frustum | 6340 |
| facing away | 342 |
| occluded | 289 |

The facing-away and occlusion checks together rejected 631 poses that the
frustum test alone would have accepted -- 39% as many as were accepted. Both
checks earn their cost.

Of the 1000 accepted poses, **991 were recorded**:

- **12 lost** to tf2 refusing to extrapolate (image stamp newer than the
  newest `/tf`). That is **1.2%**, not the ~10% a previous 20-episode run
  suggested -- 2/20 was small-sample noise, and CLAUDE.md's "budget a few
  percent" is the right figure.
- **+3 extra** captures from the record gate firing more than once.

## What is in `yolo_dataset/`

One class, `sfp_port`, covering both port entrances. Nothing in the pixels
distinguishes port 0 from port 1; which is which is a question about position
in the scene, and the recorded transforms answer it exactly.

| | Train | Val |
|---|---|---|
| Images | 2207 | 550 |
| Frames | 792 | 199 |

Split by frame index, `--val-every 5`, so 20% of frames. **Verified: no frame
index appears in both splits.** This matters more than it looks -- the three
cameras see the same instant from rigidly connected viewpoints, so an
image-level random split would put near-duplicates on both sides and inflate
the validation score.

5074 boxes across 2757 images. Box sizes, in pixels, at the native 1152x1024:

| | min | p10 | median | p90 | max |
|---|---|---|---|---|---|
| width | 8.0 | 11.3 | 17.0 | 29.5 | 136.5 |
| height | 8.0 | 10.8 | 17.1 | 29.3 | 116.2 |

A median of 17 px is why `./run.sh train` defaults to `imgsz=1152` and a P2
head: at the usual `imgsz=640` these become ~9 px, below what a stride-8
feature map can localise.

**210 images carry an empty label file.** That is deliberate. An empty label
says "no port here", which is not the same as a missing file, and those images
are legitimate negatives -- almost all from the side cameras, which are not
part of the acceptance test and often do not see a port at all.

## Findings from this run

### A visible port was being labelled as background

The exporter drops boxes that are too small (`--min-pixels`, default 8) or
clipped by the image edge, because their true extent is unknown. It used to
drop the box and keep the image. In this run that happened on **76 images
where the other port was labelled normally** -- the same picture then asserts
that a port is an object in one place and background in another.

`--drop-inconsistent` skips those images instead. It costs one sample per
image; keeping them corrupts two. Here it skipped 216 images (7% of 2973) and
the 323 unlabelable boxes went with them. Pure negatives, where no port
projects at all, are deliberately untouched.

The flag is **off by default**, so an export without it reproduces the old
behaviour. The dataset described here was built **with** it.

### The record gate sometimes fires twice

Frames `00519`/`00520` and `00770`/`00771` hold byte-identical camera and port
transforms: one capture written twice. Six frames (519, 520, 770, 771, 892,
946) also sit at poses that were never accepted -- the camera points away from
the NIC card entirely. Their empty labels are correct; one was inspected to
confirm the card is genuinely out of frame.

This is the same class of problem as the warm-up pulse already documented in
CLAUDE.md. Sub-1% and harmless -- duplicates are internally consistent and the
strays are usable hard negatives -- but **frame count is not episode count**,
and nothing downstream should assume it is.

### Per-camera coverage is uneven, by design

Acceptance is tested against `center_camera` only, so only it is guaranteed to
see both ports. Before the strict filter:

| Camera | labelled | dropped | no projection |
|---|---|---|---|
| center | 1887 | 83 | 12 |
| left | 1651 | 93 | 238 |
| right | 1612 | 147 | 223 |

The side cameras' "no projection" counts are not a fault: those ports are
outside that camera's view and correctly produce no box. The 12 on the centre
camera are the stray captures above.

## Caveats before trusting a trained number

- **Every frame is one fixture position and one arm pose.** Diversity comes
  entirely from the randomization ranges in the table above. Nothing varies
  lighting, materials, background or camera intrinsics, so a detector trained
  here has seen one room, one card and one robot.
- **The cable's visuals were disabled in the scene, for every run.** The fibre
  cable on the SFP plug in the gripper is not drawn in any frame of this
  dataset. Labels are unaffected -- they never read the pixels -- but the
  occlusion test measures rendered depth, and an invisible prim writes none, so
  the 289 "occluded" rejections cover the gripper and card only. Acceptance is
  more permissive than the physical setup, and the detector has never seen a
  cable. Note that `docs/scene_contract.yaml` does **not** record prim
  visibility, so nothing in the repo reveals this.
- **The labels are projections, not annotations.** They are geometrically
  exact to ~0.7 px given correct transforms, but they encode the *modelled*
  aperture (12.2 x 7.15 mm), not what a human would draw. As a port turns away
  the axis-aligned box over-estimates slightly; this is visible and expected.
- **No test split.** `val` is what the training run scores against. Holding
  out a genuine test set means a separate run with a different seed.
- **The side cameras carry unvetted labels.** Acceptance -- frustum, facing
  angle, occlusion -- is tested against `center_camera` only. `left_camera` and
  `right_camera` are labelled by projection whether or not the port is occluded
  or edge-on, so some of their boxes are geometrically correct and visually
  unlearnable. Measured cost: a detector trained on this data reaches 0.997
  recall on the centre camera and 0.888/0.843 on the sides. See
  `docs/training.md`.
