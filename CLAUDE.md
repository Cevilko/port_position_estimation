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
| `~/isaacsim/python.sh` | 3.11 | Isaac Sim, `omni.*`, `pxr` **only after Kit boots** | `isaacsim/randomize_visible_joints.py` |
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
training imports. It defaults to `model=yolo26s-p2.yaml` and **`imgsz=1152`**:
the ports are ~19 px in a 1152-wide frame, and the usual `imgsz=640` would
shrink them to ~11 px. The P2 config adds a stride-4 detection level, which is
where an object that small has enough grid cells to be localised.

Smoke-tested on the 60-image set: `yolo26n.pt` reaches mAP50 0.995 in 2.8
minutes, `yolo26n-p2.yaml` 0.995 in 3.2 minutes. Those numbers are a proof that
the chain runs, not a result — train and val are near-duplicate views of one
fixture position.

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
