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

## Things that will bite you

- **`bag_recorder_node` writes to the hardcoded relative path `maj_beg`**, and
  rosbag2 refuses to open a bag directory that already exists. `./run.sh
  recorder` runs from `rosbags/` and refuses up front with instructions rather
  than letting it crash on startup. Rename the previous run before re-recording.
- **A trigger fired before every topic has arrived records nothing.**
  `bag_recorder_node` checks all seven subscriptions first and rejects the
  `Trigger` with `success=False` and the list of missing topics, so this is a
  clean failure rather than a crash — but the sample is still lost. Let the sim
  publish for a few seconds before sampling, and watch for `record: FAILED` in
  the sampler's output.
- **Bag timestamps are wall-clock, message headers are sim time.** The node does
  not set `use_sim_time`. Do not join the two clocks without converting.
- **All three cameras publish `frame_id: sim_camera`.** The image message does
  not say which camera it came from. `extract_rosbag_samples.py` recovers this
  through its `TF_FRAME_ALIASES` table; anything else consuming these bags must
  do the same or it will associate all three images with one frame.
- **Never edit `ros_ws/install/`.** Rebuild with `./run.sh build`.
- **The USD scene references assets by absolute path into `~/IsaacLab`,** and
  three of those references are already dead — see `unresolved_references` in
  `docs/scene_contract.yaml`. The scene is not portable to another machine as-is.

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
