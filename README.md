# Port-insertion dataset generation (diplomski)

Generates a synthetic vision dataset for **robotic fibre-optic port insertion**
in Isaac Sim, delivered as ROS 2 bags.

A UR5e carrying an SFP fibre plug is posed at random in front of a NIC card on a
task board. Poses are kept only when **both SFP port entrances are visible to
the centre camera**, and each accepted pose is recorded as three camera images
plus the full TF tree. Because TF gives the world pose of both port entrances
and of every camera, each sample is automatically labelled — which makes this a
6-DoF pose-estimation / keypoint dataset, not just a pile of renders.

The task is the Intrinsic [AI for Industry Challenge](https://www.intrinsic.ai/events/ai-for-industry-challenge);
the toolkit it comes from is at `~/aic`.

## Quickstart

Two terminals.

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
./run.sh extract rosbags/rosbag_0   # or name one explicitly
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
isaacsim/randomize_visible_joints.py  rejection-samples arm poses until both
        │                             SFP entrances fall inside the centre
        │                             camera's frustum, then fires the scene's
        │                             ROS2 Service Client node
        │  /record_rosbag  (std_srvs/Trigger)
        ▼
ros_ws/src/bag_recorder_node          snapshots 3x(image + camera_info) + /tf
        │                             into an mcap bag on each call
        ▼
rosbags/<name>/                       the recorded dataset (gitignored)
        │
scripts/extract_rosbag_samples.py     one representative frame per camera, the
        ▼                             CameraInfo, and world-resolved TF for the
rosbag_samples/<name>/                cameras and both ports, as JPEG + YAML
```

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
| `scripts/extract_rosbag_samples.py` | bag → labelled samples |
| `scripts/dump_scene_contract.py` | scene → `docs/scene_contract.yaml` |
| `docs/scene_contract.yaml` | **generated**: what the scene publishes, in readable form |
| `tests/` | pytest for the extractor's pure functions |
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

## Verifying a change

```bash
./run.sh check
```

Runs the tests and confirms `docs/scene_contract.yaml` still matches the scene.
Everything else in the pipeline needs Isaac Sim running, so this is the only
cheap check — keep new extractor logic in pure functions so it stays that way.

## Known limitations

These are real and currently unfixed; CLAUDE.md has the detail.

- Bag timestamps are wall-clock while message headers are sim time.
- A trigger fired before all seven topics have arrived is rejected cleanly but
  still loses that sample.
- The scene references assets by absolute path into `~/IsaacLab`, three of
  which are already dead. It is not portable to another machine as-is.
- Bags hold one frame per accepted pose, not a continuous stream, so frame rate
  is whatever the sampler triggered at.
