#!/usr/bin/env bash
#
# Entry point for every step of the dataset pipeline.
#
# This exists because the startup order and the choice of interpreter are both
# load-bearing and neither is guessable:
#
#   * the ROS 2 bridge needs Jazzy sourced BEFORE Isaac Sim starts, or it
#     cannot find librmw and comes up with no topics at all;
#   * three different Python interpreters are involved, and picking the wrong
#     one fails in confusing ways (see CLAUDE.md);
#   * the recorder must already be running when the sampler fires its trigger.
#
# Run `./run.sh` with no arguments for the list of steps.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
# Isaac Sim 5.1. NOT ~/isaacsim-6.0 (a 6.0 source build, different API surface)
# and NOT ~/isaacsim-5.1.0 (does not exist, despite what some docstrings say).
ISAAC_PYTHON="${ISAAC_PYTHON:-$HOME/isaacsim/python.sh}"
# The system interpreter, which has cv2 + pytest + ROS 2. Spelled absolutely on
# purpose: the conda python that comes first on PATH has none of them.
SYSTEM_PYTHON="${SYSTEM_PYTHON:-/usr/bin/python3}"
# A fourth interpreter, and the only one with torch: training needs CUDA wheels
# that none of the other three have. Blackwell (the RTX 5090, sm_120) needs a
# cu12.8+ build, so do not swap this for an older torch.
TRAIN_PYTHON="${TRAIN_PYTHON:-$HOME/.venv/bin/python}"

die() { echo "error: $*" >&2; exit 1; }

require_file() {
    [[ -e "$1" ]] || die "$2"
}

with_ros() {
    require_file "$ROS_SETUP" "no ROS 2 at $ROS_SETUP (override with ROS_SETUP=...)"
    # ROS 2's setup scripts read unset variables (AMENT_TRACE_SETUP_FILES and
    # friends), so `set -u` makes sourcing them fail outright. Drop it for the
    # duration and put it back afterwards.
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    if [[ -f "$REPO/ros_ws/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "$REPO/ros_ws/install/setup.bash"
    fi
    set -u
}

usage() {
    cat <<'USAGE'
usage: ./run.sh <step> [args...]

Pipeline, in the order you run it:

  build              colcon build the ROS 2 workspace (--symlink-install, so
                     later source edits take effect without rebuilding)
  recorder           start bag_recorder_node and wait for /record_rosbag calls.
                     Leave this running in its own terminal.
  sample [args]      launch Isaac Sim, randomize arm poses with both SFP port
                     entrances kept in view, and trigger a recording per
                     accepted pose. Args pass through to
                     isaacsim/randomize_visible_joints.py
                     (e.g. ./run.sh sample --samples 20 --seed 0 --headless)
  extract [bag]      pull sample frames + TF + CameraInfo out of a recorded bag
                     into rosbag_samples/ (default: rosbags/rosbag_0)
  detect [args]      run the trained detector on live camera topics, publishing
                     vision_msgs/Detection2DArray per camera. Args pass through
                     as ROS params (model:=... conf:=... imgsz:=...).
                     Use this, NOT `ros2 run` -- see the note below.

Inspection and verification -- none of these need Isaac Sim:

  yolo [bag]         export an extracted bag as a YOLO detection dataset
                     (default: newest extraction in rosbag_samples/)
  train [args]       train a detector on yolo_dataset/. Defaults are tuned for
                     ~17 px objects: imgsz=1152 (native width; the usual 640
                     would shrink them to ~9), batch=-1 (auto-fit, since a P2
                     head at 1152 is memory-hungry), and reduced scale/mosaic
                     augmentation. NOTE: the default model= is an architecture
                     file, so training starts from RANDOM WEIGHTS -- pass
                     pretrained=yolo26s.pt to start from COCO instead.
                     Args pass through to the yolo CLI (model=, epochs=, ...).
  bbox <frame>       project the SFP port entrances into an extracted frame as
                     2D boxes; --annotate <out.jpg> draws them on the image
  contract           regenerate docs/scene_contract.yaml from isaacsim/scene.usd
  test               run the pytest suite
  info [bag]         ros2 bag info on a recorded bag
  topics             list the ROS 2 topics currently live
  check              test + contract, and report if the contract is stale.
                     This is the "did I break anything" command.
  rviz               open RViz with the project layout

Environment overrides: ROS_SETUP, ISAAC_PYTHON, SYSTEM_PYTHON.
USAGE
}

step="${1:-}"
[[ $# -gt 0 ]] && shift || true

case "$step" in
build)
    with_ros
    cd "$REPO/ros_ws"
    # --symlink-install: without it, install/ holds COPIES, and editing a source
    # file while running the stale installed copy is the worst class of bug
    # there is -- the code you read is not the code that runs.
    colcon build --symlink-install "$@"
    ;;

recorder)
    with_ros
    require_file "$REPO/ros_ws/install/setup.bash" "workspace not built -- run ./run.sh build first"
    # bag_recorder_node writes to a relative, timestamped 'rosbag_<stamp>' uri,
    # so running from rosbags/ keeps that output with the other bags instead of
    # wherever the shell happened to be. The node logs the absolute path it
    # opened on startup.
    mkdir -p "$REPO/rosbags"
    cd "$REPO/rosbags"
    echo "recorder: writing to rosbags/rosbag_<timestamp>; waiting for /record_rosbag"
    # use_sim_time, because the recorder resolves transforms at an image's
    # stamp and those stamps are simulation time. The sampler adds a /clock
    # publisher to the scene graph so this clock has something to follow.
    exec ros2 run bag_recorder_node bag_recorder_node --ros-args -p use_sim_time:=true
    ;;

sample)
    require_file "$ISAAC_PYTHON" "no Isaac Sim python at $ISAAC_PYTHON (override with ISAAC_PYTHON=...)"
    require_file "$REPO/isaacsim/scene.usd" "isaacsim/scene.usd is missing. It is gitignored --
       see docs/scene_contract.yaml for what it should contain, and CLAUDE.md
       for where to get it."
    # ROS 2 must be sourced BEFORE Isaac Sim starts, not after: the bridge
    # resolves librmw at extension load. Source it and the bridge comes up with
    # no topics and no error worth reading.
    with_ros
    exec "$ISAAC_PYTHON" "$REPO/isaacsim/randomize_visible_joints.py" "$@"
    ;;

extract)
    with_ros
    exec "$SYSTEM_PYTHON" "$REPO/scripts/extract_rosbag_samples.py" "$@"
    ;;

detect)
    # The one step that needs ROS 2 *and* torch in the same process. That works
    # only because Jazzy and the venv are both Python 3.12, so rclpy's cp312
    # extension modules load under the venv interpreter. `ros2 run` cannot do
    # this: colcon gives the console script a /usr/bin/python3 shebang and that
    # interpreter has no torch.
    with_ros
    require_file "$TRAIN_PYTHON" "no venv interpreter at $TRAIN_PYTHON (override with TRAIN_PYTHON=...)"
    require_file "$REPO/ros_ws/install/setup.bash" "workspace not built -- run ./run.sh build first"
    model="${YOLO_WEIGHTS:-$REPO/runs/sfp_yolo26s_p2/weights/best.pt}"
    require_file "$model" "no trained weights at $model.
       Train first with ./run.sh train, or point YOLO_WEIGHTS at a .pt file."
    args=()
    [[ "$*" == *model:=* ]] || args+=("-p" "model:=$model")
    cd "$REPO"
    exec "$TRAIN_PYTHON" -m yolo_detector_node.yolo_detector_node --ros-args \
        -p use_sim_time:=true "${args[@]}" "$@"
    ;;

yolo)
    with_ros
    exec "$SYSTEM_PYTHON" "$REPO/scripts/export_yolo_dataset.py" "$@"
    ;;

train)
    yolo_bin="$(dirname "$TRAIN_PYTHON")/yolo"
    require_file "$yolo_bin" "no yolo CLI at $yolo_bin.
       Training needs torch + ultralytics in $TRAIN_PYTHON:
           $TRAIN_PYTHON -m pip install ultralytics
       Override the interpreter with TRAIN_PYTHON=..."
    require_file "$REPO/yolo_dataset/data.yaml" "no dataset -- run ./run.sh yolo first"
    # Only supply defaults the caller has not set: the yolo CLI takes the LAST
    # occurrence of a repeated key, so emitting both would work but reads as a
    # contradiction in ps output and in the run's saved args.
    defaults=()
    [[ "$*" == *model=* ]]  || defaults+=("model=${YOLO_MODEL:-yolo26s-p2.yaml}")
    [[ "$*" == *imgsz=* ]]  || defaults+=("imgsz=${YOLO_IMGSZ:-1152}")
    [[ "$*" == *data=* ]]   || defaults+=("data=$REPO/yolo_dataset/data.yaml")
    [[ "$*" == *project=* ]] || defaults+=("project=$REPO/runs")
    # A P2 head at imgsz=1152 carries a 288x288 stride-4 feature map, so the
    # usual batch=16 (tuned at 640, no P2) is not the same ask. -1 fits the
    # batch to the card instead of guessing.
    [[ "$*" == *batch=* ]]  || defaults+=("batch=${YOLO_BATCH:--1}")
    # The ports are ~17 px at native resolution. Ultralytics defaults to
    # mosaic=1.0 (four images tiled, roughly halving object size) and scale=0.5
    # (a 50-150% rescale on top), which together push a large share of boxes
    # below the 8 px floor they were exported at. Keep some of both for
    # robustness, far less than stock.
    [[ "$*" == *scale=* ]]  || defaults+=("scale=${YOLO_SCALE:-0.25}")
    [[ "$*" == *mosaic=* ]] || defaults+=("mosaic=${YOLO_MOSAIC:-0.4}")
    # Deliberately NOT sourcing ROS: it puts its own cv2 and numpy on
    # PYTHONPATH, which shadow the venv's and break the training imports.
    exec "$yolo_bin" detect train "${defaults[@]}" "$@"
    ;;

bbox)
    with_ros
    exec "$SYSTEM_PYTHON" "$REPO/scripts/project_port_bboxes.py" "$@"
    ;;

contract)
    exec "$SYSTEM_PYTHON" "$REPO/scripts/dump_scene_contract.py" "$@"
    ;;

test)
    with_ros
    exec "$SYSTEM_PYTHON" -m pytest "$REPO/tests" "$@"
    ;;

info)
    with_ros
    bag="${1:-}"
    if [[ -z "$bag" ]]; then
        # Bags are named by timestamp, so there is no fixed path to default to.
        bag=$(ls -1dt "$REPO"/rosbags/*/ 2>/dev/null | while read -r dir; do
                  [[ -f "$dir/metadata.yaml" ]] && { echo "${dir%/}"; break; }
              done)
        [[ -n "$bag" ]] || die "no bag found in rosbags/; pass one explicitly"
        echo "using newest bag: $bag"
    fi
    exec ros2 bag info "$bag"
    ;;

topics)
    with_ros
    exec ros2 topic list
    ;;

check)
    with_ros
    echo "== tests =="
    "$SYSTEM_PYTHON" -m pytest "$REPO/tests" -q

    echo
    echo "== scene contract =="
    if [[ ! -f "$REPO/isaacsim/scene.usd" ]]; then
        echo "SKIP: isaacsim/scene.usd not present, cannot check the contract"
    else
        before="$(cat "$REPO/docs/scene_contract.yaml" 2>/dev/null || true)"
        "$SYSTEM_PYTHON" "$REPO/scripts/dump_scene_contract.py" >/dev/null 2>&1
        if [[ "$before" != "$(cat "$REPO/docs/scene_contract.yaml")" ]]; then
            echo "STALE: docs/scene_contract.yaml did not match the scene and has"
            echo "       been regenerated. Review 'git diff docs/' and commit it."
            exit 1
        fi
        echo "OK: docs/scene_contract.yaml matches isaacsim/scene.usd"
    fi
    ;;

rviz)
    with_ros
    exec rviz2 -d "$REPO/rviz/dipl.rviz"
    ;;

""|-h|--help|help)
    usage
    ;;

*)
    usage
    die "unknown step: $step"
    ;;
esac
