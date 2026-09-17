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

Inspection and verification -- none of these need Isaac Sim:

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
    # bag_recorder_node hardcodes its output to the relative path 'maj_beg',
    # and rosbag2 refuses to open a bag directory that already exists. Running
    # from rosbags/ keeps that output with the other bags instead of wherever
    # the shell happened to be, and the check below turns "silent crash on
    # startup" into a sentence that says what to do.
    mkdir -p "$REPO/rosbags"
    cd "$REPO/rosbags"
    if [[ -e maj_beg ]]; then
        die "rosbags/maj_beg already exists; rosbag2 will not reopen it.
       Rename the previous run first, e.g.
           mv rosbags/maj_beg rosbags/rosbag_\$(date +%Y%m%d_%H%M%S)"
    fi
    echo "recorder: writing to rosbags/maj_beg; waiting for /record_rosbag"
    exec ros2 run bag_recorder_node bag_recorder_node
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

contract)
    exec "$SYSTEM_PYTHON" "$REPO/scripts/dump_scene_contract.py" "$@"
    ;;

test)
    with_ros
    exec "$SYSTEM_PYTHON" -m pytest "$REPO/tests" "$@"
    ;;

info)
    with_ros
    exec ros2 bag info "${1:-$REPO/rosbags/rosbag_0}"
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
