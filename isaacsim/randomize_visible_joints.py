#!/usr/bin/env python3
"""Randomize the AIC robot arm while keeping both SFP entrances in view.

Run this with Isaac Sim's Python, not the system Python, for example:

    ~/isaacsim-5.1.0/python.sh isaacsim/randomize_visible_joints.py --samples 20

The script opens ``isaacsim/scene.usd``, finds the ``aic_unified_robot`` arm,
``center_camera``, and the two SFP entrance Xforms, then rejection-samples joint
positions until both entrance frame origins intersect the camera frustum. Each
candidate pose is given ``--settle-timeout`` seconds to settle before the FOV
check; once a pose is accepted, the pose is held for ``--record-hold-seconds``
and then the scene's ROS2 Service Client node is fired to call its
std_srvs/Trigger service (``/record_rosbag``).

The NIC card fixture is posed randomly too, so the ports do not sit at the same
world position in every capture. The card, its mount, the SC port and the base
move together as one rigid group (they are bolted together in reality), and the
only rotation applied is yaw about world +z, which is the one that keeps the
fixture standing on the table. Pass ``--no-randomize-ports`` to leave it at the
pose the scene authored.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


ARM_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

ARM_DEFAULT_POSITIONS = (
    0.1597,
    -1.3542,
    -1.6648,
    -1.6933,
    1.5710,
    -1.7306,
)

PORT_NAMES = (
    "sfp_port_0_entrance",
    "sfp_port_1_entrance",
)

#: The NIC card, its mount, the SC port and the base are one rigid fixture in
#: reality, so they are posed as a group: the same yaw and the same offset are
#: applied to every one of them. Splitting them would pull the card off its
#: mount. None of them has physics in its subtree -- they are pure visuals --
#: so moving them is a transform change and nothing else.
PORT_GROUP_PRIMS = (
    "/base_visual",
    "/nic_card_mount_visual",
    "/nic_card_visual",
    "/sc_port_visual",
)

#: The fixture stands upright on the table, so the only orientation change that
#: keeps it standing is yaw about world +z.
PORT_YAW_AXIS = "z"

#: The aperture the labeller projects, in metres, used here to sample the port
#: at its corners as well as its centre rather than trusting a single ray.
PORT_APERTURE_WIDTH = 0.0122
PORT_APERTURE_HEIGHT = 0.00715

# Headless rendering runs slower than real time, so a sim-time wait is allowed
# this multiple of wall-clock seconds before it is treated as stalled.
WAIT_WALL_CLOCK_LIMIT = 20.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stage",
        default=str(Path(__file__).resolve().parent / "scene.usd"),
        help="USD stage to open; defaults to isaacsim/scene.usd",
    )
    parser.add_argument("--headless", action="store_true", help="run without a GUI")
    parser.add_argument("--seed", type=int, help="seed for reproducible samples")
    parser.add_argument(
        "--samples",
        type=int,
        default=1,
        help="number of accepted random joint poses to produce",
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=500,
        help="maximum rejection-sampling attempts per accepted pose",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=0.5,
        help="simulated seconds to let the robot settle after teleporting a candidate, before checking FOV",
    )
    parser.add_argument(
        "--record-hold-seconds",
        type=float,
        default=2.0,
        help="simulated seconds to keep an accepted, recorded pose playing after the record service call",
    )
    parser.add_argument(
        "--port-pos-span",
        type=float,
        nargs=3,
        default=(0.06, 0.06, 0.0),
        metavar=("X", "Y", "Z"),
        help=(
            "half-width in metres of the random offset applied to the NIC card "
            "fixture; z defaults to 0 because the base sits on the table and "
            "lifting it looks unphysical"
        ),
    )
    parser.add_argument(
        "--port-yaw-span",
        type=float,
        default=math.pi,
        help=(
            "half-width in radians of the fixture's random yaw about world +z; "
            "the default is a full turn, which is safe because the ports face "
            "straight up and a z-rotation leaves that normal unchanged"
        ),
    )
    parser.add_argument(
        "--port-group-prims",
        nargs="+",
        default=PORT_GROUP_PRIMS,
        help="prims moved as one rigid fixture with the ports",
    )
    parser.add_argument(
        "--port-pivot-prim",
        default="/nic_card_visual",
        help="prim whose position the fixture's yaw turns about",
    )
    parser.add_argument(
        "--no-occlusion-check",
        action="store_true",
        help="accept poses without testing whether anything blocks the ports",
    )
    parser.add_argument(
        "--occlusion-tolerance",
        type=float,
        default=0.005,
        help="metres nearer than a port a surface must be to count as blocking it",
    )
    parser.add_argument(
        "--probe-width",
        type=int,
        default=288,
        help="width of the offscreen depth render used for the occlusion test",
    )
    parser.add_argument(
        "--max-view-angle",
        type=float,
        default=70.0,
        help=(
            "reject a pose unless every port's outward normal is within this "
            "many degrees of the direction to the camera; past 90 the camera is "
            "behind the opening and sees the back of the card"
        ),
    )
    parser.add_argument(
        "--keep-position-controller",
        action="store_true",
        help=(
            "leave the scene's ArticulationController running; it drives the arm "
            "to a fixed pose every tick, which overrides sampled joint positions"
        ),
    )
    parser.add_argument(
        "--no-randomize-ports",
        action="store_true",
        help="keep the NIC card fixture at its authored pose",
    )
    parser.add_argument(
        "--still-speed",
        type=float,
        default=1e-3,
        help="max joint speed in rad/s that still counts as a settled pose",
    )
    parser.add_argument(
        "--still-timeout",
        type=float,
        default=2.0,
        help="simulated seconds to wait for the arm to fall below --still-speed",
    )
    parser.add_argument(
        "--record-client-prim",
        default="ros2_service_client",
        help="prim path or name of the scene's ROS2 Service Client node fired on an accepted pose",
    )
    parser.add_argument(
        "--skip-record",
        action="store_true",
        help="skip firing the record service client (useful when no recorder node is running)",
    )
    parser.add_argument(
        "--joint-span",
        type=float,
        nargs="+",
        default=(0.8,),
        help=(
            "random half-width in radians around the nominal pose; give one value "
            "for all joints or six values in ARM_JOINT_NAMES order"
        ),
    )
    parser.add_argument(
        "--nominal",
        type=float,
        nargs=6,
        help=(
            "nominal joint pose in radians; defaults to the live scene pose "
            "after opening the stage"
        ),
    )
    parser.add_argument(
        "--nominal-source",
        choices=("current", "default"),
        default="current",
        help=(
            "where to take the nominal pose from when --nominal is omitted; "
            "'default' uses the UR5e task default"
        ),
    )
    parser.add_argument(
        "--joint-low",
        type=float,
        nargs=6,
        help="absolute lower joint sampling bounds in radians",
    )
    parser.add_argument(
        "--joint-high",
        type=float,
        nargs=6,
        help="absolute upper joint sampling bounds in radians",
    )
    parser.add_argument(
        "--robot-prim",
        default="aic_unified_robot",
        help="robot prim path or prim name to use",
    )
    parser.add_argument(
        "--camera-prim",
        default="center_camera",
        help="camera prim path or prim name to use",
    )
    parser.add_argument(
        "--port-prims",
        nargs=2,
        default=PORT_NAMES,
        help="two port entrance prim paths or prim names to keep in view",
    )
    parser.add_argument(
        "--save",
        help="optional path to save the stage after the last accepted pose",
    )
    parser.add_argument(
        "--leave-running",
        action="store_true",
        help="continue updating Isaac Sim after all accepted samples are produced",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples < 0:
        raise SystemExit("--samples must be 0 or more")
    # --samples 0 means "open the scene, play it, and change nothing": the ROS
    # publishers tick so a detector or RViz has live topics, but nothing is
    # randomized, no controller is disabled and no recording is triggered.
    just_run = args.samples == 0
    if args.max_attempts < 1:
        raise SystemExit("--max-attempts must be at least 1")
    if bool(args.joint_low) != bool(args.joint_high):
        raise SystemExit("--joint-low and --joint-high must be provided together")

    sys.stdout.reconfigure(line_buffering=True)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})

    import numpy as np
    import omni.graph.core as og
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation, XFormPrim
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    # The prepared scene contains ROS publishers, but enabling these extensions
    # here makes the script usable from a bare Isaac Sim launch too.
    enable_extension("omni.graph.scriptnode")
    enable_extension("isaacsim.ros2.bridge")
    _pump(app, 5)

    context = omni.usd.get_context()
    stage_path = str(Path(args.stage).expanduser().resolve())
    opened = context.open_stage(stage_path)
    if opened is False:
        raise RuntimeError(f"Failed to open stage: {stage_path}")
    _pump(app, 20)

    stage = context.get_stage()
    robot_prim = _resolve_robot_prim(stage, args.robot_prim, UsdPhysics)
    articulation_prim = _articulation_root_prim(robot_prim, UsdPhysics)
    camera_prim = _resolve_prim(stage, args.camera_prim, UsdGeom.Camera)
    port_prims = [_resolve_prim(stage, item, None) for item in args.port_prims]

    print(f"stage:  {stage_path}")
    print(f"robot:  {robot_prim.GetPath()}")
    print(f"artic:  {articulation_prim.GetPath()}")
    print(f"camera: {camera_prim.GetPath()}")
    for index, prim in enumerate(port_prims):
        print(f"port {index}: {prim.GetPath()}")

    record_gate = None
    if not args.skip_record:
        client_prim = _resolve_service_client_prim(stage, args.record_client_prim)
        service_name = client_prim.GetAttribute("inputs:serviceName").Get()
        print(f"record: {client_prim.GetPath()} -> {service_name}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _pump(app, 20)

    if not args.skip_record:
        graph_path = client_prim.GetPath().pathString.rsplit("/", 1)[0]
        clock_path = _ensure_clock_publisher(stage, og, graph_path)
        print(f"clock:  {clock_path or 'NOT PUBLISHED (recorder will use wall time)'}")
        record_gate = _setup_record_gate(stage, og, client_prim.GetPath().pathString)
        if not just_run:
            # The client node's first compute only initialises it, so spend that
            # no-op pulse here instead of losing the first accepted pose's
            # recording. Skipped when only running the scene -- that pulse can
            # produce a real capture, and here nobody asked for one.
            _pulse_record_gate(app, record_gate)
            _pump(app, 10)

    articulation = Articulation(articulation_prim.GetPath().pathString)
    articulation.initialize()
    dof_names = list(articulation.dof_names)
    joint_indices = np.array([dof_names.index(name) for name in ARM_JOINT_NAMES])

    if args.nominal is not None:
        nominal = np.asarray(args.nominal, dtype=float)
    elif args.nominal_source == "default":
        nominal = np.asarray(ARM_DEFAULT_POSITIONS, dtype=float)
    else:
        nominal = _current_arm_positions(articulation, joint_indices, np)
    # Only now: the nominal is read from the live pose, and until this point the
    # scene's controller is what holds the arm there. Disabling it earlier would
    # let the arm sag first and centre every sample on the sag instead.
    # Leave the scene's own controller alone when only running it: disabling it
    # is a sampling measure, and without it the arm just sags.
    if not args.keep_position_controller and not just_run:
        stopped = _disable_competing_controllers(stage, og)
        print(f"controller: disabled {stopped or 'none found'}")

    probe = None
    if not args.no_occlusion_check and not just_run:
        import omni.replicator.core as rep

        height = max(1, int(round(args.probe_width * 1024 / 1152)))
        probe = _VisibilityProbe(
            rep, np, Gf, camera_prim.GetPath().pathString, args.probe_width, height
        )
        _pump(app, 10)
        print(f"probe:  {args.probe_width}x{height} depth render on {camera_prim.GetPath()}")

    low, high = _sampling_bounds(args, nominal, np)
    rng = np.random.default_rng(args.seed)

    fixture = None
    if not args.no_randomize_ports and not just_run:
        fixture = _RigidFixture(
            XFormPrim, np, args.port_group_prims, args.port_pivot_prim
        )
        print(
            "fixture: "
            + ", ".join(args.port_group_prims)
            + f"\n   yaw about {args.port_pivot_prim} at "
            + _format_joint_vector(fixture.pivot)
            + f", pos span {tuple(args.port_pos_span)} m, yaw span "
            + f"+/-{args.port_yaw_span:.3f} rad"
        )

    if just_run:
        print("mode:    running the scene only, no randomization")
    else:
        print("nominal: " + _format_joint_vector(nominal))
        print("low:     " + _format_joint_vector(low))
        print("high:    " + _format_joint_vector(high))

    accepted = 0
    last_accepted = None
    rejections: dict[str, int] = {}
    while accepted < args.samples:
        for attempt in range(1, args.max_attempts + 1):
            candidate = rng.uniform(low, high)
            _apply_arm_positions(articulation, joint_indices, candidate, np)
            if fixture is not None:
                # Pose the fixture per attempt, not per accepted sample: the
                # frustum test below then vets the arm pose and the port pose
                # together, which is the combination that gets recorded.
                span = np.asarray(args.port_pos_span, dtype=float)
                fixture_offset = rng.uniform(-span, span)
                fixture_yaw = float(
                    rng.uniform(-args.port_yaw_span, args.port_yaw_span)
                )
                fixture.apply(fixture_yaw, fixture_offset)
            _wait_seconds(app, timeline, args.settle_timeout)

            rejection = _camera_contains_prims(
                camera_prim, port_prims, Usd, UsdGeom, Gf, args.max_view_angle,
                probe, args.occlusion_tolerance,
            )
            rejections[rejection] = rejections.get(rejection, 0) + 1
            if not rejection:
                accepted += 1
                last_accepted = candidate.copy()
                print(
                    f"accepted {accepted}/{args.samples} after {attempt} attempts: "
                    + _format_joint_vector(candidate)
                )
                if fixture is not None:
                    print(
                        f"  fixture: yaw {fixture_yaw:+.4f} rad, offset "
                        + _format_joint_vector(fixture_offset)
                    )
                # Capture last, not first. The cameras ride the arm, so a
                # trigger fired while it is still settling records an image
                # taken from a slightly different pose than the transforms
                # published alongside it, which shifts every projected box.
                _wait_seconds(app, timeline, args.record_hold_seconds)
                speed = _wait_until_still(
                    app, timeline, articulation, joint_indices, np,
                    args.still_speed, args.still_timeout,
                )
                if speed > args.still_speed:
                    print(
                        f"warning: arm still moving at {speed:.4f} rad/s when captured"
                    )
                if record_gate is not None:
                    _pulse_record_gate(app, record_gate)
                    print("record:  triggered")
                break
        else:
            raise RuntimeError(
                "No visible joint sample found. Try reducing --joint-span, "
                "setting --nominal near a known good pose, or increasing --max-attempts."
            )

    if not just_run:
        summary = ", ".join(
            f"{reason or 'accepted'}={count}" for reason, count in sorted(rejections.items())
        )
        print(f"attempts: {summary}")

    if args.save:
        save_path = str(Path(args.save).expanduser().resolve())
        _author_arm_joint_state(robot_prim, last_accepted, UsdPhysics)
        if context.save_as_stage(save_path):
            print(f"saved:   {save_path}")
        else:
            raise RuntimeError(f"Failed to save stage: {save_path}")

    # Running the scene only is pointless if the process then exits, so that
    # mode keeps the app alive whether or not --leave-running was passed.
    if args.leave_running or just_run:
        print("running; close the window or press Ctrl-C to stop")
        while app.is_running():
            app.update()

    app.close()
    return 0


def _pump(app, frames: int) -> None:
    for _ in range(frames):
        app.update()


def _wait_seconds(app, timeline, seconds: float) -> None:
    """Play for ``seconds`` of simulation time.

    Elapsed time is accumulated per frame rather than compared against a start
    value: playback restarts from the stage's start time when it reaches the
    end, which makes a single start/now comparison run backwards and never
    finish. A wall-clock deadline bounds the wait if the timeline stops
    advancing altogether.
    """
    if seconds <= 0:
        app.update()
        return

    elapsed = 0.0
    previous = timeline.get_current_time()
    deadline = time.monotonic() + WAIT_WALL_CLOCK_LIMIT * seconds + 10.0
    while elapsed < seconds:
        app.update()
        if not timeline.is_playing():
            timeline.play()
        current = timeline.get_current_time()
        delta = current - previous
        previous = current
        # A negative delta is the timeline wrapping back to its start.
        if delta > 0:
            elapsed += delta
        if time.monotonic() > deadline:
            print(
                f"warning: waited {elapsed:.2f}s of sim time for a {seconds:.2f}s "
                "wait before the wall-clock limit; continuing"
            )
            return


ARTICULATION_CONTROLLER_NODE_TYPE = "isaacsim.core.nodes.IsaacArticulationController"
CLOCK_NODE_TYPE = "isaacsim.ros2.bridge.ROS2PublishClock"
SIM_TIME_NODE_TYPE = "isaacsim.core.nodes.IsaacReadSimulationTime"
CONTEXT_NODE_TYPE = "isaacsim.ros2.bridge.ROS2Context"
SERVICE_CLIENT_NODE_TYPE = "isaacsim.ros2.bridge.OgnROS2ServiceClient"
PLAYBACK_TICK_NODE_TYPE = "omni.graph.action.OnPlaybackTick"
RECORD_GATE_NAME = "record_gate"


class _VisibilityProbe:
    """Depth-buffer occlusion test for points in the sampling camera's view.

    A physics raycast is no use here: the card carries no colliders and neither
    does the gripper, so a ray would pass through both. The rendered depth
    buffer sees whatever the camera sees, whatever it is made of.

    The test is one-sided on purpose. A port entrance is a hole, so an
    unobstructed sample reads the inside of the cage *behind* the entrance
    plane and comes back farther than the entrance. Only depth that is clearly
    nearer than the entrance means something is in the way.
    """

    def __init__(self, rep, np, Gf, camera_path: str, width: int, height: int):
        self._np = np
        self._Gf = Gf
        self._size = (width, height)
        self._render_product = rep.create.render_product(camera_path, (width, height))
        self._annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self._annotator.attach([self._render_product])
        self._warned = False

    def _depth(self):
        data = self._annotator.get_data()
        if data is None:
            return None
        array = self._np.asarray(data)
        return array if array.size and array.ndim == 2 else None

    def occluded_points(self, points_world, frustum, camera_position, tolerance):
        """How many of ``points_world`` are blocked, and how many were testable."""
        depth = self._depth()
        if depth is None:
            if not self._warned:
                print("warning: no depth from the visibility probe; occlusion unchecked")
                self._warned = True
            return 0, 0

        height, width = depth.shape
        view = frustum.ComputeViewMatrix()
        projection = frustum.ComputeProjectionMatrix()
        blocked = 0
        tested = 0
        for point in points_world:
            clip = self._Gf.Vec4d(point[0], point[1], point[2], 1.0) * view * projection
            if clip[3] <= 0.0:
                continue
            ndc_x, ndc_y = clip[0] / clip[3], clip[1] / clip[3]
            px = int((ndc_x * 0.5 + 0.5) * width)
            py = int((1.0 - (ndc_y * 0.5 + 0.5)) * height)
            if not (0 <= px < width and 0 <= py < height):
                continue
            tested += 1
            expected = float(self._np.linalg.norm(self._np.asarray(point) - camera_position))
            measured = float(depth[py, px])
            if measured > 0.0 and measured < expected - tolerance:
                blocked += 1
        return blocked, tested


def _quat_multiply_wxyz(a, b, np):
    """Hamilton product of two scalar-first quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


class _RigidFixture:
    """Poses a set of prims as one rigid body, yawing about a shared pivot.

    The authored pose is captured once and every randomisation is expressed
    relative to it, so errors cannot accumulate over a long run and the fixture
    can always be returned to where the scene put it.
    """

    def __init__(self, XFormPrim, np, paths, pivot_path):
        self._np = np
        self._paths = list(paths)
        self._view = XFormPrim(self._paths)
        positions, orientations = self._view.get_world_poses()
        self._base_positions = np.asarray(positions, dtype=float).copy()
        self._base_orientations = np.asarray(orientations, dtype=float).copy()
        if pivot_path in self._paths:
            self._pivot = self._base_positions[self._paths.index(pivot_path)].copy()
        else:
            self._pivot = self._base_positions.mean(axis=0)

    @property
    def pivot(self):
        return self._pivot

    def apply(self, yaw: float, offset) -> None:
        np = self._np
        half = yaw / 2.0
        yaw_quat = np.array([np.cos(half), 0.0, 0.0, np.sin(half)])

        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        relative = self._base_positions - self._pivot
        rotated = np.column_stack(
            [
                cos_yaw * relative[:, 0] - sin_yaw * relative[:, 1],
                sin_yaw * relative[:, 0] + cos_yaw * relative[:, 1],
                relative[:, 2],
            ]
        )
        positions = self._pivot + rotated + np.asarray(offset, dtype=float)
        orientations = np.stack(
            [
                _quat_multiply_wxyz(yaw_quat, base, np)
                for base in self._base_orientations
            ]
        )
        self._view.set_world_poses(positions, orientations)


def _wait_until_still(app, timeline, articulation, joint_indices, np,
                      threshold: float, timeout: float) -> float:
    """Pump frames until the arm's fastest joint drops below ``threshold``.

    Returns the last speed seen, so the caller can report a pose that never
    settled rather than silently recording a smeared one.
    """
    start = timeline.get_current_time()
    speed = float("inf")
    while True:
        velocities = np.asarray(articulation.get_joint_velocities())
        if velocities.ndim == 2:
            velocities = velocities[0]
        speed = float(np.max(np.abs(velocities[joint_indices])))
        if speed <= threshold:
            return speed
        elapsed = timeline.get_current_time() - start
        if elapsed < 0 or elapsed >= timeout:
            return speed
        app.update()


def _disable_competing_controllers(stage, og):
    """Stop ArticulationController nodes from overriding the sampled pose.

    The scene drives the arm to one fixed joint command on every tick. Setting
    joint positions without stopping that means the controller pulls the arm
    straight back: barely visible if you capture immediately, total once you
    capture after a hold. Symptoms are joints that never settle and a camera
    that hardly moves however wide the sampling span is.
    """
    disabled = []
    for prim in stage.Traverse():
        if _node_type(prim) != ARTICULATION_CONTROLLER_NODE_TYPE:
            continue
        path = prim.GetPath().pathString
        node = og.Controller.node(path)
        try:
            node.set_disabled(True)
            disabled.append(path)
            continue
        except AttributeError:
            pass
        source = prim.GetAttribute("inputs:execIn").GetConnections()
        if source:
            og.Controller.edit(
                path.rsplit("/", 1)[0],
                {
                    og.Controller.Keys.DISCONNECT: [
                        (str(source[0]), f"{path}.inputs:execIn")
                    ]
                },
            )
            disabled.append(path)
    return disabled


def _resolve_service_client_prim(stage, query: str):
    if query.startswith("/"):
        prim = stage.GetPrimAtPath(query)
        if prim.IsValid():
            return prim

    matches = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == query and _node_type(prim) == SERVICE_CLIENT_NODE_TYPE
    ]
    if not matches:
        raise RuntimeError(
            f"Could not find a {SERVICE_CLIENT_NODE_TYPE} node named '{query}'. "
            "Pass --record-client-prim, or --skip-record to run without recording."
        )
    if len(matches) > 1:
        chosen = _choose_shortest_path(matches)
        print(f"warning: found {len(matches)} service clients, using {chosen.GetPath()}")
        return chosen
    return matches[0]


def _node_type(prim):
    attribute = prim.GetAttribute("node:type")
    return attribute.Get() if attribute else None


def _setup_record_gate(stage, og, client_path: str):
    """Wire ``tick -> Branch -> service client`` and return the Branch condition.

    The service client only sends when its ``execIn`` is driven by a real graph
    connection, so recording is gated behind a Branch whose condition is opened
    for exactly one frame per accepted pose.
    """
    graph_path = client_path.rsplit("/", 1)[0]
    gate_path = f"{graph_path}/{RECORD_GATE_NAME}"
    tick_path = _find_node_of_type(stage, graph_path, PLAYBACK_TICK_NODE_TYPE)
    if tick_path is None:
        raise RuntimeError(f"No {PLAYBACK_TICK_NODE_TYPE} node found in {graph_path}")

    if not stage.GetPrimAtPath(gate_path).IsValid():
        og.Controller.edit(
            graph_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    (RECORD_GATE_NAME, "omni.graph.action.Branch")
                ]
            },
        )

    connections = [
        (f"{tick_path}.outputs:tick", f"{gate_path}.inputs:execIn"),
        (f"{gate_path}.outputs:execTrue", f"{client_path}.inputs:execIn"),
    ]
    missing = [pair for pair in connections if not _is_connected(stage, *pair)]
    og.Controller.edit(
        graph_path,
        {
            og.Controller.Keys.SET_VALUES: [(f"{gate_path}.inputs:condition", False)],
            og.Controller.Keys.CONNECT: missing,
        },
    )
    return og.Controller.attribute("inputs:condition", og.Controller.node(gate_path))


def _ensure_clock_publisher(stage, og, graph_path: str) -> str | None:
    """Publish /clock from the scene's simulation time, adding the node if absent.

    The recorder resolves transforms at an image's stamp, which needs its clock
    to agree with the sim time those stamps are in. Without /clock the recorder
    runs on wall-clock time and every lookup falls outside the TF buffer.
    """
    existing = _find_node_of_type(stage, graph_path, CLOCK_NODE_TYPE)
    if existing is not None:
        return existing

    tick_path = _find_node_of_type(stage, graph_path, PLAYBACK_TICK_NODE_TYPE)
    sim_time_path = _find_node_of_type(stage, graph_path, SIM_TIME_NODE_TYPE)
    context_path = _find_node_of_type(stage, graph_path, CONTEXT_NODE_TYPE)
    if tick_path is None or sim_time_path is None:
        return None

    clock_path = f"{graph_path}/ros2_publish_clock"
    og.Controller.edit(
        graph_path,
        {
            og.Controller.Keys.CREATE_NODES: [("ros2_publish_clock", CLOCK_NODE_TYPE)],
            og.Controller.Keys.SET_VALUES: [
                (f"{clock_path}.inputs:topicName", "clock")
            ],
            og.Controller.Keys.CONNECT: [
                (f"{tick_path}.outputs:tick", f"{clock_path}.inputs:execIn"),
                (
                    f"{sim_time_path}.outputs:simulationTime",
                    f"{clock_path}.inputs:timeStamp",
                ),
            ]
            + (
                [(f"{context_path}.outputs:context", f"{clock_path}.inputs:context")]
                if context_path is not None
                else []
            ),
        },
    )
    return clock_path


def _find_node_of_type(stage, graph_path: str, node_type: str):
    prefix = graph_path + "/"
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.startswith(prefix) and _node_type(prim) == node_type:
            return path
    return None


def _is_connected(stage, source: str, target: str) -> bool:
    prim_path, attribute_name = target.rsplit(".", 1)
    attribute = stage.GetPrimAtPath(prim_path).GetAttribute(attribute_name)
    if not attribute:
        return False
    return any(str(item) == source for item in attribute.GetConnections())


def _pulse_record_gate(app, condition_attribute) -> None:
    """Open the record gate for exactly one frame, sending a single request."""
    condition_attribute.set(True)
    app.update()
    condition_attribute.set(False)


def _resolve_robot_prim(stage, query: str, UsdPhysics):
    prim = _resolve_prim(stage, query, None)
    if _contains_arm_joints(prim) or prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return prim

    matches = [
        child
        for child in stage.Traverse()
        if child.GetName() == query and _contains_arm_joints(child)
    ]
    if matches:
        return _choose_shortest_path(matches)

    raise RuntimeError(
        f"Resolved '{query}' to {prim.GetPath()}, but it does not contain the "
        f"expected arm joints: {', '.join(ARM_JOINT_NAMES)}"
    )


def _resolve_prim(stage, query: str, schema):
    if query.startswith("/"):
        prim = stage.GetPrimAtPath(query)
        if prim.IsValid() and (schema is None or prim.IsA(schema)):
            return prim

    matches = [
        item
        for item in stage.Traverse()
        if item.GetName() == query and (schema is None or item.IsA(schema))
    ]
    if not matches:
        kind = "prim" if schema is None else getattr(schema, "__name__", str(schema))
        raise RuntimeError(f"Could not find {kind} by path or name: {query}")
    if len(matches) > 1:
        chosen = _choose_shortest_path(matches)
        print(
            f"warning: found {len(matches)} prims named '{query}', using {chosen.GetPath()}"
        )
        return chosen
    return matches[0]


def _choose_shortest_path(prims):
    return sorted(prims, key=lambda item: len(item.GetPath().pathString))[0]


def _contains_arm_joints(root_prim) -> bool:
    names = {
        prim.GetName()
        for prim in root_prim.GetStage().Traverse()
        if prim.GetPath().HasPrefix(root_prim.GetPath())
    }
    return all(name in names for name in ARM_JOINT_NAMES)


def _articulation_root_prim(robot_prim, UsdPhysics):
    if robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return robot_prim
    matches = [
        prim
        for prim in robot_prim.GetStage().Traverse()
        if prim.GetPath().HasPrefix(robot_prim.GetPath())
        and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if not matches:
        raise RuntimeError(f"No ArticulationRootAPI prim under {robot_prim.GetPath()}")
    return _choose_shortest_path(matches)


def _current_arm_positions(articulation, joint_indices, np):
    positions = np.asarray(articulation.get_joint_positions())
    if positions.ndim == 2:
        positions = positions[0]
    return positions[joint_indices].astype(float)


def _sampling_bounds(args, nominal, np):
    if args.joint_low is not None:
        low = np.asarray(args.joint_low, dtype=float)
        high = np.asarray(args.joint_high, dtype=float)
    else:
        span = _expand_joint_arg(args.joint_span, "--joint-span", np)
        low = nominal - span
        high = nominal + span

    if np.any(high <= low):
        raise SystemExit("each upper joint bound must be greater than the lower bound")
    return low, high


def _expand_joint_arg(values, label: str, np):
    if len(values) == 1:
        return np.repeat(float(values[0]), len(ARM_JOINT_NAMES))
    if len(values) == len(ARM_JOINT_NAMES):
        return np.asarray(values, dtype=float)
    raise SystemExit(f"{label} expects one value or {len(ARM_JOINT_NAMES)} values")


def _apply_arm_positions(articulation, joint_indices, positions, np) -> None:
    shaped_positions = np.asarray(positions, dtype=float).reshape(1, -1)
    zeros = np.zeros_like(shaped_positions)
    articulation.set_joint_positions(shaped_positions, joint_indices=joint_indices)
    articulation.set_joint_velocities(zeros, joint_indices=joint_indices)
    articulation.set_joint_position_targets(shaped_positions, joint_indices=joint_indices)


def _author_arm_joint_state(robot_prim, positions, UsdPhysics) -> None:
    values = dict(zip(ARM_JOINT_NAMES, positions))
    for prim in robot_prim.GetStage().Traverse():
        if not prim.GetPath().HasPrefix(robot_prim.GetPath()):
            continue
        radians = values.get(prim.GetName())
        if radians is None or not prim.IsA(UsdPhysics.Joint):
            continue
        degrees = math.degrees(float(radians))
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            drive.CreateTargetPositionAttr().Set(degrees)
        for attribute, value in (
            ("state:angular:physics:position", degrees),
            ("state:angular:physics:velocity", 0.0),
        ):
            joint_state = prim.GetAttribute(attribute)
            if joint_state:
                joint_state.Set(value)


def _port_sample_points(transform, Gf):
    """Aperture centre and corners in world space, from the entrance frame."""
    half_w = PORT_APERTURE_WIDTH / 2.0
    half_h = PORT_APERTURE_HEIGHT / 2.0
    local = [
        (0.0, 0.0, 0.0),
        (-half_w, 0.0, -half_h),
        (half_w, 0.0, -half_h),
        (half_w, 0.0, half_h),
        (-half_w, 0.0, half_h),
    ]
    return [transform.Transform(Gf.Vec3d(*point)) for point in local]


def _camera_contains_prims(
    camera_prim, target_prims, Usd, UsdGeom, Gf, max_view_angle: float = 70.0,
    probe=None, occlusion_tolerance: float = 0.005,
) -> str:
    """True when every port is in frustum AND facing the camera.

    In-frustum alone is not visibility. A port entrance is an opening in one
    face of the cage, so once the camera passes the plane of that face it sees
    the back of the card while the port's origin is still happily inside the
    frustum. Labelling those produces boxes over a surface with no port in it.
    The entrance frame's local +y is the outward normal, so the angle between
    it and the direction to the camera is the test.
    """
    time = Usd.TimeCode.Default()
    camera = UsdGeom.Camera(camera_prim)
    gf_camera = camera.GetCamera(time)
    camera_to_world = camera.ComputeLocalToWorldTransform(time)
    gf_camera.transform = camera_to_world
    frustum = gf_camera.frustum
    camera_position = camera_to_world.ExtractTranslation()
    cosine_limit = math.cos(math.radians(max_view_angle))

    for prim in target_prims:
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time)
        position = transform.ExtractTranslation()
        if not frustum.Intersects(Gf.Vec3d(position)):
            return "out of frustum"

        normal = Gf.Vec3d(transform.ExtractRotationMatrix()[1]).GetNormalized()
        to_camera = Gf.Vec3d(camera_position - position).GetNormalized()
        if Gf.Dot(normal, to_camera) < cosine_limit:
            return "facing away"

        if probe is not None:
            points = _port_sample_points(transform, Gf)
            blocked, tested = probe.occluded_points(
                points, frustum, camera_position, occlusion_tolerance
            )
            # Any blocked sample rejects the pose: the test only fires when a
            # surface is nearer than the entrance, which an unobstructed port
            # never produces, so a single hit is already strong evidence.
            if blocked or (tested and tested < len(points)):
                return "occluded"
    return ""


def _world_position(prim, time, UsdGeom):
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time).ExtractTranslation()


def _format_joint_vector(values) -> str:
    return "[" + ", ".join(f"{float(value):+.4f}" for value in values) + "]"


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
