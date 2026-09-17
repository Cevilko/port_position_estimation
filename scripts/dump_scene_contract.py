#!/usr/bin/env python3
"""Dump a readable summary of the USD scene to docs/scene_contract.yaml.

``isaacsim/scene.usd`` is binary USDC and gitignored, so neither a human
reading a diff nor an agent reading the repo can see what the scene actually
publishes. This script extracts the parts that form a contract with the rest
of the pipeline -- camera prims and intrinsics, render-product resolutions,
every ROS 2 topic and service the graph names, the arm joints, and the port
entrance frames -- and writes them as tracked YAML.

**Re-run this after any change to the scene**, and commit the result::

    scripts/dump_scene_contract.py

Needs only ``pxr`` (OpenUSD), not a running Isaac Sim. It looks for a usable
interpreter in this order, and re-executes itself under the first one it finds:

1. the interpreter already running, if ``pxr`` imports
2. ``$USD_PYTHON``
3. ``~/usd_root/python-usd-venv/bin/python`` (NVIDIA's pre-built OpenUSD)

Isaac Sim's own ``python.sh`` does *not* work -- it only puts ``pxr`` on the
path once the Kit application has booted.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEFAULT_STAGE = REPO / "isaacsim" / "scene.usd"
DEFAULT_OUTPUT = REPO / "docs" / "scene_contract.yaml"

#: Interpreters to try, in order, when ``pxr`` is not importable here.
USD_PYTHON_CANDIDATES = (
    os.environ.get("USD_PYTHON"),
    str(Path.home() / "usd_root" / "python-usd-venv" / "bin" / "python"),
)


def reexec_under_usd_python() -> None:
    """Re-run this script under an interpreter that has ``pxr``.

    Exits the process either way: it either hands off, or explains what to
    install. Doing this here rather than making the caller pick the
    interpreter is what lets ``run.sh`` and a bare ``./dump_scene_contract.py``
    both just work.
    """

    for candidate in USD_PYTHON_CANDIDATES:
        if not candidate or not Path(candidate).is_file():
            continue
        probe = subprocess.run(
            [candidate, "-c", "import pxr"], capture_output=True, text=True
        )
        if probe.returncode == 0:
            os.execv(candidate, [candidate, str(Path(__file__).resolve()), *sys.argv[1:]])

    sys.exit(
        "No interpreter with OpenUSD (pxr) found.\n"
        "Tried: " + ", ".join(c for c in USD_PYTHON_CANDIDATES if c) + "\n"
        "Set USD_PYTHON=/path/to/python, or `pip install usd-core` into a venv.\n"
        "Note: Isaac Sim's python.sh does NOT work here -- pxr only becomes\n"
        "importable after the Kit app boots."
    )


try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ModuleNotFoundError:
    reexec_under_usd_python()


# Graph inputs worth recording. Anything else is layout noise that would make
# the dump churn on every unrelated edit.
INTERESTING_INPUTS = (
    "inputs:topicName",
    "inputs:nodeNamespace",
    "inputs:frameId",
    "inputs:serviceName",
    "inputs:messageName",
    "inputs:messagePackage",
    "inputs:width",
    "inputs:height",
    "inputs:parentPrim",
    "inputs:qosProfile",
)

INTERESTING_RELS = ("inputs:cameraPrim", "inputs:targetPrims", "inputs:renderProductPath")
"""Names that carry prim targets. ``renderProductPath`` is authored as a plain
attribute on the camera helpers and as a relationship elsewhere, so each one is
resolved by its actual property type rather than assumed."""

ARM_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def quote(value) -> str:
    """YAML-safe scalar. Strings are quoted so ``/topic`` is never a comment."""

    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"') + '"'


def camera_intrinsics(camera: UsdGeom.Camera, width: int, height: int) -> tuple[dict, str]:
    """Pinhole fx/fy/cx/cy from USD lens attributes at a render resolution.

    USD keeps focal length and aperture in tenths of a scene unit ("mm" by
    convention); the ratio is what matters, so the units cancel.

    ``fy`` is deliberately set equal to ``fx`` rather than derived from the
    vertical aperture: that is what the bridge actually publishes, verified
    against a real recording (``rosbag_samples/rosbag_0/camera_info/`` has
    ``fx == fy == 997.6612517868593``, matching horizontal aperture alone).

    Returns the intrinsics plus a warning string, empty when the vertical
    aperture agrees with the render aspect ratio. A disagreement is worth
    surfacing -- it means the lens as authored does not describe the image as
    rendered, so anything undistorting with these numbers should use fx.
    """

    focal = camera.GetFocalLengthAttr().Get()
    h_aperture = camera.GetHorizontalApertureAttr().Get()
    v_aperture = camera.GetVerticalApertureAttr().Get()
    if not focal or not h_aperture:
        return {}, ""

    fx = width * focal / h_aperture
    intrinsics = {"fx": round(fx, 4), "fy": round(fx, 4), "cx": width / 2.0, "cy": height / 2.0}

    warning = ""
    if v_aperture:
        aperture_aspect = h_aperture / v_aperture
        render_aspect = width / height
        if abs(aperture_aspect - render_aspect) > 1e-3:
            warning = (
                f"vertical_aperture implies aspect {aperture_aspect:.4f} but the "
                f"render product is {render_aspect:.4f}; fy is reported as fx, "
                f"which is what the bridge publishes"
            )
    return intrinsics, warning


def world_translation(prim: Usd.Prim) -> tuple | None:
    if not prim or not prim.IsValid() or not UsdGeom.Xformable(prim):
        return None
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = matrix.ExtractTranslation()
    return (round(t[0], 6), round(t[1], 6), round(t[2], 6))


def open_stage_capturing_diagnostics(stage_path: Path):
    """Open the stage and report which asset references failed to resolve.

    Walking ``GetPrimStack()`` finds nothing useful here: the dead references
    are authored inside *referenced* layers (the vendor's ``sc_port_visual.usd``
    points at absolute paths under ``~/IsaacLab`` that no longer exist), not on
    prims in this stage's root layer. USD reports those as diagnostics on the
    C++ side, so capture them at the file-descriptor level -- ``contextlib``'s
    redirect only moves Python's ``sys.stderr`` and would miss all of it.
    """

    import re
    import tempfile

    stderr_fd = sys.stderr.fileno()
    saved_fd = os.dup(stderr_fd)
    with tempfile.TemporaryFile(mode="w+") as sink:
        try:
            os.dup2(sink.fileno(), stderr_fd)
            stage = Usd.Stage.Open(str(stage_path))
        finally:
            os.dup2(saved_fd, stderr_fd)
            os.close(saved_fd)
        sink.seek(0)
        diagnostics = sink.read()

    unresolved = sorted(set(re.findall(r"Could not open asset @([^@]+)@", diagnostics)))
    return stage, unresolved


def collect(stage: Usd.Stage, unresolved: list[str]) -> list[str]:
    prims = list(stage.Traverse())
    lines: list[str] = []

    def section(title: str) -> None:
        lines.append("")
        lines.append(f"# {'-' * 70}")
        lines.append(f"{title}:")

    lines.append("# Generated by scripts/dump_scene_contract.py -- DO NOT EDIT BY HAND.")
    lines.append("# Regenerate and commit after any change to isaacsim/scene.usd.")
    lines.append("")
    lines.append(f"source_stage: {quote('isaacsim/scene.usd')}")
    lines.append(f"total_prims: {len(prims)}")

    # --- render products, keyed by camera, so resolution and intrinsics meet ---
    products = {}
    for prim in prims:
        if prim.GetAttribute("node:type").Get() != "isaacsim.core.nodes.IsaacCreateRenderProduct":
            continue
        prop = prim.GetProperty("inputs:cameraPrim")
        rel = prop.GetTargets() if isinstance(prop, Usd.Relationship) else []
        if not rel:
            continue
        products[str(rel[0])] = {
            "node": str(prim.GetPath()),
            "width": prim.GetAttribute("inputs:width").Get(),
            "height": prim.GetAttribute("inputs:height").Get(),
        }

    section("cameras")
    for prim in prims:
        if not prim.IsA(UsdGeom.Camera):
            continue
        path = str(prim.GetPath())
        camera = UsdGeom.Camera(prim)
        product = products.get(path, {})
        width, height = product.get("width"), product.get("height")
        lines.append(f"  - prim: {quote(path)}")
        # The TF frame is the camera's PARENT xform, not the camera itself --
        # that is what ROS2PublishTransformTree is pointed at.
        lines.append(f"    tf_frame: {quote(prim.GetParent().GetName())}")
        lines.append(f"    focal_length: {camera.GetFocalLengthAttr().Get()}")
        lines.append(f"    horizontal_aperture: {camera.GetHorizontalApertureAttr().Get()}")
        lines.append(f"    vertical_aperture: {camera.GetVerticalApertureAttr().Get()}")
        clip = camera.GetClippingRangeAttr().Get()
        if clip:
            lines.append(f"    clipping_range: [{clip[0]}, {clip[1]}]")
        if width and height:
            lines.append(f"    resolution: [{width}, {height}]")
            intrinsics, warning = camera_intrinsics(camera, width, height)
            if intrinsics:
                pairs = ", ".join(f"{k}: {v}" for k, v in intrinsics.items())
                lines.append(f"    intrinsics: {{{pairs}}}")
            if warning:
                lines.append(f"    intrinsics_note: {quote(warning)}")
        position = world_translation(prim)
        if position:
            lines.append(f"    world_position: [{', '.join(str(v) for v in position)}]")

    # --- the ROS 2 graph: every topic and service the scene names ------------
    section("ros2_graph")
    graphs: dict[str, list[Usd.Prim]] = {}
    for prim in prims:
        if prim.GetTypeName() == "OmniGraphNode":
            graphs.setdefault(str(prim.GetPath().GetParentPath()), []).append(prim)

    for graph_path in sorted(graphs):
        lines.append(f"  - graph: {quote(graph_path)}")
        lines.append("    nodes:")
        for prim in sorted(graphs[graph_path], key=lambda p: p.GetName()):
            lines.append(f"      - name: {quote(prim.GetName())}")
            lines.append(f"        type: {quote(prim.GetAttribute('node:type').Get())}")
            for name in INTERESTING_INPUTS:
                value = prim.GetAttribute(name).Get()
                if value in (None, "", []):
                    continue
                lines.append(f"        {name.split(':', 1)[1]}: {quote(value)}")
            for name in INTERESTING_RELS:
                prop = prim.GetProperty(name)
                key = name.split(":", 1)[1]
                if isinstance(prop, Usd.Relationship):
                    targets = prop.GetTargets()
                    if not targets:
                        continue
                    lines.append(f"        {key}:")
                    lines += [f"          - {quote(t)}" for t in targets]
                elif isinstance(prop, Usd.Attribute):
                    value = prop.Get()
                    if value not in (None, "", []):
                        lines.append(f"        {key}: {quote(value)}")

    # --- resolved topic names, the thing consumers actually bind to ----------
    section("published_topics")
    lines.append(
        "  # nodeNamespace + topicName as the bridge composes them. This is the"
    )
    lines.append("  # list `ros2 topic list` should show once you press Play.")
    for prim in prims:
        node_type = prim.GetAttribute("node:type").Get() or ""
        namespace = prim.GetAttribute("inputs:nodeNamespace").Get() or ""
        if "ROS2CameraHelper" in node_type:
            topic = prim.GetAttribute("inputs:topicName").Get() or "rgb"
            lines.append(f"  - {quote('/' + '/'.join(filter(None, [namespace, topic])))}")
        elif "ROS2CameraInfoHelper" in node_type:
            lines.append(
                f"  - {quote('/' + '/'.join(filter(None, [namespace, 'camera_info'])))}"
            )
        elif "PublishTransformTree" in node_type:
            lines.append(f"  - {quote('/tf')}")

    # --- robot ---------------------------------------------------------------
    section("robot")
    roots = [p for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    for root in roots:
        lines.append(f"  - articulation_root: {quote(root.GetPath())}")
    joints = {p.GetName(): str(p.GetPath()) for p in prims if p.IsA(UsdPhysics.RevoluteJoint)}
    lines.append("    arm_joints:")
    for name in ARM_JOINT_NAMES:
        lines.append(f"      - name: {quote(name)}")
        lines.append(f"        prim: {quote(joints.get(name, 'MISSING'))}")

    # --- targets -------------------------------------------------------------
    section("port_entrances")
    lines.append("  # The frames randomize_visible_joints.py keeps inside the camera frustum.")
    for prim in prims:
        if not prim.GetName().startswith("sfp_port_") or "entrance" not in prim.GetName():
            continue
        lines.append(f"  - name: {quote(prim.GetName())}")
        lines.append(f"    prim: {quote(prim.GetPath())}")
        position = world_translation(prim)
        if position:
            lines.append(f"    world_position: [{', '.join(str(v) for v in position)}]")

    # --- unresolved references ----------------------------------------------
    # The stage references assets by absolute path into ~/IsaacLab. Some are
    # already dead. An agent that cannot open the stage should still learn this.
    section("unresolved_references")
    lines.append("  # Absolute-path asset references the stage could not open.")
    lines.append("  # Non-empty here means the scene is not portable between machines.")
    for path in unresolved:
        lines.append(f"  - {quote(path)}")
    if not unresolved:
        lines.append("  []")

    return lines


def main() -> int:
    stage_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_STAGE
    output_path = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_OUTPUT

    if not stage_path.is_file():
        sys.exit(f"stage not found: {stage_path}")

    stage, unresolved = open_stage_capturing_diagnostics(stage_path)
    if stage is None:
        sys.exit(f"could not open stage: {stage_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(collect(stage, unresolved)) + "\n", encoding="utf-8")
    print(f"wrote {output_path.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
