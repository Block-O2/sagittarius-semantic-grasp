#!/usr/bin/env python3
"""Read-only smoke check of pure helpers; no ROS, model, device, or motion I/O."""

import ast
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector"
NODES = PACKAGE / "nodes"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    python_files = list((NODES / "perception_framework").rglob("*.py"))
    python_files += [NODES / name for name in (
        "language_guided_grasp.py", "language_guided_calibration.py",
        "manual_vision_calibration.py", "perception_backend_smoke_test.py",
        "publish_test_image.py",
    )]
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    xml_files = list((PACKAGE / "launch").glob("*.launch")) + [PACKAGE / "package.xml"]
    for path in xml_files:
        ET.parse(path)
    print("PASS: {} Python syntax files, {} XML files".format(len(python_files), len(xml_files)))

    # Deliberately import only helpers with no model or robot initialization.
    sys.path.insert(0, str(NODES))
    from perception_framework.task_parsing import parse_task_command
    from perception_framework.detection_types import DetectionBox, DetectionResult
    from perception_framework.decision import evaluate_target_selection
    from perception_framework.stability import CenterStabilityFilter

    single = parse_task_command("red block")
    require(len(single.steps) == 1 and single.pick_target_text == "red block", "single target parsing")
    pair = parse_task_command("pick blue block and place it into red bucket")
    require(pair.pick_target_text == "blue block" and pair.place_target_text == "red bucket", "pick/place parsing")
    sort = parse_task_command("put each block into the bucket of the same color")
    require([(s.pick_target_text, s.place_target_text) for s in sort.steps] == [
        ("red block", "red bucket"), ("blue block", "blue bucket"),
        ("green block", "green bucket"),
    ], "fixed same-color expansion")
    multi = parse_task_command("red block left of blue block, green block right of blue block")
    require(len(multi.steps) == 2 and [s.place_relation for s in multi.steps] == ["left_of", "right_of"], "relative multi-step parsing")
    print("PASS: single, pick/place, fixed same-color, and relative multi-step commands")

    low = DetectionBox((0, 0, 20, 20), 0.2, "red block")
    high = DetectionBox((30, 30, 50, 50), 0.8, "red block")
    def result(boxes):
        return DetectionResult("synthetic", 0.0, (640, 480), boxes=boxes)
    for target, data, expected in (
        ("", result([high]), "invalid_target"),
        ("red block", None, "no_detection"),
        ("red block", result([]), "no_detection"),
        ("red block", result([low]), "low_confidence"),
    ):
        decision = evaluate_target_selection(data, target, min_score=0.35)
        require(decision.status == expected and not decision.should_execute, expected)
    decision = evaluate_target_selection(result([low, high]), "red block", 0.35)
    require(decision.should_execute and decision.selected_box is high, "highest-confidence selection")
    require(high.center == (40.0, 40.0), "box-center fallback")
    high.metadata["refined_center"] = (39.0, 41.0)
    require(high.center == (39.0, 41.0), "refined-center precedence")
    print("PASS: invalid / missing / low-confidence rejection, selection, center fallback")

    stable = CenterStabilityFilter(3, 8.0)
    for point in ((10, 10), (11, 10)):
        stable.add(point)
    require(not stable.is_stable(), "premature lock")
    stable.add((12, 10))
    require(stable.is_stable() and stable.average_center() == (11.0, 10.0), "stable lock")
    stable.add((40, 10))
    require(not stable.is_stable(), "unstable center must not lock")
    stable.reset()
    require(not stable.is_stable(), "reset must clear lock")
    print("PASS: insufficient / stable / displaced / reset frame windows")
    print("No ROS nodes, model inference, camera/serial access, or robot commands were used.")


if __name__ == "__main__":
    main()
