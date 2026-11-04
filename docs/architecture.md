# Architecture and source map

[README](../README.md) · [Testing](testing-and-running.md)

The semantic integration extends the original Sagittarius color-grasp examples. The execution wrapper calls the existing `SGRCtrlAction` path; MoveIt and the SDK remain responsible for robot planning and hardware communication.

## Repository Structure

Main package path:

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/
```

Important files:

| File | Role |
| --- | --- |
| `language_guided_grasp.py` | Main node for task parsing, detection, stability checks, coordinate mapping, grasp/place scheduling |
| `language_guided_calibration.py` | Language-guided calibration node for collecting calibration points from viewpoints |
| `manual_vision_calibration.py` | Fits `vision_config*.yaml` from manually collected pixel/robot coordinate pairs |
| `center_refinement.py` | HSV/contour refinement inside detection boxes |
| `task_parsing.py` | Converts simple language tasks into pick/place steps |
| `coordinate_mapping.py` | Loads calibration YAML and maps image pixels into the robot work plane |
| `execution.py` | Thin wrapper over the original `SGRCtrlAction` execution path |
| `grounding_dino.py` | GroundingDINO backend |
| `run_single_machine_gpu_grasp.sh` | Single-machine GPU real-robot test launcher |
| `ensure_sagittarius_serial.sh` | Checks/fixes Sagittarius serial mapping |

## Architecture

```text
Input layer
  - camera image /usb_cam/image_raw
  - text task /grasp_target_text

Perception layer
  - BasePerceptionBackend
  - GroundingDinoBackend
  - HSV / contour center refinement

Decision layer
  - target selection
  - confidence filtering
  - stable-frame detection
  - task state machine

Mapping layer
  - VisionPlaneMapper
  - vision_config_pick_front.yaml
  - vision_config_place_front.yaml
  - vision_config_left.yaml / right.yaml

Execution layer
  - SagittariusGraspExecutor
  - SGRCtrlAction
  - MoveIt
  - Sagittarius SDK
```

Main state flow:

```text
waiting_for_target -> detecting -> target_locked -> grasping -> placing -> done
```

Failure state:

```text
failed
```

Common failure reasons:

- Empty target text.
- No detection.
- Detection confidence below threshold.
- Unstable target center.
- MoveIt planning failure.
- Placement target not visible.
- Robot execution failure.

State topic:

```bash
rostopic echo /language_guided_grasp/state
```

## Reading the code

All paths below are relative to [`sagittarius_object_color_detector`](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector):

- [Main node](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/language_guided_grasp.py): task lifecycle, view selection, perception, and dispatch.
- [Parser](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/task_parsing.py): regex patterns and fixed same-color pairs; not an LLM.
- [Selection decisions](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/decision.py) and [stability](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/stability.py): confidence threshold plus adjacent-center displacement over a recent frame window. Stability is not object tracking or proof of scene safety.
- [Center refinement](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/center_refinement.py): largest HSV contour in the detector ROI for recognized red/green/blue text. On failure, `DetectionBox.center` falls back to box center.
- [Mapping](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/coordinate_mapping.py): `x=k1*pixel_y+b1`, `y=k2*pixel_x+b2`.
- [Executor](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/execution.py): action client for the existing [sgr_ctrl](../src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/sgr_ctrl.py).

The state flow above is a reading aid. Check the node and state-topic messages for actual substep and failure transitions. A reported executor success indicates the action result; it does not independently verify where an object finally landed.
