# Testing and running

[README](../README.md) · [Setup](setup.md) · [Calibration](calibration.md)

## 1. No ROS, model, camera, or robot

From the repository root with Python 3.8 or newer:

```bash
python3 tools/offline_smoke.py
```

This standard-library-only check imports pure parsing, selection, detection-data, and stability helpers. It verifies supported task patterns and rejection of empty, missing, low-confidence, and unstable synthetic inputs. It sends no robot requests and does not test model inference or the ROS state machine.

For read-only calibration inspection with PyYAML, use the two `--dry-run` commands in [calibration](calibration.md#read-only-fit-inspection). No calibration values are changed.

## 2. Offline model inference on a local image

After [setting up GroundingDINO](setup.md#groundingdino-runtime), the existing CLI below runs inference without importing the robot executor or starting ROS. Use a real saved camera image for meaningful perception inspection; the bundled `sample_red_block.ppm` is a synthetic fixture, not a robot experiment.

```bash
PKG=src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector
python3 "$PKG/nodes/perception_backend_smoke_test.py" \
  --image artifacts/2026-04-26-front-hsv-refinement/annotated_latest.jpg \
  --text 'blue block' --device cpu \
  --config "$GROUNDINGDINO_CONFIG" --weights "$GROUNDINGDINO_WEIGHTS" \
  --output /tmp/sagittarius-perception-preview.jpg
```

The archived JPEG already contains annotations; this example checks the CLI path, not a clean-image benchmark. Substitute a new unannotated local frame for an inference assessment. The CLI draws detections and selection decisions; it does **not** exercise HSV refinement, stable-frame accumulation, or execution. No model inference was run during the documentation audit.

## 3. Understand the existing ROS launchers before use

| Entry point | What it actually starts |
| --- | --- |
| `language_guided_grasp.launch` | `demo_true.launch` → real SDK / MoveIt, `sgr_ctrl.py`, camera, language node. `execute_grasp` and `sgr_ctrl_init_pose` default true. |
| `run_single_machine_gpu_grasp.sh` | Same launch. Defaults `EXECUTE_GRASP=false` but `SGR_CTRL_INIT_POSE=true`; initializes the controller independently. Defaults workspace to `~/sagittarius_ws`. |
| `language_guided_grasp_image_test.launch` | A local image publisher **plus real SDK / MoveIt and sgr_ctrl**; defaults `execute_grasp=true`, and exposes no `sgr_ctrl_init_pose` argument. Not an isolated vision test. |
| `language_guided_calibration_base.launch` / calibration helper | Robot-assisted calibration; inspect includes and pose moves before use. |
| `ensure_sagittarius_serial.sh` | May change device symlinks/permissions with sudo; not a read-only device check. |

`execute_grasp=false` disables the language node's action client and its observation scanning. It does **not** disable the separate controller's initial home move, SDK initialization, or other nodes on the ROS network. Do not use these launchers as a no-hardware smoke test. `drop_after_grasp=false` also does not remove an explicit task's dynamic place phase; it controls the fallback fixed drop after a pick-only command.

## 4. Operator-controlled camera / ROS inspection

Only proceed on a prepared robot host after the physical arm state, device mapping, calibrated poses, table, and camera have been checked by the operator. The following reduces application-level motion triggers but still starts a real driver; it is not an isolated no-motion environment. Do not connect to a live remote ROS master for an offline check.

After activating the compatible GroundingDINO environment and sourcing ROS / the workspace, set host-specific CUDA paths and use an explicit launch:

```bash
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
roslaunch sagittarius_object_color_detector language_guided_grasp.launch \
  execute_grasp:=false sgr_ctrl_init_pose:=false \
  move_to_search_pose_on_startup:=false search_pose_mode:=none \
  rejection_motion_enabled:=false \
  groundingdino_config:="$GROUNDINGDINO_CONFIG" \
  groundingdino_weights:="$GROUNDINGDINO_WEIGHTS" \
  cuda_home:="$CUDA_HOME" torch_lib_dir:="$SAG_TORCH_LIB_DIR"
```

Set `CUDA_HOME` and `SAG_TORCH_LIB_DIR` to the actual installation directories first; see [setup](setup.md). Use `video_dev:=/dev/video2` if appropriate. This command was reviewed statically, not launched in the audit.

Inspect the image and mapped values in this mode:

```bash
rostopic echo /language_guided_grasp/state
rostopic pub -1 /grasp_target_text std_msgs/String "data: 'red block'"
```

The annotated topic is `/language_guided_grasp/annotated_image`. Confirm the selected object and refined center, changing XY with object motion, and the active calibration for the current camera pose. The main launch does not save raw/annotated files by default; the helper enables both, saving `/tmp/language_guided_grasp_raw_single_gpu.jpg` and `/tmp/language_guided_grasp_single_gpu.jpg`. `/tmp` files are session output, not published artifacts.

## 5. Physical grasp / pick-place session

Real motion remains operator-controlled. The historical front-only entry point is `language_guided_grasp.launch`; the helper enables task execution with `EXECUTE_GRASP=true`. A bare main launch already enables execution and controller initialization, so do not treat it as a preview command.

Before enabling execution, validate the environment and site calibration above. Begin with one known object and inspect the result before a multi-step task. `pick_z`, drop height, view poses, and the relative-placement offset are physical settings; this documentation update does not change them or supply portable safe values.

Supported examples:

```text
red block
pick red block and place it into red bucket
put each block into the bucket of the same color
red block left of blue block
```

Same-color expansion uses three fixed pairs. A missing or unstable place target can leave a picked object held while the request fails; rejection is not automatic task recovery. See [evidence](evidence.md) for the recorded outcomes. No real-arm actions were issued in the November audit.

## Parameter reference

The main launch defaults to `box_threshold=0.35`, `text_threshold=0.25`, `min_grasp_score=0.35`, `stable_required=5`, and `center_tolerance=8.0` pixels. The GPU helper overrides the last two to 3 and 12.0; placement scanning defaults to two stable frames and front-only views. These are existing settings, not recommendations to relax gates.

The complete historical parameter, launch, and test-mode notes remain in [中文历史记录](history/README.zh-CN-20260426.md) and the [pre-reorganization README](https://github.com/Block-O2/sagittarius-semantic-grasp/blob/a46ca3bd5ac2de44d2b20c44a63d4438e10b1cbf/README.md#key-parameters). Read them with the launcher caveat above.
