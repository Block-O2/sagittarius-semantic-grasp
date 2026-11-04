# USB, camera, and troubleshooting

[README](../README.md) · [Setup](setup.md) · [Testing](testing-and-running.md)

## Hardware and Software Environment

Robot hardware:

- Sagittarius robotic arm.
- Sagittarius MoveIt configuration.
- Sagittarius SDK / `sgr_ctrl`.
- ROS namespace: `/sgr532`.
- Serial device usually mapped as `/dev/ttyACM0` or `/dev/sagittarius -> /dev/ttyACM0`.
- Under WSL2, USB serial passthrough usually requires Windows `usbipd`.

Camera hardware:

- UVC camera through ROS `usb_cam`.
- Default device: `/dev/video0`.
- Default image settings: 640x480, 10 FPS, MJPEG.

Compute platform:

- Ubuntu 20.04 / WSL2 Ubuntu 20.04.
- ROS Noetic.
- CUDA-capable GPU recommended for GroundingDINO.
- CPU inference is possible but too slow for comfortable real-robot testing.

## Inspect USB and camera devices

On the Ubuntu robot host, these commands list devices without starting robot nodes:

```bash
lsusb
ls -l /dev/ttyACM* /dev/sagittarius /dev/video*
```

Under WSL2, Windows `usbipd` attachment is a separate setup step. The serial helper can create `/dev/sagittarius` and change permissions using sudo; review it before running. Do not assume `/dev/ttyACM0` or `/dev/video0` identifies the same device after reconnecting. Prefer a stable UVC connection or native Ubuntu if passthrough repeatedly fails.

A two-machine ROS arrangement was proposed historically, but no portable two-machine deployment was validated in the November audit. Keep ROS master / hostname settings specific to the chosen host rather than copying old machine addresses.

## Detection is correct, but the arm misses

Check the active `vision_config*.yaml`, observation pose, camera resolution, and whether calibration used the same center definition as runtime. Inspect the raw frame, selected ROI, HSV-refined center, and mapped XY in order. HSV failure falls back to the box center; a plausible detector box does not prove accurate geometry.

## MoveIt reports self-collision or invalid start state

`Start state appears to be in collision`, invalid path, or contact between `sgr532/link1` and `sgr532/link5` can reflect an actual bad initial state. The [work log](../WORKLOG.md) also records a loose-joint arm during development. Inspect the physical arm and RViz state before a new session. Do not disable collision checking or change hardware safety settings to make a demo proceed.

## Placement target missing or unstable

The place stage observes again after grasping. A bucket outside the view, partially occluded, or deliberately moved may never form a stable target. Record this as a failed/incomplete task, even if rejection works as intended. The software's confidence and center checks do not establish full system safety.

## GroundingDINO does not load

Check the chosen interpreter imports `cv_bridge`, `torch`, `groundingdino`, and `cv2`; check config/weights paths and CUDA / torch library paths in [setup](setup.md). The helper's offline Hugging Face defaults require a cached BERT tokenizer/model. The checkpoint and historical package versions are not bundled or locked.

## Debug outputs


State topic:

```bash
rostopic echo /language_guided_grasp/state
```

Annotated image topic:

```text
/language_guided_grasp/annotated_image
```

Common saved images:

```text
/tmp/language_guided_grasp_raw_single_gpu.jpg
/tmp/language_guided_grasp_single_gpu.jpg
/tmp/language_guided_grasp_latest.jpg
```

If grasping is inaccurate, check:

1. Whether the raw image clearly shows the target.
2. Whether the annotated detection box is correct.
3. Whether the refined center is near the target center.
4. Whether the logged pixel center is reasonable.
5. Whether mapped x/y matches the actual table position.
6. Whether the active `vision_config` matches the current observation pose.

The helper and main launch use different image-saving defaults; see [testing](testing-and-running.md#4-operator-controlled-camera--ros-inspection). Published evidence is under [`artifacts/`](../artifacts/2026-04-26-front-hsv-refinement), not `/tmp`.

The detailed original troubleshooting and WSL instructions are preserved in [中文历史记录](history/README.zh-CN-20260426.md); the original English [Common Issues](https://github.com/Block-O2/sagittarius-semantic-grasp/blob/a46ca3bd5ac2de44d2b20c44a63d4438e10b1cbf/README.md#common-issues) remain pinned to their source revision.
