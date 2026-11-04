# Environment and workspace setup

[README](../README.md) · [Testing](testing-and-running.md) · [Troubleshooting](troubleshooting.md)

The historical development target is Ubuntu 20.04 (native or WSL2), ROS1 Noetic, MoveIt, and a CUDA-capable GPU. This guide documents the source requirements; it is not a claim that a fresh full ROS / CUDA environment was installed in the November audit. There is no dependency lockfile, original GroundingDINO commit, checkpoint hash, or complete historical environment export.

## Workspace layout

This repository **is the catkin workspace root**: it already contains `src/sagittarius_arm_ros`. Do not clone it inside another workspace's `src/` directory. For the historical helper defaults, clone it as `~/sagittarius_ws`, or set `SAGITTARIUS_WS` to the absolute clone path. Existing local workspaces should be kept separate.

```bash
git clone https://github.com/Block-O2/sagittarius-semantic-grasp.git "$HOME/sagittarius_ws"
cd "$HOME/sagittarius_ws"
export SAGITTARIUS_WS="$PWD"
```

After installing ROS Noetic from the [official Ubuntu installation guide](https://wiki.ros.org/noetic/Installation/Ubuntu), inspect dependencies before building. The source uses:

| Layer | Required components / source evidence |
| --- | --- |
| Build | catkin, C++11, Boost system, pkg-config, yaml-cpp, OpenCV headers; package CMakeLists |
| ROS Python / messages | rospy, actionlib, actionlib_msgs, geometry_msgs, sensor_msgs, std_msgs, cv_bridge, image_transport, message_generation/runtime, dynamic_reconfigure |
| Robot / planning | MoveIt commander and planning plugins, robot_state_publisher, joint_state_publisher, Sagittarius packages under `src/`; see `install.sh` and launch includes |
| Camera | ROS `usb_cam`, UVC/MJPEG input at 640×480, 10 FPS by default |
| Semantic Python | GroundingDINO, torch / torchvision, Pillow, OpenCV (`cv2`), NumPy, PyYAML, and GroundingDINO's transitive dependencies |

ROS package manifests are inherited and incomplete (some license fields are `TODO`; `image_transport` appears in CMake but not the detector manifest). `rosdep` alone is not a verified complete installer. On a prepared Ubuntu/ROS host, the intended build sequence is:

```bash
source /opt/ros/noetic/setup.bash
cd "$SAGITTARIUS_WS"
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

These installation/build commands were not executed by the documentation audit. Inspect apt/rosdep output and resolve the components listed above for the host; do not infer success from shell syntax checks.

## Existing installers and onekey.sh

[onekey.sh](../onekey.sh) is the inherited NXROBO interactive menu, not the semantic demo's one-command installer. It derives some paths from its own location, but `catkin_make` uses the **current directory**, so run it only from the workspace root. Menu option 103 calls [`install.sh`](../src/sagittarius_arm_ros/install.sh); other entries can start hardware examples.

The installer uses sudo, copies udev rules, triggers device discovery, installs many ROS packages, and adds a historical Intel RealSense apt repository with pinned packages / `apt-key`. The semantic UVC-camera flow does not itself require depth sensing. Review these legacy steps before using them; current repository availability and a clean full installation have not been verified. No installer was executed in this audit.

## GroundingDINO runtime

Use the [official GroundingDINO installation instructions](https://github.com/IDEA-Research/GroundingDINO#install) for a Python/PyTorch/torchvision/CUDA combination supported by that checkout. ROS Noetic's Python 3.8 `cv_bridge` also has to import in the **same interpreter** that runs the ROS nodes; an arbitrary new Python virtual environment is not sufficient. A virtual environment with access to system ROS packages may be needed. Record your chosen versions and checkpoint hash when establishing a new environment.

The adapter uses `groundingdino.models.build_model`, `SLConfig`, model transforms, torch, Pillow, and the `bert-base-uncased` text encoder. Weights are not shipped here. Configure real local paths:

```bash
export GROUNDINGDINO_ROOT="$HOME/GroundingDINO"
export GROUNDINGDINO_VENV="$HOME/groundingdino-venv"
export GROUNDINGDINO_CONFIG="$GROUNDINGDINO_ROOT/groundingdino/config/GroundingDINO_SwinT_OGC.py"
export GROUNDINGDINO_WEIGHTS="$GROUNDINGDINO_ROOT/weights/groundingdino_swint_ogc.pth"
```

The helper defaults to `/mnt/d/ai_models/...`, a historical WSL machine path, and `DEVICE=cuda`. Its `HF_HUB_OFFLINE=1` and `TRANSFORMERS_OFFLINE=1` defaults require the text encoder/tokenizer cache to be populated beforehand; merely downloading the `.pth` does not populate it. The adapter checks the config and weights and can fall back to CPU when CUDA is unavailable; this is not equivalent performance validation.

The main launch additionally defaults `cuda_home=/usr/local/cuda-12.1` and `torch_lib_dir=/mnt/d/ai_models/groundingdino-venv/lib/python3.8/site-packages/torch/lib`. Override both for your installation in an explicit launch. The helper does not forward environment overrides for these two launch arguments. Set `LD_LIBRARY_PATH` (even to an empty string if appropriate) before direct launch because the XML expands `$(env LD_LIBRARY_PATH)`.

Validate imports without starting ROS nodes, using the intended runtime interpreter after sourcing the ROS environment:

```bash
python3 -c 'import rospy, cv_bridge, yaml, cv2, numpy, torch, PIL, groundingdino; print("runtime imports available")'
```

This import check does not load weights, test inference, or validate hardware. Continue with [offline perception and staged operation](testing-and-running.md).
