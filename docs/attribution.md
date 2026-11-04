# Attribution and license scope

[README](../README.md)

This project integrates semantic perception and task handling into the Sagittarius robot workspace. Credit for the hardware control backbone belongs to its upstream authors; the repository is not a new robot SDK, new MoveIt implementation, or a newly trained foundation detector.

| Component | Provenance and role | License evidence / boundary |
| --- | --- | --- |
| Sagittarius workspace, models, examples, SDK/control chain, `onekey.sh` | [NXROBO/sagittarius_ws](https://github.com/NXROBO/sagittarius_ws); local `src/sagittarius_arm_ros`, with extensions in this repository | Root [LICENSE](../LICENSE) is BSD-3-Clause, copyright 2021 NXROBO. Its blob is identical to [upstream LICENSE](https://github.com/NXROBO/sagittarius_ws/blob/main/LICENSE). Retain notices. The exact upstream source revision for every vendored file was not established. |
| Project semantic integration | `language_guided_grasp.py`, calibration tools, `nodes/perception_framework`, language launchers and scripts; see [contributions](../README.md#my-contributions) | Distributed within this repository under its existing BSD license context. The existing NXROBO notice does not attribute authorship of these additions to NXROBO, nor does this document create a new copyright notice or relicense any file. |
| ROS1 Noetic and individual ROS packages | [ROS](https://www.ros.org/), catkin, messages, camera, bridge, and action interfaces | Separate packages retain their own licenses; “ROS” is not one blanket license. Inspect installed package notices before redistribution. |
| MoveIt | [MoveIt source](https://github.com/moveit/moveit/tree/noetic-devel) and generated Sagittarius configuration | Upstream MoveIt LICENSE.txt is BSD-3-Clause (Willow Garage and contributors); it remains an external dependency. Local `sagittarius_moveit/package.xml` declares BSD; this does not relicense the full dependency tree. Consult [upstream notices](https://github.com/moveit/moveit/blob/noetic-devel/LICENSE.txt). |
| GroundingDINO | [IDEA-Research/GroundingDINO](https://github.com/IDEA-Research/GroundingDINO), pretrained language-conditioned object detection | Upstream code [LICENSE](https://github.com/IDEA-Research/GroundingDINO/blob/main/LICENSE) is Apache-2.0. It is imported externally, not relicensed BSD by this repository. Obtain weights and text-encoder assets from their original sources and check their applicable terms separately. |
| Other external Python / native libraries | PyTorch, torchvision, OpenCV, NumPy, Pillow, PyYAML, CUDA and their dependencies | Each retains its own notices and terms; none is relicensed by this repository's root LICENSE. No model weights or new third-party binaries are added in the documentation update. |

## Remaining attribution limits

Several inherited package manifests, including the SDK and detector, contain `<license>TODO</license>`. Robot description / Gazebo / MoveIt manifests declare BSD. These inconsistent package fields are recorded rather than guessed or silently replaced. The BSD root license permits source modification subject to its conditions, but its existing NXROBO copyright line alone does not establish authorship or a separate explicit grant from every later contributor. Contributor copyright notices and complete per-file provenance could be clarified by the owner in a future licensing review.

The root LICENSE and all existing file headers remain unchanged. This documentation describes the observed distribution and attribution boundaries; it does not claim legal clearance for every external asset, checkpoint, or dependency, or NXROBO endorsement of the project.
