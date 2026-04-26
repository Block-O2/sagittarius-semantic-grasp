# Work Log

## 2026-04-26 - Sagittarius semantic grasp front-view stabilization

### Background
- The project evolved from the original color-sorting demo into a language-guided grasping pipeline driven by GroundingDINO.
- The current real-robot workflow that is closest to stable operation is:
  1. startup initialization
  2. enter a calibrated front observation pose
  3. detect a pick target
  4. grasp
  5. detect a front placement target
  6. place

### What was changed in this round
- Added and iterated on a front-only calibrated runtime path using:
  - `vision_config_pick_front.yaml`
  - `vision_config_place_front.yaml`
- Reduced the default demo path to a simpler front-view-only setup:
  - `pick_front` for grasp
  - `place_front` for placement
  - left/right views disabled by default
- Added task parsing support for:
  - explicit multi-step commands
  - relative placement commands such as `left of` and `right of`
  - same-color pairing commands such as `put each block into the bucket of the same color`
- Simplified the default launch/runtime parameters so the demo can be started with a short launch command.
- Enabled CUDA inference for the current GroundingDINO environment and verified that the perception backend can load on `cuda`.

### Important findings from real-robot testing

#### 1. Hardware state can invalidate otherwise-correct software logic
- A large class of failures came from the robot start state, not from parsing or perception.
- Repeated MoveIt errors included:
  - `Start state appears to be in collision`
  - self-collision reports between `sgr532/link1` and `sgr532/link5`
- One physical arm was later found to have a loose joint.
- After switching to a healthier arm, startup-to-observation succeeded again.
- Conclusion:
  - when debugging "robot does not move", verify the actual arm state before blaming the front-view logic.

#### 2. Front-view calibration improved spread but did not fully solve placement precision
- The newer `pick_front` and `place_front` calibration files are healthier than the earlier tightly clustered samples.
- However, bucket drop precision is still insufficient for reliable "drop into bucket" behavior.
- In testing:
  - red and blue blocks could be grasped
  - green block was sometimes grasped off-center
  - blocks were placed near the correct bucket, but often not into the bucket center

#### 3. The main weakness is not only calibration residual; it is the center definition
- The current semantic pipeline mostly uses the GroundingDINO bounding-box center as the target center.
- This is acceptable for semantic selection, but weak for:
  - precise block grasp center estimation
  - precise bucket opening center estimation
- Different object colors and shapes can shift the bounding-box center even when detection is semantically correct.

### Architectural decision recorded here
- Keep the system model-swappable.
- Do **not** tightly bind grasp/placement geometry to GroundingDINO box centers.
- Preferred structure going forward:
  1. **semantic detector layer**
     - GroundingDINO today
     - replaceable later with another model
  2. **center refinement layer**
     - refine geometry inside the selected ROI
     - for colored blocks/buckets, use HSV contour center refinement
  3. **pixel-to-robot mapper**
     - keep the calibrated plane mapping layer independent
  4. **executor**
     - perform grasp/place using refined centers

This preserves the engineering goal that the detection model can be replaced later without rewriting the whole manipulation stack.

### Reference to original repo behavior
- The legacy color pipeline did **not** rely on generic detector box centers:
  - `grasp_once.py` used HSV segmentation and largest-contour center
  - `color_classification_fixed.py` also relied on color-specific geometry
- This is an important reference point for the next refactor.

### Current recommendation
- The next meaningful improvement should be:
  - keep GroundingDINO for semantic target selection
  - refine the center inside the selected ROI with HSV contour extraction for known color targets
  - then recalibrate `pick_front` and `place_front` once using the new center definition

### Open issues / next work
- Implement model-agnostic ROI center refinement:
  - DINO selects the target ROI
  - HSV refines center for `red/blue/green block` and `red/blue/green bucket`
- Recalibrate:
  - `vision_config_pick_front.yaml`
  - `vision_config_place_front.yaml`
- Re-evaluate:
  - grasp center on green block
  - placement center relative to bucket opening
  - same-color multi-step sorting task

### Known-good user-facing commands at the end of this round
- Startup:
  - `roslaunch sagittarius_object_color_detector language_guided_grasp.launch`
- Front-only same-color sorting:
  - `put each block into the bucket of the same color`
- Front-only relative placement:
  - `red block left of blue block`
  - `red block left of blue block, green block right of blue block`

## 2026-04-26 - Implemented ROI color-center refinement

### Change summary
- Added a model-agnostic center refinement layer:
  - new file: `perception_framework/center_refinement.py`
- The current behavior is now:
  1. GroundingDINO selects the semantic target ROI
  2. if the target text contains a known color (`red/green/blue` or Chinese equivalents),
     HSV contour extraction runs inside the selected ROI
  3. the largest contour center is used as the refined geometric center
  4. if refinement fails, the pipeline falls back to the original DINO box center

### Integration points
- `language_guided_grasp.py`
  - refined centers are now used at runtime before stability filtering and mapping
- `language_guided_calibration.py`
  - calibration now uses the same refined-center logic as runtime
- `detection_types.py`
  - `DetectionBox.center` now prefers `metadata["refined_center"]` when available
- `visualization.py`
  - overlays mark refined centers as `refined`
- `coordinate_mapping.py`
  - now also loads HSV color ranges from the vision config

### Practical implication
- Existing `pick_front` and `place_front` calibration files were generated with the old
  "DINO box center" definition, so they should be regenerated once with the new logic
  before evaluating grasp/place precision again.
