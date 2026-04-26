# 2026-04-26 Front HSV Refinement Experiment

## Scope
- Front-view-only real-robot semantic grasping
- GroundingDINO for semantic target selection
- HSV contour center refinement for colored blocks and buckets

## Files
- `annotated_latest.jpg`
  - latest annotated detection snapshot for PPT/screenshots
- `language_guided_grasp_node_excerpt.log`
  - compact runtime log excerpt with key events only

## What changed in this round
- Added ROI color-center refinement:
  - DINO selects the object ROI
  - HSV contour center refines the geometric center inside the ROI
- Unified calibration and runtime center definition
  - calibration now uses the same refined center as runtime

## Observed real-robot outcome
- Same-color sorting command was expanded into 3 steps:
  - `red block -> red bucket`
  - `blue block -> blue bucket`
  - `green block -> green bucket`
- Runtime results recorded from the log excerpt:
  - red block grasp succeeded
  - red bucket place step succeeded
  - blue block grasp succeeded
  - blue bucket place step succeeded
  - green block grasp succeeded
  - green bucket was not found during placement

## Interpretation
- Front-view semantic grasping is now substantially more stable than the earlier
  box-center-only pipeline.
- Color-center refinement improved grasp center quality enough for repeated
  red/blue/green block pickup in the tested setup.
- Placement remains more sensitive than grasping, especially for bucket-center
  localization and for targets that are not clearly visible in the front view.

## Recommended PPT talking points
- "Model-swappable architecture: semantic detection and geometric center refinement are decoupled."
- "GroundingDINO is used for target understanding, while color contours refine the true grasp/place center."
- "Real robot completed multiple same-color sort steps in sequence with front-view-only perception."
