# Coordinate mapping and calibration

[README](../README.md) · [Setup](setup.md) · [Testing](testing-and-running.md)

Calibration files are historical setup measurements, not portable robot settings. Recalibrate for the physical camera, observation pose, resolution, table, and center definition. Do not overwrite committed CSV/YAML evidence while inspecting it; fit to a separate file for a new setup.


The vision model outputs image pixel coordinates:

```text
pixel_x, pixel_y
```

The robot needs work-plane coordinates:

```text
robot_x, robot_y, robot_z
```

The current system follows the original Sagittarius linear mapping style:

```text
robot_x = k1 * pixel_y + b1
robot_y = k2 * pixel_x + b2
```

YAML format:

```yaml
LinearRegression:
  k1: ...
  b1: ...
  k2: ...
  b2: ...
```

Recommended calibration files:

| File | Usage |
| --- | --- |
| `vision_config_pick_front.yaml` | Front-view pick-stage calibration |
| `vision_config_place_front.yaml` | Front-view place-stage calibration |
| `vision_config_left.yaml` | Left-view calibration scaffold; requires site-specific validation |
| `vision_config_right.yaml` | Right-view calibration scaffold; requires site-specific validation |
| `vision_config_front.yaml` | Compatibility front-view config |
| `vision_config.yaml` | Original compatibility entry point |

The most reliable current real-robot results use:

```text
vision_config_pick_front.yaml
vision_config_place_front.yaml
```

Current 3x3 calibration data:

```text
manual_calibration_points_pick_front.csv
manual_calibration_points_place_front.csv
```

The calibration grid covers:

```text
x = 0.22, 0.24, 0.26
y = -0.03, 0.00, 0.03
```

Do not casually reuse `pick_front` calibration for `place_front`; the robot pose, camera view, and object type can differ.

Manual calibration flow:

1. Move the arm to a fixed observation pose.
2. Place a calibration object on the table, such as a blue block or bucket.
3. Detect the target with GroundingDINO.
4. Refine the center using HSV/contour processing inside the box.
5. Record `pixel_x, pixel_y, robot_x, robot_y, score, label`.
6. Save as `manual_calibration_points_*.csv`.
7. Fit `vision_config_*.yaml` with the manual calibration script.

Example calibration row:

```csv
pixel_x,pixel_y,robot_x,robot_y,score,label
383.7,376.9,0.22,-0.03,0.95,blue block
```

Calibration notes:

- Points should cover the actual grasp/place area, not only the image center.
- `pick_front` should cover where blocks usually appear.
- `place_front` should cover where buckets usually appear.
- Recalibrate if the camera position, arm observation pose, or resolution changes.
- If the detection is correct but the robot lands in the wrong location, check calibration before blaming GroundingDINO.

## Read-only fit inspection

With PyYAML installed, from the repository root:

```bash
PKG=src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector
python3 "$PKG/nodes/manual_vision_calibration.py" \
  --csv "$PKG/config/manual_calibration_points_pick_front.csv" \
  --vision-config "$PKG/config/vision_config_pick_front.yaml" --dry-run
python3 "$PKG/nodes/manual_vision_calibration.py" \
  --csv "$PKG/config/manual_calibration_points_place_front.csv" \
  --vision-config "$PKG/config/vision_config_place_front.yaml" --dry-run
```

`--dry-run` prints fitted coefficients and training-point residuals without writing YAML. Residuals are not held-out grasp or placement errors. The two CSVs contain nine points each; both cover x=0.22/0.24/0.26 m and y=-0.03/0/0.03 m. The archived log includes some mapped targets outside that grid, so it is not a workspace-wide accuracy evaluation.

## New calibration session

The collection launchers can move the arm to observation poses. Use them only in a prepared operator-controlled hardware session after [testing guidance](testing-and-running.md). Keep the same camera resolution and HSV/box-center definition for collection and runtime. Save new measurements separately from the committed experiment files. Existing left/right YAMLs are scaffolding for further validation, not evidence of calibrated multi-view performance.

The [work log](../WORKLOG.md) records why earlier box-center calibration should be regenerated after introducing contour refinement. It does not establish that every committed calibration point was collected after that change.
