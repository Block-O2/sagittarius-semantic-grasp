# Evidence and claim boundaries

[README](../README.md) · [Validation audit](validation-20261105.md)

## Three original YouTube demonstrations

| Recording | Original description | Interpretation |
| --- | --- | --- |
| [Demo 1 — 4V7Y5Qn2Jmw](https://www.youtube.com/watch?v=4V7Y5Qn2Jmw) | Blue block picked and placed into a red bucket. | Illustrative end-to-end physical-arm run, not repeated-trial statistics. |
| [Demo 2 — vSotKdQC2tA](https://www.youtube.com/watch?v=vSotKdQC2tA) | Same-color stress test: red placement rejected after incomplete detection; blue completed; green bucket deliberately moved and placement rejected. | Blue completed; red and green tasks incomplete. Deliberate disturbance is not a measured natural failure rate. |
| [Demo 3 — i96AO-EEe4g](https://www.youtube.com/watch?v=i96AO-EEe4g) | Prompted placement onto a palm. | A reported non-bucket example; no claim that the object was absent from GroundingDINO pretraining, no broad generalization or human-interaction safety claim. |

Source: [original README demo descriptions](https://github.com/Block-O2/sagittarius-semantic-grasp/blob/a46ca3bd5ac2de44d2b20c44a63d4438e10b1cbf/README.md#demo). These descriptions are historical author reports. In the November audit the YouTube watch pages / oEmbed could not be fetched, and browser connection timed out. Playback, exact visible frame sequences, and public availability could therefore not be independently confirmed. The original IDs and links are preserved; no replacement footage or generated robot images were added.

## April 26 front-view experiment — a separate run

The committed [experiment note](../artifacts/2026-04-26-front-hsv-refinement/README.md), [runtime excerpt](../artifacts/2026-04-26-front-hsv-refinement/language_guided_grasp_node_excerpt.log), and [camera snapshot](../artifacts/2026-04-26-front-hsv-refinement/annotated_latest.jpg) remain unchanged.

The excerpt records CUDA backend loading and separate pick/place mappers. The three-step command is followed by:

| Event | Timestamp in saved log (timezone not specified) |
| --- | --- |
| Red grasp success | 18:19:22.335 |
| Red dynamic-place success | 18:19:40.316 |
| Blue grasp success | 18:19:57.831 |
| Blue dynamic-place success | 18:20:12.416 |
| Green grasp success | 18:20:29.428 |
| `place_target_not_found`; green request rejected | 18:20:32.599–603 |

This is a compact selected excerpt, not a full bag, complete trial set, or video-to-log synchronization record. Success messages are action-level results; final in-bucket accuracy was not independently measured here. Do not merge these red/blue outcomes with the different Demo 2 outcomes or report a combined success rate.

The README snapshot is an existing 640×480 annotated camera image showing a selected blue-block box/center and another lower-score candidate. It is a perception preview; it contains no view of a completed robot action and is not known to be from any of the three videos.

## What the source supports

- GroundingDINO detection, known-color HSV/contour refinement with fallback, score gating, stable-center checks, and independent pick/place calibration are implemented.
- The parser has explicit patterns for pick/place, relative placement, and fixed same-color red/blue/green expansion. It is not a general language planner.
- The executor uses Sagittarius actions backed by the existing MoveIt / SDK chain.
- Left/right view code/configuration exists, with views disabled by default. There is no evidence here of validated multi-view recovery or open-scene manipulation.
- No ablation or repeated controlled accuracy study isolates the benefit of HSV refinement. Historical claims of improved stability are observations, not quantified causal results.

[Architecture and source links](architecture.md) connect these claims to code. The [work log](../WORKLOG.md) preserves earlier placement-precision problems and the refinement decision; “next work” in that historical log does not describe the present documentation task.
