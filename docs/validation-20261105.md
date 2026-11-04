# Documentation and reproduction audit — 2026-11-05

[README](../README.md) · [Evidence](evidence.md) · [Setup](setup.md)

## Baseline and scope

Reviewed GitHub `main` at `a46ca3bd5ac2de44d2b20c44a63d4438e10b1cbf` (tree `bdc853f0734464f22428a280de0a757c5365b20b`). The existing local checkout was two README-only commits behind, at `bdf4a5a8851cd1abdfed5c2b95178a734df54803`. GitHub's compare API confirmed that all non-README source and artifact files matched. Work was prepared in an isolated copy, preserving the existing checkout's untracked reports and videos.

The change reorganizes the approximately 600-line English/mixed README into a 95-line presentation and focused guides. The earlier 988-line Chinese README is preserved verbatim after an archive header, with its original commit linked. The pre-reorganization English README remains linked at its immutable commit. `WORKLOG.md`, original artifacts, calibration data, launch files, control code, hardware parameters, and LICENSE are unchanged. The only new executable is a standard-library offline smoke checker.

## Checks performed

| Check | Observed result | What it establishes |
| --- | --- | --- |
| `python3 tools/offline_smoke.py` | PASS | 19 semantic Python files parsed; 11 detector launch/manifest XML files parsed; pure helper scenarios below passed. |
| Shell syntax | All 11 tracked `.sh` files passed `bash -n`, including `onekey.sh`, installer, and semantic helpers. | Shell parsing only, not runtime correctness, ShellCheck lint, installation, or hardware readiness. |
| Command parser smoke | PASS for single target, explicit pick/place, fixed same-color pairs, relative multi-step placement. | Supported pattern behavior on synthetic text, not broad language understanding. |
| Selection / stability smoke | PASS for invalid text, missing / low-confidence candidates, highest-score selection, center fallback/override, insufficient / stable / displaced / reset frame windows. | Pure helper behavior, not integrated ROS execution or a safety certification. |
| Calibration inspection | Both nine-point CSVs fit successfully with `--dry-run`; fitted coefficients matched stored YAML to absolute tolerance 1e-12. All detector YAML files parsed. | Internal consistency of archived calibration data. |
| Artifact review | Original 640×480 JPEG visually inspected; log excerpt and experiment note cross-checked. | Image is a perception preview; log records action returns and green missing-place rejection. |
| Markdown | New README submitted to GitHub's GFM rendering API; returned one H1, seven H2s, three tables, one image, code blocks, and Mermaid enrichment markup. | GitHub parsed the Markdown and preserved the three video links / relative image reference. Browser visual layout and Mermaid client-side rendering were not verified. |
| References | Local file and heading-anchor checks performed for README and active docs; public source links / artifact tree checked through GitHub APIs. | Linked repository targets exist; does not establish YouTube playback. |
| Attribution | Root LICENSE blob matches NXROBO upstream; MoveIt BSD and GroundingDINO Apache-2.0 source notices checked; inherited `TODO` manifest licenses recorded. | Attribution boundaries documented, without changing license text. |

Commands were run on macOS. The standard-library smoke used the system Python. Calibration / YAML checks used an already available Python environment containing PyYAML, with bytecode writes disabled; no dependency was installed or changed in another project. These checks do not reproduce the historical Noetic Python 3.8 / CUDA environment.

Calibration fit residuals on the same nine points:

| Mapping | Mean absolute x residual | Mean absolute y residual |
| --- | --- | --- |
| `pick_front` | 2.321 mm | 1.144 mm |
| `place_front` | 1.773 mm | 2.485 mm |

These are calibration training-point residuals, **not measured robot grasp or placement accuracy**.

## Important findings

- `execute_grasp=false` disables the language node's action client, while the launcher still creates `sgr_ctrl` with an independent initial home move and includes the real SDK. The image-test launcher also includes the real stack. Active documentation no longer calls those launchers hardware-free.
- The helper assumes `~/sagittarius_ws`, WSL-specific model/venv defaults, and offline Hugging Face caches. CUDA / torch library directories also appear in launch arguments and require site overrides.
- `onekey.sh` uses the caller's working directory for `catkin_make`; its install option performs system changes and includes legacy RealSense installation steps. It was syntax-checked, never executed.
- Demo 2's original description and the April 26 excerpt describe different outcomes. They remain separate. Rejected tasks are incomplete tasks, not successes.

## Not validated

No ROS master, launch file, serial device, camera, installer, MoveIt trajectory, robot control node, or robot action was started. No GroundingDINO checkpoint inference or clean Ubuntu / WSL2 / CUDA installation was performed. There was no physical robot rerun, no new video, no new task-success benchmark, no multi-view validation, and no claim of open-scene or human-interaction safety.

All three YouTube watch pages and oEmbed metadata requests failed to load from the available environment; the browser connection also timed out. The original video links are retained, but current public playback and the frame-level descriptions await independent viewing. GitHub API Markdown rendering was checked; full browser visual acceptance remains unconfirmed.
