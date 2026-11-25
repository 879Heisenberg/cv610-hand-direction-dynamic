# Trajectory stability patch

This patch targets the reported `spider-web` trails during fast left/right hand swipes.

## Changes

- `src/tracker.c` / `include/hand_direction/tracker.h`
  - Add a time-scaled association gate. A high-confidence detection far from the current track can no longer steal the track in one frame.
  - Increase EMA responsiveness automatically as measured motion speed rises, while retaining the configured base smoothing for small jitter.
  - When no plausible association exists, emit no track point; the existing lost timeout controls reacquisition.
- `src/trajectory.c`
  - Reject an implausible one-frame jump by starting a fresh stroke at the new point. This prevents the outlier from being connected to both the old and new positions.
- `app/display_drm.c`
  - Add a second line-segment distance/time guard as a rendering safety net.
- `tests/`
  - Add regression tests for a remote high-confidence detection, fast direction reversal, and trajectory spikes.

## Initial tuning

- Tracker max speed: `4.5` normalized units/second.
- Tracker gate margin: `0.06` normalized units.
- EMA alpha: configured base alpha plus `0.10 * measured_speed`, clamped to `[0.25, 0.78]`.
- Trajectory/display jump gate: `0.06 + 4.5 * elapsed_seconds` normalized units.

These are starting values, not universal constants. Tune them with the actual camera FPS, detection cadence, and hand distance from camera. If real fast swipes are rejected, raise `max_speed_norm_per_second` and the corresponding `4.5` values together. If detector jumps still leak through, lower them carefully.

## Validation performed

Host CMake build succeeded and all four CTest suites passed. The CV610-specific DRM source was updated but could not be cross-compiled in this environment because the board SDK / cross toolchain is not present. Rebuild on the target SDK before deployment.

## Follow-up fix: points outside the visible crop

- `draw_gesture_overlay()` now rejects trajectory points outside the source-frame crop before calling the normalized-to-display conversion.
- This prevents the conversion helper's intentional edge clamping from turning off-crop points into artificial points on the screen border.
- The renderer also explicitly breaks the stroke after an implausibly large segment.
