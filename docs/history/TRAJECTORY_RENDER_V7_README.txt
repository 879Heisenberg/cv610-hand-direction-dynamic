Hand Direction - Trajectory Rendering Refactor v7

Based on the cumulative v6 patch (NEON preprocessing + inference interval 2).

This update changes only app/display_drm.c. Detection, tracking association,
gesture recognition thresholds, NPU inference, CPU preprocessing, and the
trajectory sampling/state logic are unchanged.

Rendering changes:
- Convert valid trajectory samples to display coordinates once per frame.
- Break strokes on invalid/out-of-crop points, timestamp reversal, or an
  implausibly large movement; never connect across a break.
- Apply a tiny 1-2-1 smoothing pass to interior points to reduce pixel jitter
  without introducing a large temporal delay.
- Draw a slightly thicker cyan stroke with small round caps and a distinct
  yellow current-point marker.

Validation:
- Host Release build and CTest passed (4/4).
- The host test suite does not compile the CV610 DRM display source.
- This environment does not contain the board's cross-compiled libdrm headers
  or CV610 SDK, so the updated display source still needs to be compiled and
  visually checked on the CV610 build environment/device.
