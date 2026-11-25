Hand Direction - 30/20/2 configuration
======================================

Configuration:
- Camera sensor target: 30 FPS
- Direction window: 667 ms (nominal 20 camera frames at 30 FPS)
- NPU inference interval: every 2 processed camera frames
- Includes the v5 ARM NEON preprocessing path

Expected behavior at the current measured application rate of about 19.2 FPS:
- Inference updates: about 9.6 per second (roughly one every 104 ms)
- Trajectory points inside the 667 ms window: about 6.4

The application camera_fps value is still limited by synchronous LCD rendering
(about 49 ms per frame), so changing the inference interval does not by itself
raise camera_fps to 30. Validate board-side camera_fps, prep_ms, CPU load, and
gesture recognition consistency after deployment.
