Hand Direction - CPU Preprocess Optimization v5
================================================

Cumulative patch based on v4 (which already includes the v3 trajectory/display fixes).

Changes in v5:
- Adds ARM NEON NV21 row conversion for ARMv7 NEON targets.
- Converts source rows to RGB once and reuses two cached rows for bilinear resize,
  avoiding repeated YUV-to-RGB conversion of the same source pixels.
- Processes 8 pixels per NEON operation and handles any remaining pixels with a scalar tail.
- Preserves the existing YUV coefficients, bilinear sampling coordinates, NCHW layout,
  normalization and Letterbox padding behavior.
- Non-NEON builds retain the v4 scalar NV21 fast path.
- Retains v4 selective padding initialization and Release/-O3 configuration.

The supplied CV610 build cache already uses:
  -mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4 -O3
These flags enable the NEON path automatically.

Validation:
- Full cumulative Release CMake build succeeded.
- CTest 4/4 passed.
- 640x360 NV21 host output compared with v4: 519,168/519,168 FP32 values matched
  exactly; maximum absolute difference 0.
- ARMv7 NEON source compiled with clang for armv7-none-eabi, NEON-VFPv4, softfp,
  -O3, and -Werror.

Not yet verified: board-specific CV610 cross-build and measured board prep_ms.
This environment does not contain the original CV610 cross-compiler or target board.

Apply the files in this archive over the project root. This is cumulative; do not
apply v3 or v4 separately.
