#include "hand_direction/preprocess.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define HAND_PREPROCESS_NEON 1
#else
#define HAND_PREPROCESS_NEON 0
#endif

#define HAND_SOURCE_MAX_DIMENSION 4096U
#define HAND_STACK_ROW_MAX_WIDTH 1920U
#define HAND_RGB_CHANNELS 3U
#define HAND_LETTERBOX_SIDES 2U
#define HAND_ROUND_HALF 0.5
#define HAND_PIXEL_MAX 255.0F
#define HAND_INVALID_ROW UINT32_MAX

typedef struct {
    uint32_t row_index[2];
    uint8_t *planes[2][HAND_RGB_CHANNELS];
} planar_rgb_row_cache;

static void preprocess_message(char *buffer, size_t size, const char *format, ...)
{
    va_list arguments;
    if (buffer == NULL || size == 0U) return;
    va_start(arguments, format);
    (void)vsnprintf(buffer, size, format, arguments);
    va_end(arguments);
}

#if !HAND_PREPROCESS_NEON
static uint8_t clamp_pixel(int value)
{
    if (value <= 0) return 0U;
    if (value >= 255) return 255U;
    return (uint8_t)value;
}
#endif

static uint32_t round_even(double value)
{
    const double whole = floor(value);
    const double fraction = value - whole;
    uint32_t rounded = (uint32_t)whole;
    if (fraction > HAND_ROUND_HALF ||
        (fraction == HAND_ROUND_HALF && (rounded & 1U) != 0U)) ++rounded;
    return rounded;
}

int hand_letterbox_init(uint32_t source_width, uint32_t source_height,
                        hand_letterbox_transform *transform)
{
    double scale;
    uint32_t resized_width, resized_height;
    if (transform == NULL || source_width == 0U || source_height == 0U ||
        source_width > HAND_SOURCE_MAX_DIMENSION || source_height > HAND_SOURCE_MAX_DIMENSION) return -1;
    scale = fmin((double)HAND_MODEL_INPUT_WIDTH / (double)source_width,
                 (double)HAND_MODEL_INPUT_HEIGHT / (double)source_height);
    resized_width = round_even((double)source_width * scale);
    resized_height = round_even((double)source_height * scale);
    if (resized_width == 0U || resized_height == 0U ||
        resized_width > HAND_MODEL_INPUT_WIDTH || resized_height > HAND_MODEL_INPUT_HEIGHT) return -1;
    transform->source_width = source_width;
    transform->source_height = source_height;
    transform->resized_width = resized_width;
    transform->resized_height = resized_height;
    transform->pad_left = (HAND_MODEL_INPUT_WIDTH - resized_width) / HAND_LETTERBOX_SIDES;
    transform->pad_top = (HAND_MODEL_INPUT_HEIGHT - resized_height) / HAND_LETTERBOX_SIDES;
    transform->scale = (float)scale;
    return 0;
}

static int validate_destination(const float *destination,
                                size_t destination_bytes,
                                size_t destination_row_stride,
                                size_t *out_row_floats,
                                char *error_message,
                                size_t error_message_size)
{
    const size_t minimum_row_stride = (size_t)HAND_MODEL_INPUT_WIDTH * sizeof(float);
    const size_t total_rows = (size_t)HAND_MODEL_INPUT_HEIGHT * HAND_MODEL_INPUT_CHANNELS;

    if (destination == NULL || (uintptr_t)destination % _Alignof(float) != 0U ||
        destination_row_stride < minimum_row_stride ||
        destination_row_stride % sizeof(float) != 0U ||
        total_rows > SIZE_MAX / destination_row_stride ||
        destination_bytes < total_rows * destination_row_stride) {
        preprocess_message(error_message,
                           error_message_size,
                           "invalid 416x416 FP32 NCHW destination");
        return -1;
    }
    if (out_row_floats != NULL) {
        *out_row_floats = destination_row_stride / sizeof(float);
    }
    return 0;
}

int hand_preprocess_fill_padding(float *destination,
                                 size_t destination_bytes,
                                 size_t destination_row_stride,
                                 char *error_message,
                                 size_t error_message_size)
{
    const float padding_value = (float)HAND_MODEL_INPUT_PADDING / HAND_PIXEL_MAX;
    size_t row_floats = 0U;
    size_t total_floats;
    size_t index;

    if (validate_destination(destination,
                             destination_bytes,
                             destination_row_stride,
                             &row_floats,
                             error_message,
                             error_message_size) != 0) {
        return -1;
    }

    total_floats = (size_t)HAND_MODEL_INPUT_CHANNELS * HAND_MODEL_INPUT_HEIGHT * row_floats;
    for (index = 0U; index < total_floats; ++index) {
        destination[index] = padding_value;
    }
    preprocess_message(error_message, error_message_size, "");
    return 0;
}

static int frame_layout_is_valid(const hand_frame *frame, char *error_message,
                                 size_t error_message_size)
{
    size_t width, height;
    if (frame == NULL || frame->data == NULL) {
        preprocess_message(error_message, error_message_size, "frame data is null"); return 0;
    }
    if (frame->width == 0U || frame->height == 0U ||
        frame->width > HAND_SOURCE_MAX_DIMENSION || frame->height > HAND_SOURCE_MAX_DIMENSION) {
        preprocess_message(error_message, error_message_size, "invalid frame dimensions %ux%u",
                           (unsigned int)frame->width, (unsigned int)frame->height); return 0;
    }
    width = (size_t)frame->width; height = (size_t)frame->height;
    if (frame->stride == 0U) {
        preprocess_message(error_message, error_message_size, "frame stride is zero"); return 0;
    }
    switch (frame->format) {
    case HAND_PIXEL_FORMAT_NV21: {
        size_t y_bytes, vu_bytes;
        if ((frame->width & 1U) != 0U || (frame->height & 1U) != 0U ||
            (size_t)frame->stride < width || height > SIZE_MAX / (size_t)frame->stride) {
            preprocess_message(error_message, error_message_size, "invalid NV21 dimensions or stride"); return 0;
        }
        y_bytes = (size_t)frame->stride * height;
        if (height / 2U > SIZE_MAX / (size_t)frame->stride) {
            preprocess_message(error_message, error_message_size, "NV21 VU plane overflows"); return 0;
        }
        vu_bytes = (size_t)frame->stride * (height / 2U);
        if (y_bytes > SIZE_MAX - vu_bytes) {
            preprocess_message(error_message, error_message_size, "NV21 frame size overflows"); return 0;
        }
        return 1;
    }
    case HAND_PIXEL_FORMAT_RGB888:
    case HAND_PIXEL_FORMAT_BGR888:
        if (width > SIZE_MAX / HAND_RGB_CHANNELS ||
            (size_t)frame->stride < width * HAND_RGB_CHANNELS ||
            height > SIZE_MAX / (size_t)frame->stride) {
            preprocess_message(error_message, error_message_size, "RGB/BGR stride is too small"); return 0;
        }
        return 1;
    case HAND_PIXEL_FORMAT_GRAY8:
        if ((size_t)frame->stride < width || height > SIZE_MAX / (size_t)frame->stride) {
            preprocess_message(error_message, error_message_size, "GRAY8 stride is too small"); return 0;
        }
        return 1;
    default:
        preprocess_message(error_message, error_message_size, "unsupported pixel format"); return 0;
    }
}

#if HAND_PREPROCESS_NEON
static inline uint8_t clamp_pixel(int value)
{
    if (value <= 0) return 0U;
    if (value >= 255) return 255U;
    return (uint8_t)value;
}

static inline uint8x8_t neon_yuv_channel(int16x8_t y, int16x8_t u, int16x8_t v,
                                         int u_coeff, int v_coeff)
{
    int32x4_t lo = vmull_n_s16(vget_low_s16(y), 298);
    int32x4_t hi = vmull_n_s16(vget_high_s16(y), 298);
    if (u_coeff != 0) {
        lo = vmlal_n_s16(lo, vget_low_s16(u), (int16_t)u_coeff);
        hi = vmlal_n_s16(hi, vget_high_s16(u), (int16_t)u_coeff);
    }
    if (v_coeff != 0) {
        lo = vmlal_n_s16(lo, vget_low_s16(v), (int16_t)v_coeff);
        hi = vmlal_n_s16(hi, vget_high_s16(v), (int16_t)v_coeff);
    }
    lo = vshrq_n_s32(vaddq_s32(lo, vdupq_n_s32(128)), 8);
    hi = vshrq_n_s32(vaddq_s32(hi, vdupq_n_s32(128)), 8);
    return vqmovun_s16(vcombine_s16(vqmovn_s32(lo), vqmovn_s32(hi)));
}

static void nv21_row_to_planar_neon(const hand_frame *frame,
                                    uint32_t y,
                                    uint8_t *r_out,
                                    uint8_t *g_out,
                                    uint8_t *b_out)
{
    const uint8_t *y_row = frame->data + (size_t)y * frame->stride;
    const uint8_t *vu_row = frame->data + (size_t)frame->stride * frame->height +
                            (size_t)(y / 2U) * frame->stride;
    uint32_t x = 0U;

    for (; x + 8U <= frame->width; x += 8U) {
        const uint8x8_t y8 = vld1_u8(y_row + x);
        const uint8x8_t vu8 = vld1_u8(vu_row + x);
        const uint8x8x2_t deinterleaved = vuzp_u8(vu8, vu8);
        const uint8x8x2_t vd = vzip_u8(deinterleaved.val[0], deinterleaved.val[0]);
        const uint8x8x2_t ud = vzip_u8(deinterleaved.val[1], deinterleaved.val[1]);
        int16x8_t yy = vreinterpretq_s16_u16(vmovl_u8(y8));
        int16x8_t vv = vreinterpretq_s16_u16(vmovl_u8(vd.val[0]));
        int16x8_t uu = vreinterpretq_s16_u16(vmovl_u8(ud.val[0]));

        yy = vmaxq_s16(vsubq_s16(yy, vdupq_n_s16(16)), vdupq_n_s16(0));
        vv = vsubq_s16(vv, vdupq_n_s16(128));
        uu = vsubq_s16(uu, vdupq_n_s16(128));

        vst1_u8(r_out + x, neon_yuv_channel(yy, uu, vv, 0, 409));
        vst1_u8(g_out + x, neon_yuv_channel(yy, uu, vv, -100, -208));
        vst1_u8(b_out + x, neon_yuv_channel(yy, uu, vv, 516, 0));
    }

    for (; x < frame->width; ++x) {
        const uint8_t *vu_pixel = vu_row + (size_t)(x & ~1U);
        const int v = (int)vu_pixel[0] - 128;
        const int u = (int)vu_pixel[1] - 128;
        int luma = (int)y_row[x] - 16;
        if (luma < 0) luma = 0;
        r_out[x] = clamp_pixel((298 * luma + 409 * v + 128) >> 8);
        g_out[x] = clamp_pixel((298 * luma - 100 * u - 208 * v + 128) >> 8);
        b_out[x] = clamp_pixel((298 * luma + 516 * u + 128) >> 8);
    }
}
#else
static void nv21_row_to_planar_scalar(const hand_frame *frame,
                                      uint32_t y,
                                      uint8_t *r_out,
                                      uint8_t *g_out,
                                      uint8_t *b_out)
{
    const uint8_t *y_row = frame->data + (size_t)y * frame->stride;
    const uint8_t *vu_row = frame->data + (size_t)frame->stride * frame->height +
                            (size_t)(y / 2U) * frame->stride;
    uint32_t x;

    for (x = 0U; x < frame->width; ++x) {
        const uint8_t *vu = vu_row + (size_t)(x & ~1U);
        const int v = (int)vu[0] - 128;
        const int u = (int)vu[1] - 128;
        int luma = (int)y_row[x] - 16;
        if (luma < 0) luma = 0;
        r_out[x] = clamp_pixel((298 * luma + 409 * v + 128) >> 8);
        g_out[x] = clamp_pixel((298 * luma - 100 * u - 208 * v + 128) >> 8);
        b_out[x] = clamp_pixel((298 * luma + 516 * u + 128) >> 8);
    }
}
#endif

static void source_row_to_planar_rgb(const hand_frame *frame,
                                     uint32_t y,
                                     uint8_t *const planes[HAND_RGB_CHANNELS])
{
    const uint8_t *row = frame->data + (size_t)y * frame->stride;
    uint8_t *r_out = planes[0];
    uint8_t *g_out = planes[1];
    uint8_t *b_out = planes[2];
    uint32_t x;

    if (frame->format == HAND_PIXEL_FORMAT_NV21) {
#if HAND_PREPROCESS_NEON
        nv21_row_to_planar_neon(frame, y, r_out, g_out, b_out);
#else
        nv21_row_to_planar_scalar(frame, y, r_out, g_out, b_out);
#endif
        return;
    }
    if (frame->format == HAND_PIXEL_FORMAT_RGB888) {
        for (x = 0U; x < frame->width; ++x) {
            r_out[x] = row[(size_t)x * 3U];
            g_out[x] = row[(size_t)x * 3U + 1U];
            b_out[x] = row[(size_t)x * 3U + 2U];
        }
    } else if (frame->format == HAND_PIXEL_FORMAT_BGR888) {
        for (x = 0U; x < frame->width; ++x) {
            r_out[x] = row[(size_t)x * 3U + 2U];
            g_out[x] = row[(size_t)x * 3U + 1U];
            b_out[x] = row[(size_t)x * 3U];
        }
    } else {
        (void)memcpy(r_out, row, (size_t)frame->width);
        (void)memcpy(g_out, row, (size_t)frame->width);
        (void)memcpy(b_out, row, (size_t)frame->width);
    }
}

static unsigned int get_planar_row_slot(const hand_frame *frame,
                                        planar_rgb_row_cache *cache,
                                        uint32_t row_index,
                                        uint32_t protected_row)
{
    unsigned int slot;
    for (slot = 0U; slot < 2U; ++slot) {
        if (cache->row_index[slot] == row_index) return slot;
    }
    for (slot = 0U; slot < 2U; ++slot) {
        if (cache->row_index[slot] != protected_row) break;
    }
    if (slot >= 2U) slot = 0U;
    source_row_to_planar_rgb(frame, row_index, cache->planes[slot]);
    cache->row_index[slot] = row_index;
    return slot;
}

static void prepare_axis_q8(uint32_t source_extent,
                            uint32_t destination_extent,
                            uint16_t indices[HAND_MODEL_INPUT_WIDTH],
                            uint16_t other_indices[HAND_MODEL_INPUT_WIDTH],
                            uint16_t w0_q8[HAND_MODEL_INPUT_WIDTH],
                            uint16_t w1_q8[HAND_MODEL_INPUT_WIDTH])
{
    uint32_t index;
    for (index = 0U; index < destination_extent; ++index) {
        const double position = ((double)index + HAND_ROUND_HALF) *
                                (double)source_extent / (double)destination_extent -
                                HAND_ROUND_HALF;
        int base = (int)floor(position);
        double fraction = position - (double)base;
        uint16_t w1;
        if (base < 0) {
            base = 0;
            fraction = 0.0;
        }
        if ((uint32_t)base >= source_extent - 1U) {
            base = (int)source_extent - 1;
            fraction = 0.0;
        }
        if (fraction < 0.0) {
            fraction = 0.0;
        } else if (fraction > 1.0) {
            fraction = 1.0;
        }
        w1 = (uint16_t)(fraction * 256.0 + 0.5);
        if (w1 > 256U) {
            w1 = 256U;
        }
        indices[index] = (uint16_t)base;
        other_indices[index] = (uint16_t)((uint32_t)base + 1U < source_extent
                                          ? (uint32_t)base + 1U
                                          : (uint32_t)base);
        w1_q8[index] = w1;
        w0_q8[index] = (uint16_t)(256U - w1);
    }
}

static void resize_planar_to_nchw(const hand_frame *frame,
                                  const hand_letterbox_transform *transform,
                                  float *destination,
                                  size_t row_floats,
                                  uint8_t *slot0_storage,
                                  uint8_t *slot1_storage)
{
    uint16_t x0[HAND_MODEL_INPUT_WIDTH], x1[HAND_MODEL_INPUT_WIDTH];
    uint16_t xw0[HAND_MODEL_INPUT_WIDTH], xw1[HAND_MODEL_INPUT_WIDTH];
    uint16_t y0[HAND_MODEL_INPUT_HEIGHT], y1[HAND_MODEL_INPUT_HEIGHT];
    uint16_t yw0[HAND_MODEL_INPUT_HEIGHT], yw1[HAND_MODEL_INPUT_HEIGHT];
    const size_t width = (size_t)frame->width;
    const size_t plane_floats = (size_t)HAND_MODEL_INPUT_HEIGHT * row_floats;
    const float inv_scale = 1.0F / (HAND_PIXEL_MAX * 65536.0F);
#if HAND_PREPROCESS_NEON
    const float32x4_t scale_v = vdupq_n_f32(inv_scale);
#endif
    planar_rgb_row_cache cache;
    uint32_t y_out;

    cache.row_index[0] = HAND_INVALID_ROW;
    cache.row_index[1] = HAND_INVALID_ROW;
    cache.planes[0][0] = slot0_storage;
    cache.planes[0][1] = slot0_storage + width;
    cache.planes[0][2] = slot0_storage + width * 2U;
    cache.planes[1][0] = slot1_storage;
    cache.planes[1][1] = slot1_storage + width;
    cache.planes[1][2] = slot1_storage + width * 2U;

    prepare_axis_q8(transform->source_width, transform->resized_width, x0, x1, xw0, xw1);
    prepare_axis_q8(transform->source_height, transform->resized_height, y0, y1, yw0, yw1);

    for (y_out = 0U; y_out < transform->resized_height; ++y_out) {
        const uint32_t out_y = transform->pad_top + y_out;
        const uint16_t wy0 = yw0[y_out];
        const uint16_t wy1 = yw1[y_out];
        const unsigned int slot0 = get_planar_row_slot(frame, &cache, y0[y_out], y1[y_out]);
        const unsigned int slot1 = get_planar_row_slot(frame, &cache, y1[y_out], y0[y_out]);
#if HAND_PREPROCESS_NEON
        const uint16x4_t wy0_v = vdup_n_u16(wy0);
        const uint16x4_t wy1_v = vdup_n_u16(wy1);
#endif
        uint32_t channel;

        for (channel = 0U; channel < HAND_RGB_CHANNELS; ++channel) {
            const uint8_t *src0_ch = cache.planes[slot0][channel];
            const uint8_t *src1_ch = cache.planes[slot1][channel];
            float *dst_row = destination + (size_t)channel * plane_floats +
                             (size_t)out_y * row_floats + transform->pad_left;
            uint32_t ox = 0U;

#if HAND_PREPROCESS_NEON
            for (; ox + 8U <= transform->resized_width; ox += 8U) {
                const uint8_t b00[8] = {
                    src0_ch[x0[ox + 0U]], src0_ch[x0[ox + 1U]],
                    src0_ch[x0[ox + 2U]], src0_ch[x0[ox + 3U]],
                    src0_ch[x0[ox + 4U]], src0_ch[x0[ox + 5U]],
                    src0_ch[x0[ox + 6U]], src0_ch[x0[ox + 7U]]
                };
                const uint8_t b10[8] = {
                    src0_ch[x1[ox + 0U]], src0_ch[x1[ox + 1U]],
                    src0_ch[x1[ox + 2U]], src0_ch[x1[ox + 3U]],
                    src0_ch[x1[ox + 4U]], src0_ch[x1[ox + 5U]],
                    src0_ch[x1[ox + 6U]], src0_ch[x1[ox + 7U]]
                };
                const uint8_t b01[8] = {
                    src1_ch[x0[ox + 0U]], src1_ch[x0[ox + 1U]],
                    src1_ch[x0[ox + 2U]], src1_ch[x0[ox + 3U]],
                    src1_ch[x0[ox + 4U]], src1_ch[x0[ox + 5U]],
                    src1_ch[x0[ox + 6U]], src1_ch[x0[ox + 7U]]
                };
                const uint8_t b11[8] = {
                    src1_ch[x1[ox + 0U]], src1_ch[x1[ox + 1U]],
                    src1_ch[x1[ox + 2U]], src1_ch[x1[ox + 3U]],
                    src1_ch[x1[ox + 4U]], src1_ch[x1[ox + 5U]],
                    src1_ch[x1[ox + 6U]], src1_ch[x1[ox + 7U]]
                };
                const uint16x8_t p00 = vmovl_u8(vld1_u8(b00));
                const uint16x8_t p10 = vmovl_u8(vld1_u8(b10));
                const uint16x8_t p01 = vmovl_u8(vld1_u8(b01));
                const uint16x8_t p11 = vmovl_u8(vld1_u8(b11));
                const uint16x8_t wx0 = vld1q_u16(xw0 + ox);
                const uint16x8_t wx1 = vld1q_u16(xw1 + ox);
                const uint16x8_t top = vmlaq_u16(vmulq_u16(p00, wx0), p10, wx1);
                const uint16x8_t bot = vmlaq_u16(vmulq_u16(p01, wx0), p11, wx1);
                const uint32x4_t acc_lo = vmlal_u16(vmull_u16(vget_low_u16(top), wy0_v),
                                                    vget_low_u16(bot), wy1_v);
                const uint32x4_t acc_hi = vmlal_u16(vmull_u16(vget_high_u16(top), wy0_v),
                                                    vget_high_u16(bot), wy1_v);

                vst1q_f32(dst_row + ox, vmulq_f32(vcvtq_f32_u32(acc_lo), scale_v));
                vst1q_f32(dst_row + ox + 4U, vmulq_f32(vcvtq_f32_u32(acc_hi), scale_v));
            }
#endif
            for (; ox < transform->resized_width; ++ox) {
                const uint32_t ix0 = x0[ox];
                const uint32_t ix1 = x1[ox];
                const uint32_t wx0 = xw0[ox];
                const uint32_t wx1 = xw1[ox];
                const uint32_t top = (uint32_t)src0_ch[ix0] * wx0 +
                                     (uint32_t)src0_ch[ix1] * wx1;
                const uint32_t bot = (uint32_t)src1_ch[ix0] * wx0 +
                                     (uint32_t)src1_ch[ix1] * wx1;
                const uint32_t acc = top * (uint32_t)wy0 + bot * (uint32_t)wy1;

                dst_row[ox] = (float)acc * inv_scale;
            }
        }
    }
}

int hand_preprocess_frame_ex(const hand_frame *frame,
                             float *destination,
                             size_t destination_bytes,
                             size_t destination_row_stride,
                             int skip_padding_fill,
                             hand_letterbox_transform *transform,
                             char *error_message,
                             size_t error_message_size)
{
    const float padding_value = (float)HAND_MODEL_INPUT_PADDING / HAND_PIXEL_MAX;
    uint8_t stack_storage[2U * HAND_RGB_CHANNELS * HAND_STACK_ROW_MAX_WIDTH];
    size_t row_floats = 0U;
    uint8_t *heap_storage = NULL;
    uint8_t *row_storage;
    size_t row_bytes;

    if (!frame_layout_is_valid(frame, error_message, error_message_size)) return -1;
    if (validate_destination(destination,
                             destination_bytes,
                             destination_row_stride,
                             &row_floats,
                             error_message,
                             error_message_size) != 0 ||
        transform == NULL ||
        hand_letterbox_init(frame->width, frame->height, transform) != 0) {
        preprocess_message(error_message, error_message_size, "invalid 416x416 FP32 NCHW destination or letterbox");
        return -1;
    }

    if (skip_padding_fill == 0) {
        size_t channel;

        /* Only clear letterbox bars and aligned row tails; resized image pixels
           are overwritten below. */
        for (channel = 0U; channel < HAND_MODEL_INPUT_CHANNELS; ++channel) {
            size_t y;
            float *channel_base = destination + channel * (size_t)HAND_MODEL_INPUT_HEIGHT * row_floats;
            for (y = 0U; y < HAND_MODEL_INPUT_HEIGHT; ++y) {
                float *row = channel_base + y * row_floats;
                const int image_row = y >= transform->pad_top &&
                                      y < (size_t)transform->pad_top + transform->resized_height;
                const size_t image_left = image_row ? transform->pad_left : 0U;
                const size_t image_right = image_row
                    ? (size_t)transform->pad_left + transform->resized_width : 0U;
                size_t x;
                for (x = 0U; x < image_left; ++x) row[x] = padding_value;
                for (x = image_right; x < row_floats; ++x) row[x] = padding_value;
            }
        }
    }

    row_bytes = (size_t)frame->width * HAND_RGB_CHANNELS;
    if (frame->width <= HAND_STACK_ROW_MAX_WIDTH) {
        row_storage = stack_storage;
    } else {
        if (row_bytes > SIZE_MAX / 2U) {
            preprocess_message(error_message, error_message_size, "RGB row cache size overflows");
            return -1;
        }
        heap_storage = (uint8_t *)malloc(row_bytes * 2U);
        if (heap_storage == NULL) {
            preprocess_message(error_message, error_message_size, "unable to allocate RGB row cache");
            return -1;
        }
        row_storage = heap_storage;
    }

    resize_planar_to_nchw(frame,
                          transform,
                          destination,
                          row_floats,
                          row_storage,
                          row_storage + row_bytes);

    free(heap_storage);
    preprocess_message(error_message, error_message_size, "");
    return 0;
}

int hand_preprocess_frame(const hand_frame *frame, float *destination,
                          size_t destination_bytes, size_t destination_row_stride,
                          hand_letterbox_transform *transform,
                          char *error_message, size_t error_message_size)
{
    return hand_preprocess_frame_ex(frame,
                                    destination,
                                    destination_bytes,
                                    destination_row_stride,
                                    0,
                                    transform,
                                    error_message,
                                    error_message_size);
}
