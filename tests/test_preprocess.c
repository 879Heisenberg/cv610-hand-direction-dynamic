#include "hand_direction/preprocess.h"
#include "hand_direction/types.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expression)                                                       \
    do {                                                                        \
        if (!(expression)) {                                                    \
            fprintf(stderr, "CHECK failed at %s:%d: %s\n",                    \
                    __FILE__, __LINE__, #expression);                           \
            return -1;                                                          \
        }                                                                       \
    } while (0)

static int nearly_equal(float actual, float expected)
{
    return fabsf(actual - expected) <= 1.0F / 255.0F + 1e-6F;
}

static float destination_value(const float *destination,
                               size_t row_floats,
                               uint32_t channel,
                               uint32_t y,
                               uint32_t x)
{
    return destination[(size_t)channel * HAND_MODEL_INPUT_HEIGHT * row_floats +
                       (size_t)y * row_floats + x];
}

static int check_uniform_destination(const float *destination,
                                     size_t row_stride,
                                     const float expected[3])
{
    const size_t row_floats = row_stride / sizeof(float);
    const float padding = (float)HAND_MODEL_INPUT_PADDING / 255.0F;
    uint32_t channel;
    uint32_t y;

    for (channel = 0U; channel < 3U; ++channel) {
        CHECK(nearly_equal(destination_value(destination,
                                             row_floats,
                                             channel,
                                             208U,
                                             208U),
                           expected[channel]));
    }
    for (y = 0U; y < HAND_MODEL_INPUT_HEIGHT; ++y) {
        size_t x;
        const float *row = destination + (size_t)y * row_floats;

        for (x = HAND_MODEL_INPUT_WIDTH; x < row_floats; ++x) {
            CHECK(nearly_equal(row[x], padding));
        }
    }
    return 0;
}

static int run_rgb_case(hand_pixel_format format, const float expected[3])
{
    const uint32_t width = 4U;
    const uint32_t height = 4U;
    const size_t source_stride = (size_t)width * 3U + 5U;
    const size_t source_bytes = source_stride * height;
    const size_t destination_stride = (size_t)HAND_MODEL_INPUT_WIDTH * sizeof(float) + 32U;
    const size_t destination_bytes =
        destination_stride * HAND_MODEL_INPUT_HEIGHT * HAND_MODEL_INPUT_CHANNELS;
    uint8_t *source = (uint8_t *)malloc(source_bytes);
    uint8_t *destination = (uint8_t *)malloc(destination_bytes);
    hand_frame frame = {0};
    hand_letterbox_transform transform;
    char error[128] = {0};
    int result;

    CHECK(source != NULL);
    CHECK(destination != NULL);
    memset(source, 0xCD, source_bytes);
    memset(destination, 0, destination_bytes);
    for (uint32_t y = 0U; y < height; ++y) {
        for (uint32_t x = 0U; x < width; ++x) {
            uint8_t *pixel = source + (size_t)y * source_stride + (size_t)x * 3U;

            if (format == HAND_PIXEL_FORMAT_RGB888) {
                pixel[0] = 10U;
                pixel[1] = 20U;
                pixel[2] = 30U;
            } else {
                pixel[0] = 30U;
                pixel[1] = 20U;
                pixel[2] = 10U;
            }
        }
    }

    frame.data = source;
    frame.width = width;
    frame.height = height;
    frame.stride = (uint32_t)source_stride;
    frame.format = format;
    result = hand_preprocess_frame(&frame,
                                   (float *)destination,
                                   destination_bytes,
                                   destination_stride,
                                   &transform,
                                   error,
                                   sizeof(error));
    CHECK(result == 0);
    CHECK(transform.resized_width == HAND_MODEL_INPUT_WIDTH);
    CHECK(transform.resized_height == HAND_MODEL_INPUT_HEIGHT);
    CHECK(transform.pad_left == 0U);
    CHECK(transform.pad_top == 0U);
    CHECK(check_uniform_destination((const float *)destination,
                                    destination_stride,
                                    expected) == 0);

    free(destination);
    free(source);
    return 0;
}

static int test_rgb_and_bgr(void)
{
    const float rgb_expected[3] = {10.0F / 255.0F, 20.0F / 255.0F, 30.0F / 255.0F};
    const float bgr_expected[3] = {10.0F / 255.0F, 20.0F / 255.0F, 30.0F / 255.0F};

    CHECK(run_rgb_case(HAND_PIXEL_FORMAT_RGB888, rgb_expected) == 0);
    CHECK(run_rgb_case(HAND_PIXEL_FORMAT_BGR888, bgr_expected) == 0);
    return 0;
}

static int test_gray(void)
{
    const uint32_t width = 4U;
    const uint32_t height = 4U;
    const size_t source_stride = 7U;
    const size_t source_bytes = source_stride * height;
    const size_t destination_stride = (size_t)HAND_MODEL_INPUT_WIDTH * sizeof(float);
    const size_t destination_bytes =
        destination_stride * HAND_MODEL_INPUT_HEIGHT * HAND_MODEL_INPUT_CHANNELS;
    uint8_t *source = (uint8_t *)malloc(source_bytes);
    uint8_t *destination = (uint8_t *)malloc(destination_bytes);
    hand_frame frame = {0};
    hand_letterbox_transform transform;
    const float expected[3] = {77.0F / 255.0F, 77.0F / 255.0F, 77.0F / 255.0F};
    char error[128] = {0};

    CHECK(source != NULL);
    CHECK(destination != NULL);
    memset(source, 0xCD, source_bytes);
    for (uint32_t y = 0U; y < height; ++y) {
        memset(source + (size_t)y * source_stride, 77, width);
    }
    memset(destination, 0, destination_bytes);
    frame.data = source;
    frame.width = width;
    frame.height = height;
    frame.stride = (uint32_t)source_stride;
    frame.format = HAND_PIXEL_FORMAT_GRAY8;
    CHECK(hand_preprocess_frame(&frame,
                                (float *)destination,
                                destination_bytes,
                                destination_stride,
                                &transform,
                                error,
                                sizeof(error)) == 0);
    CHECK(check_uniform_destination((const float *)destination,
                                    destination_stride,
                                    expected) == 0);
    free(destination);
    free(source);
    return 0;
}

static int test_nv21(void)
{
    const uint32_t width = 4U;
    const uint32_t height = 4U;
    const size_t stride = 7U;
    const size_t y_bytes = stride * height;
    const size_t source_bytes = y_bytes + stride * (height / 2U);
    const size_t destination_stride = (size_t)HAND_MODEL_INPUT_WIDTH * sizeof(float);
    const size_t destination_bytes =
        destination_stride * HAND_MODEL_INPUT_HEIGHT * HAND_MODEL_INPUT_CHANNELS;
    uint8_t *source = (uint8_t *)malloc(source_bytes);
    uint8_t *destination = (uint8_t *)malloc(destination_bytes);
    hand_frame frame = {0};
    hand_letterbox_transform transform;
    const float expected = 69.0F / 255.0F;
    const float expected_rgb[3] = {expected, expected, expected};
    char error[128] = {0};

    CHECK(source != NULL);
    CHECK(destination != NULL);
    memset(source, 76, y_bytes);
    memset(source + y_bytes, 128, source_bytes - y_bytes);
    memset(destination, 0, destination_bytes);
    frame.data = source;
    frame.width = width;
    frame.height = height;
    frame.stride = (uint32_t)stride;
    frame.format = HAND_PIXEL_FORMAT_NV21;
    CHECK(hand_preprocess_frame(&frame,
                                (float *)destination,
                                destination_bytes,
                                destination_stride,
                                &transform,
                                error,
                                sizeof(error)) == 0);
    CHECK(check_uniform_destination((const float *)destination,
                                    destination_stride,
                                    expected_rgb) == 0);
    free(destination);
    free(source);
    return 0;
}

static int test_letterbox_and_invalid_contracts(void)
{
    hand_letterbox_transform transform;
    uint8_t pixel = 0U;
    hand_frame frame = {0};
    char error[128] = {0};

    CHECK(hand_letterbox_init(640U, 360U, &transform) == 0);
    CHECK(transform.resized_width == 416U);
    CHECK(transform.resized_height == 234U);
    CHECK(transform.pad_left == 0U);
    CHECK(transform.pad_top == 91U);
    CHECK(hand_letterbox_init(0U, 360U, &transform) != 0);

    frame.data = &pixel;
    frame.width = 1U;
    frame.height = 1U;
    frame.stride = 0U;
    frame.format = HAND_PIXEL_FORMAT_GRAY8;
    CHECK(hand_preprocess_frame(&frame,
                                NULL,
                                0U,
                                0U,
                                &transform,
                                error,
                                sizeof(error)) != 0);
    CHECK(strstr(error, "stride is zero") != NULL);
    return 0;
}

static uint8_t ref_clamp(int value)
{
    if (value <= 0) return 0U;
    if (value >= 255) return 255U;
    return (uint8_t)value;
}

static void ref_nv21_rgb(const uint8_t *nv21,
                         uint32_t height,
                         uint32_t stride,
                         uint32_t x,
                         uint32_t y,
                         float rgb[3])
{
    const uint8_t *y_row = nv21 + (size_t)y * stride;
    const uint8_t *vu_row = nv21 + (size_t)stride * height +
                            (size_t)(y / 2U) * stride + (size_t)(x & ~1U);
    const int v = (int)vu_row[0] - 128;
    const int u = (int)vu_row[1] - 128;
    int luma = (int)y_row[x] - 16;

    if (luma < 0) luma = 0;
    rgb[0] = (float)ref_clamp((298 * luma + 409 * v + 128) >> 8);
    rgb[1] = (float)ref_clamp((298 * luma - 100 * u - 208 * v + 128) >> 8);
    rgb[2] = (float)ref_clamp((298 * luma + 516 * u + 128) >> 8);
}

static void fill_gradient_nv21(uint8_t *nv21, uint32_t width, uint32_t height, uint32_t seed)
{
    uint32_t y;
    uint8_t *vu = nv21 + (size_t)width * height;

    for (y = 0U; y < height; ++y) {
        uint32_t x;
        for (x = 0U; x < width; ++x) {
            nv21[(size_t)y * width + x] =
                (uint8_t)(16U + ((x * 3U + y * 5U + seed * 19U) % 220U));
        }
    }
    for (y = 0U; y < height / 2U; ++y) {
        uint32_t x;
        for (x = 0U; x < width; x += 2U) {
            vu[(size_t)y * width + x] =
                (uint8_t)(108U + ((x + y * 3U + seed * 7U) & 31U));
            vu[(size_t)y * width + x + 1U] =
                (uint8_t)(112U + ((x * 2U + y + seed * 11U) & 31U));
        }
    }
}

static int test_nv21_640x360_gradient_and_skip_padding(void)
{
    const uint32_t width = 640U;
    const uint32_t height = 360U;
    const size_t nv21_bytes = (size_t)width * height * 3U / 2U;
    const size_t dst_stride = (size_t)HAND_MODEL_INPUT_WIDTH * sizeof(float) + 32U;
    const size_t row_floats = dst_stride / sizeof(float);
    const size_t dst_bytes =
        dst_stride * HAND_MODEL_INPUT_HEIGHT * HAND_MODEL_INPUT_CHANNELS;
    const float padding = (float)HAND_MODEL_INPUT_PADDING / 255.0F;
    uint8_t *src1 = (uint8_t *)malloc(nv21_bytes);
    uint8_t *src2 = (uint8_t *)malloc(nv21_bytes);
    float *dst_full = (float *)malloc(dst_bytes);
    float *dst_fast = (float *)malloc(dst_bytes);
    float *dst_ref2 = (float *)malloc(dst_bytes);
    hand_frame frame = {0};
    hand_letterbox_transform transform;
    char error[128] = {0};
    uint32_t ch;
    uint32_t oy;

    CHECK(src1 != NULL && src2 != NULL);
    CHECK(dst_full != NULL && dst_fast != NULL && dst_ref2 != NULL);

    fill_gradient_nv21(src1, width, height, 1U);
    fill_gradient_nv21(src2, width, height, 2U);

    frame.data = src1;
    frame.width = width;
    frame.height = height;
    frame.stride = width;
    frame.format = HAND_PIXEL_FORMAT_NV21;
    CHECK(hand_preprocess_frame(&frame,
                                dst_full,
                                dst_bytes,
                                dst_stride,
                                &transform,
                                error,
                                sizeof(error)) == 0);
    CHECK(transform.resized_width == 416U);
    CHECK(transform.resized_height == 234U);
    CHECK(transform.pad_left == 0U);
    CHECK(transform.pad_top == 91U);

    /* Verify active area against independent floating-point bilinear math */
    for (oy = 0U; oy < transform.resized_height; ++oy) {
        const double py = ((double)oy + 0.5) * (double)height / (double)transform.resized_height - 0.5;
        int y0 = (int)floor(py);
        float wy = (float)(py - (double)y0);
        uint32_t y1;
        uint32_t ox;

        if (y0 < 0) { y0 = 0; wy = 0.0F; }
        if ((uint32_t)y0 >= height - 1U) { y0 = (int)height - 1; wy = 0.0F; }
        y1 = (uint32_t)y0 + 1U < height ? (uint32_t)y0 + 1U : (uint32_t)y0;

        for (ox = 0U; ox < transform.resized_width; ++ox) {
            const double px = ((double)ox + 0.5) * (double)width / (double)transform.resized_width - 0.5;
            int x0 = (int)floor(px);
            float wx = (float)(px - (double)x0);
            uint32_t x1;
            float p00[3], p10[3], p01[3], p11[3];

            if (x0 < 0) { x0 = 0; wx = 0.0F; }
            if ((uint32_t)x0 >= width - 1U) { x0 = (int)width - 1; wx = 0.0F; }
            x1 = (uint32_t)x0 + 1U < width ? (uint32_t)x0 + 1U : (uint32_t)x0;

            ref_nv21_rgb(src1, height, width, (uint32_t)x0, (uint32_t)y0, p00);
            ref_nv21_rgb(src1, height, width, x1, (uint32_t)y0, p10);
            ref_nv21_rgb(src1, height, width, (uint32_t)x0, y1, p01);
            ref_nv21_rgb(src1, height, width, x1, y1, p11);

            for (ch = 0U; ch < 3U; ++ch) {
                const float top = (1.0F - wx) * p00[ch] + wx * p10[ch];
                const float bot = (1.0F - wx) * p01[ch] + wx * p11[ch];
                const float expected = ((1.0F - wy) * top + wy * bot) / 255.0F;
                const float actual = destination_value(
                    dst_full, row_floats, ch, transform.pad_top + oy, ox);

                CHECK(fabsf(actual - expected) <= 1.0F / 255.0F + 1e-4F);
            }
        }
    }

    /* Verify static padding fill + repeated skip_padding_fill=1 calls */
    CHECK(hand_preprocess_fill_padding(dst_fast, dst_bytes, dst_stride, error, sizeof(error)) == 0);
    CHECK(hand_preprocess_frame_ex(&frame,
                                   dst_fast,
                                   dst_bytes,
                                   dst_stride,
                                   1,
                                   &transform,
                                   error,
                                   sizeof(error)) == 0);
    frame.data = src2;
    CHECK(hand_preprocess_frame_ex(&frame,
                                   dst_fast,
                                   dst_bytes,
                                   dst_stride,
                                   1,
                                   &transform,
                                   error,
                                   sizeof(error)) == 0);
    CHECK(hand_preprocess_frame(&frame,
                                dst_ref2,
                                dst_bytes,
                                dst_stride,
                                &transform,
                                error,
                                sizeof(error)) == 0);

    for (ch = 0U; ch < 3U; ++ch) {
        uint32_t y;
        for (y = 0U; y < HAND_MODEL_INPUT_HEIGHT; ++y) {
            size_t x;
            const int in_pad_bar =
                y < transform.pad_top ||
                y >= transform.pad_top + transform.resized_height;
            for (x = 0U; x < row_floats; ++x) {
                const float actual = destination_value(dst_fast, row_floats, ch, y, (uint32_t)x);
                const float reference = destination_value(dst_ref2, row_floats, ch, y, (uint32_t)x);

                CHECK(fabsf(actual - reference) <= 1e-6F);
                if (in_pad_bar || x >= HAND_MODEL_INPUT_WIDTH) {
                    CHECK(fabsf(actual - padding) <= 1e-6F);
                }
            }
        }
    }

    free(dst_ref2);
    free(dst_fast);
    free(dst_full);
    free(src2);
    free(src1);
    return 0;
}

int main(void)
{
    CHECK(test_letterbox_and_invalid_contracts() == 0);
    CHECK(test_rgb_and_bgr() == 0);
    CHECK(test_gray() == 0);
    CHECK(test_nv21() == 0);
    CHECK(test_nv21_640x360_gradient_and_skip_padding() == 0);
    puts("preprocess tests passed");
    return 0;
}
