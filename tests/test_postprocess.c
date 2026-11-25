#include "hand_direction/postprocess.h"
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

static void set_output(float *output,
                       size_t channel_stride,
                       size_t channel,
                       size_t candidate,
                       float value)
{
    output[channel * (channel_stride / sizeof(float)) + candidate] = value;
}

static void set_candidate(float *output,
                          size_t channel_stride,
                          size_t candidate,
                          float center_x,
                          float center_y,
                          float width,
                          float height,
                          float score)
{
    set_output(output, channel_stride, 0U, candidate, center_x);
    set_output(output, channel_stride, 1U, candidate, center_y);
    set_output(output, channel_stride, 2U, candidate, width);
    set_output(output, channel_stride, 3U, candidate, height);
    set_output(output, channel_stride, 4U, candidate, score);
}

static int nearly_equal(float actual, float expected)
{
    return fabsf(actual - expected) <= 1e-5F;
}

static int test_decode_nms_and_letterbox(void)
{
    const size_t candidate_count = 6U;
    const size_t channel_stride = candidate_count * sizeof(float) + 16U;
    const size_t output_bytes = HAND_MODEL_OUTPUT_CHANNELS * channel_stride;
    float *output = (float *)malloc(output_bytes);
    hand_letterbox_transform transform;
    hand_postprocess_config config;
    hand_postprocess_workspace workspace = {0};
    hand_detection detections[8] = {0};
    size_t detection_count = 99U;
    char error[128] = {0};

    CHECK(output != NULL);
    memset(output, 0xAB, output_bytes);
    for (size_t channel = 0U; channel < HAND_MODEL_OUTPUT_CHANNELS; ++channel) {
        for (size_t candidate = 0U; candidate < candidate_count; ++candidate) {
            set_output(output, channel_stride, channel, candidate, 0.0F);
        }
    }

    CHECK(hand_letterbox_init(200U, 100U, &transform) == 0);
    CHECK(transform.resized_width == 416U);
    CHECK(transform.resized_height == 208U);
    CHECK(transform.pad_top == 104U);

    /* First two boxes overlap and must be reduced by NMS. */
    set_candidate(output, channel_stride, 0U, 208.0F, 208.0F, 416.0F, 208.0F, 0.90F);
    set_candidate(output, channel_stride, 1U, 209.0F, 208.0F, 410.0F, 208.0F, 0.80F);
    set_candidate(output, channel_stride, 2U, 100.0F, 208.0F, 50.0F, 50.0F, 0.70F);
    /* The remaining candidates are invalid or below threshold and must be ignored. */
    set_candidate(output, channel_stride, 3U, NAN, 208.0F, 100.0F, 100.0F, 0.95F);
    set_candidate(output, channel_stride, 4U, 208.0F, 208.0F, 0.0F, 100.0F, 0.95F);
    set_candidate(output, channel_stride, 5U, 208.0F, 208.0F, 100.0F, 100.0F, 0.10F);

    hand_postprocess_config_init_defaults(&config);
    CHECK(config.confidence_threshold == HAND_POSTPROCESS_DEFAULT_CONFIDENCE);
    CHECK(config.nms_iou_threshold == HAND_POSTPROCESS_DEFAULT_NMS_IOU);
    CHECK(hand_postprocess_workspace_init(&workspace, candidate_count) == 0);
    CHECK(hand_postprocess_decode(output,
                                  output_bytes,
                                  channel_stride,
                                  candidate_count,
                                  &transform,
                                  0,
                                  &config,
                                  &workspace,
                                  detections,
                                  8U,
                                  &detection_count,
                                  error,
                                  sizeof(error)) == 0);
    CHECK(detection_count == 2U);
    CHECK(detections[0].class_id == 0);
    CHECK(detections[1].class_id == 0);
    CHECK(nearly_equal(detections[0].confidence, 0.90F));
    CHECK(nearly_equal(detections[0].x0, 0.0F));
    CHECK(nearly_equal(detections[0].y0, 0.0F));
    CHECK(nearly_equal(detections[0].x1, 1.0F));
    CHECK(nearly_equal(detections[0].y1, 1.0F));
    CHECK(nearly_equal(detections[1].confidence, 0.70F));
    CHECK(nearly_equal(detections[1].x0, 75.0F / 2.08F / 200.0F));
    CHECK(nearly_equal(detections[1].y0, 183.0F / 2.08F / 100.0F - 0.5F));

    hand_postprocess_workspace_destroy(&workspace);
    free(output);
    return 0;
}

static int test_invalid_output_layout(void)
{
    const size_t candidate_count = 2U;
    const size_t channel_stride = candidate_count * sizeof(float);
    const size_t output_bytes = HAND_MODEL_OUTPUT_CHANNELS * channel_stride;
    float *output = (float *)calloc(1U, output_bytes);
    hand_letterbox_transform transform;
    hand_postprocess_config config;
    hand_postprocess_workspace workspace = {0};
    hand_detection detections[1] = {0};
    size_t detection_count = 0U;
    char error[128] = {0};

    CHECK(output != NULL);
    CHECK(hand_letterbox_init(100U, 100U, &transform) == 0);
    hand_postprocess_config_init_defaults(&config);
    CHECK(hand_postprocess_workspace_init(&workspace, candidate_count) == 0);
    CHECK(hand_postprocess_decode(output,
                                  output_bytes - 1U,
                                  channel_stride,
                                  candidate_count,
                                  &transform,
                                  0,
                                  &config,
                                  &workspace,
                                  detections,
                                  1U,
                                  &detection_count,
                                  error,
                                  sizeof(error)) != 0);
    CHECK(detection_count == 0U);
    CHECK(strstr(error, "layout") != NULL);
    hand_postprocess_workspace_destroy(&workspace);
    free(output);
    return 0;
}

int main(void)
{
    CHECK(test_decode_nms_and_letterbox() == 0);
    CHECK(test_invalid_output_layout() == 0);
    puts("postprocess tests passed");
    return 0;
}
