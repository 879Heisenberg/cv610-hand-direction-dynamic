#include "hand_direction/postprocess.h"

#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HAND_FP32_BYTES sizeof(float)
#define HAND_IOU_EPSILON 1e-9

static void postprocess_message(char *buffer, size_t size, const char *format, ...)
{
    va_list arguments;

    if (buffer == NULL || size == 0U) {
        return;
    }

    va_start(arguments, format);
    (void)vsnprintf(buffer, size, format, arguments);
    va_end(arguments);
}

static int transform_is_valid(const hand_letterbox_transform *transform)
{
    hand_letterbox_transform expected;

    return transform != NULL &&
           hand_letterbox_init(transform->source_width,
                               transform->source_height,
                               &expected) == 0 &&
           transform->resized_width == expected.resized_width &&
           transform->resized_height == expected.resized_height &&
           transform->pad_left == expected.pad_left &&
           transform->pad_top == expected.pad_top &&
           transform->scale == expected.scale;
}

static int output_layout_is_valid(const float *output,
                                  size_t output_bytes,
                                  size_t channel_stride_bytes,
                                  size_t candidate_count)
{
    size_t candidate_bytes;
    size_t last_channel_bytes;

    if (output == NULL || (uintptr_t)output % _Alignof(float) != 0U ||
        candidate_count == 0U || candidate_count > SIZE_MAX / HAND_FP32_BYTES ||
        channel_stride_bytes < sizeof(float) ||
        channel_stride_bytes % sizeof(float) != 0U) {
        return 0;
    }

    candidate_bytes = candidate_count * HAND_FP32_BYTES;
    if (channel_stride_bytes < candidate_bytes ||
        (HAND_MODEL_OUTPUT_CHANNELS - 1U) >
            SIZE_MAX / channel_stride_bytes) {
        return 0;
    }

    last_channel_bytes = (HAND_MODEL_OUTPUT_CHANNELS - 1U) * channel_stride_bytes;
    return output_bytes >= last_channel_bytes &&
           candidate_bytes <= output_bytes - last_channel_bytes;
}

static float output_value(const float *output,
                          size_t channel_stride_bytes,
                          size_t channel,
                          size_t candidate)
{
    const size_t channel_offset = channel * (channel_stride_bytes / sizeof(float));
    return output[channel_offset + candidate];
}

static int candidate_compare(const void *left, const void *right)
{
    const hand_postprocess_candidate *a = (const hand_postprocess_candidate *)left;
    const hand_postprocess_candidate *b = (const hand_postprocess_candidate *)right;

    if (a->score > b->score) {
        return -1;
    }
    if (a->score < b->score) {
        return 1;
    }
    if (a->y0 < b->y0) {
        return -1;
    }
    if (a->y0 > b->y0) {
        return 1;
    }
    if (a->x0 < b->x0) {
        return -1;
    }
    if (a->x0 > b->x0) {
        return 1;
    }
    return 0;
}

static double candidate_area(const hand_postprocess_candidate *candidate)
{
    return ((double)candidate->x1 - (double)candidate->x0) *
           ((double)candidate->y1 - (double)candidate->y0);
}

static double candidate_iou(const hand_postprocess_candidate *a,
                            const hand_postprocess_candidate *b)
{
    const double left = fmax((double)a->x0, (double)b->x0);
    const double top = fmax((double)a->y0, (double)b->y0);
    const double right = fmin((double)a->x1, (double)b->x1);
    const double bottom = fmin((double)a->y1, (double)b->y1);
    const double overlap_width = fmax(0.0, right - left);
    const double overlap_height = fmax(0.0, bottom - top);
    const double intersection = overlap_width * overlap_height;
    const double denominator = candidate_area(a) + candidate_area(b) -
                               intersection + HAND_IOU_EPSILON;

    return denominator > 0.0 ? intersection / denominator : 0.0;
}

static int candidate_is_suppressed(const hand_postprocess_candidate *candidate,
                                   const hand_postprocess_candidate *kept,
                                   size_t kept_count,
                                   float nms_iou_threshold)
{
    size_t index;

    for (index = 0U; index < kept_count; ++index) {
        if (candidate_iou(candidate, &kept[index]) > (double)nms_iou_threshold) {
            return 1;
        }
    }
    return 0;
}

static float map_coordinate(float coordinate,
                            uint32_t padding,
                            float scale,
                            uint32_t source_extent)
{
    const double mapped = ((double)coordinate - (double)padding) / (double)scale;
    const double clipped = fmin((double)source_extent,
                                fmax(0.0, mapped));
    return (float)(clipped / (double)source_extent);
}

void hand_postprocess_config_init_defaults(hand_postprocess_config *config)
{
    if (config == NULL) {
        return;
    }

    config->confidence_threshold = HAND_POSTPROCESS_DEFAULT_CONFIDENCE;
    config->nms_iou_threshold = HAND_POSTPROCESS_DEFAULT_NMS_IOU;
}

int hand_postprocess_workspace_init(hand_postprocess_workspace *workspace,
                                    size_t candidate_capacity)
{
    hand_postprocess_candidate *candidates;

    if (workspace == NULL || candidate_capacity == 0U ||
        candidate_capacity > SIZE_MAX / sizeof(*candidates)) {
        return -1;
    }

    candidates = (hand_postprocess_candidate *)calloc(candidate_capacity,
                                                       sizeof(*candidates));
    if (candidates == NULL) {
        return -1;
    }

    workspace->candidates = candidates;
    workspace->capacity = candidate_capacity;
    return 0;
}

void hand_postprocess_workspace_destroy(hand_postprocess_workspace *workspace)
{
    if (workspace == NULL) {
        return;
    }
    free(workspace->candidates);
    workspace->candidates = NULL;
    workspace->capacity = 0U;
}

int hand_postprocess_inspect_scores(
    const float *output,
    size_t output_bytes,
    size_t channel_stride_bytes,
    size_t candidate_count,
    const hand_letterbox_transform *transform,
    hand_postprocess_score_diagnostic *diagnostic,
    char *error_message,
    size_t error_message_size)
{
    size_t candidate_index;
    double score_sum = 0.0;

    if (diagnostic == NULL) {
        postprocess_message(error_message,
                            error_message_size,
                            "missing score diagnostic output");
        return -1;
    }
    memset(diagnostic, 0, sizeof(*diagnostic));
    diagnostic->candidate_count = candidate_count;

    if (!output_layout_is_valid(output,
                                output_bytes,
                                channel_stride_bytes,
                                candidate_count)) {
        postprocess_message(error_message,
                            error_message_size,
                            "output tensor layout does not fit [5,%zu] FP32",
                            candidate_count);
        return -1;
    }
    if (!transform_is_valid(transform)) {
        postprocess_message(error_message,
                            error_message_size,
                            "invalid letterbox transform");
        return -1;
    }

    for (candidate_index = 0U; candidate_index < candidate_count; ++candidate_index) {
        const float score = output_value(output,
                                         channel_stride_bytes,
                                         4U,
                                         candidate_index);
        float center_x;
        float center_y;
        float width;
        float height;
        hand_postprocess_candidate candidate;
        size_t insert_index;

        if (!isfinite(score)) {
            continue;
        }

        if (diagnostic->valid_score_count == 0U) {
            diagnostic->min_score = score;
            diagnostic->max_score = score;
        } else {
            diagnostic->min_score = fminf(diagnostic->min_score, score);
            diagnostic->max_score = fmaxf(diagnostic->max_score, score);
        }
        ++diagnostic->valid_score_count;
        score_sum += (double)score;

        center_x = output_value(output, channel_stride_bytes, 0U, candidate_index);
        center_y = output_value(output, channel_stride_bytes, 1U, candidate_index);
        width = output_value(output, channel_stride_bytes, 2U, candidate_index);
        height = output_value(output, channel_stride_bytes, 3U, candidate_index);
        if (!isfinite(center_x) || !isfinite(center_y) ||
            !isfinite(width) || !isfinite(height) || width <= 0.0F || height <= 0.0F) {
            continue;
        }

        candidate.score = score;
        candidate.x0 = center_x - width * 0.5F;
        candidate.y0 = center_y - height * 0.5F;
        candidate.x1 = center_x + width * 0.5F;
        candidate.y1 = center_y + height * 0.5F;
        if (candidate.x1 <= candidate.x0 || candidate.y1 <= candidate.y0) {
            continue;
        }

        insert_index = 0U;
        while (insert_index < diagnostic->top_count &&
               diagnostic->top[insert_index].score >= candidate.score) {
            ++insert_index;
        }
        if (insert_index >= HAND_POSTPROCESS_DEBUG_TOP_COUNT) {
            continue;
        }

        if (diagnostic->top_count < HAND_POSTPROCESS_DEBUG_TOP_COUNT) {
            ++diagnostic->top_count;
        }
        {
            size_t move_index = diagnostic->top_count - 1U;

            while (move_index > insert_index) {
                diagnostic->top[move_index] = diagnostic->top[move_index - 1U];
                --move_index;
            }
        }
        diagnostic->top[insert_index] = candidate;
    }

    if (diagnostic->valid_score_count != 0U) {
        diagnostic->mean_score =
            score_sum / (double)diagnostic->valid_score_count;
    }
    postprocess_message(error_message, error_message_size, "");
    return 0;
}

int hand_postprocess_decode(const float *output,
                            size_t output_bytes,
                            size_t channel_stride_bytes,
                            size_t candidate_count,
                            const hand_letterbox_transform *transform,
                            int class_id,
                            const hand_postprocess_config *config,
                            hand_postprocess_workspace *workspace,
                            hand_detection *detections,
                            size_t detection_capacity,
                            size_t *detection_count,
                            char *error_message,
                            size_t error_message_size)
{
    size_t candidate_total = 0U;
    size_t kept_total = 0U;
    size_t candidate_index;

    if (detection_count != NULL) {
        *detection_count = 0U;
    }
    if (detections == NULL || detection_count == NULL || detection_capacity == 0U ||
        config == NULL || workspace == NULL || workspace->candidates == NULL ||
        workspace->capacity < candidate_count || class_id < 0) {
        postprocess_message(error_message,
                            error_message_size,
                            "invalid postprocess arguments or workspace");
        return -1;
    }
    if (!isfinite(config->confidence_threshold) ||
        !isfinite(config->nms_iou_threshold) ||
        config->confidence_threshold < 0.0F || config->confidence_threshold > 1.0F ||
        config->nms_iou_threshold < 0.0F || config->nms_iou_threshold > 1.0F) {
        postprocess_message(error_message,
                            error_message_size,
                            "invalid confidence or NMS threshold");
        return -1;
    }
    if (!output_layout_is_valid(output,
                                output_bytes,
                                channel_stride_bytes,
                                candidate_count)) {
        postprocess_message(error_message,
                            error_message_size,
                            "output tensor layout does not fit [5,%zu] FP32",
                            candidate_count);
        return -1;
    }
    if (!transform_is_valid(transform)) {
        postprocess_message(error_message,
                            error_message_size,
                            "invalid letterbox transform");
        return -1;
    }

    for (candidate_index = 0U; candidate_index < candidate_count; ++candidate_index) {
        const float score = output_value(output,
                                         channel_stride_bytes,
                                         4U,
                                         candidate_index);
        float center_x;
        float center_y;
        float width;
        float height;
        hand_postprocess_candidate *candidate;

        if (!isfinite(score) || score < 0.0F || score > 1.0F ||
            score < config->confidence_threshold) {
            continue;
        }

        center_x = output_value(output, channel_stride_bytes, 0U, candidate_index);
        center_y = output_value(output, channel_stride_bytes, 1U, candidate_index);
        width = output_value(output, channel_stride_bytes, 2U, candidate_index);
        height = output_value(output, channel_stride_bytes, 3U, candidate_index);
        if (!isfinite(center_x) || !isfinite(center_y) ||
            !isfinite(width) || !isfinite(height) || width <= 0.0F || height <= 0.0F) {
            continue;
        }

        candidate = &workspace->candidates[candidate_total];
        candidate->score = score;
        candidate->x0 = center_x - width * 0.5F;
        candidate->y0 = center_y - height * 0.5F;
        candidate->x1 = center_x + width * 0.5F;
        candidate->y1 = center_y + height * 0.5F;
        if (candidate->x1 <= candidate->x0 || candidate->y1 <= candidate->y0) {
            continue;
        }
        ++candidate_total;
    }

    qsort(workspace->candidates, candidate_total,
          sizeof(workspace->candidates[0]), candidate_compare);

    for (candidate_index = 0U; candidate_index < candidate_total; ++candidate_index) {
        hand_postprocess_candidate *candidate = &workspace->candidates[candidate_index];

        if (!candidate_is_suppressed(candidate,
                                     workspace->candidates,
                                     kept_total,
                                     config->nms_iou_threshold)) {
            if (kept_total != candidate_index) {
                workspace->candidates[kept_total] = *candidate;
            }
            ++kept_total;
        }
    }

    {
        const size_t publish_count = kept_total < detection_capacity
                                         ? kept_total
                                         : detection_capacity;
        size_t output_index;

        for (output_index = 0U; output_index < publish_count; ++output_index) {
            const hand_postprocess_candidate *candidate =
                &workspace->candidates[output_index];
            hand_detection detection;

            detection.x0 = map_coordinate(candidate->x0,
                                          transform->pad_left,
                                          transform->scale,
                                          transform->source_width);
            detection.y0 = map_coordinate(candidate->y0,
                                          transform->pad_top,
                                          transform->scale,
                                          transform->source_height);
            detection.x1 = map_coordinate(candidate->x1,
                                          transform->pad_left,
                                          transform->scale,
                                          transform->source_width);
            detection.y1 = map_coordinate(candidate->y1,
                                          transform->pad_top,
                                          transform->scale,
                                          transform->source_height);
            if (detection.x1 <= detection.x0 || detection.y1 <= detection.y0) {
                continue;
            }
            detection.confidence = candidate->score;
            detection.class_id = class_id;
            detections[*detection_count] = detection;
            ++(*detection_count);
        }
    }

    postprocess_message(error_message, error_message_size, "");
    return 0;
}
