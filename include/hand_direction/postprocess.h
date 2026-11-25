#ifndef HAND_DIRECTION_POSTPROCESS_H
#define HAND_DIRECTION_POSTPROCESS_H

#include "hand_direction/preprocess.h"
#include "hand_direction/types.h"

#include <stddef.h>

#define HAND_MODEL_OUTPUT_CHANNELS 5U
#define HAND_MODEL_OUTPUT_CANDIDATES 3549U
#define HAND_POSTPROCESS_DEFAULT_CONFIDENCE 0.25F
#define HAND_POSTPROCESS_DEFAULT_NMS_IOU 0.45F
#define HAND_POSTPROCESS_DEBUG_TOP_COUNT 3U

typedef struct {
    float confidence_threshold;
    float nms_iou_threshold;
} hand_postprocess_config;

typedef struct {
    float score;
    float x0;
    float y0;
    float x1;
    float y1;
} hand_postprocess_candidate;

typedef struct {
    hand_postprocess_candidate *candidates;
    size_t capacity;
} hand_postprocess_workspace;

typedef struct {
    size_t candidate_count;
    size_t valid_score_count;
    float min_score;
    float max_score;
    double mean_score;
    size_t top_count;
    hand_postprocess_candidate top[HAND_POSTPROCESS_DEBUG_TOP_COUNT];
} hand_postprocess_score_diagnostic;

void hand_postprocess_config_init_defaults(hand_postprocess_config *config);

int hand_postprocess_workspace_init(hand_postprocess_workspace *workspace,
                                    size_t candidate_capacity);
void hand_postprocess_workspace_destroy(hand_postprocess_workspace *workspace);

int hand_postprocess_inspect_scores(
    const float *output,
    size_t output_bytes,
    size_t channel_stride_bytes,
    size_t candidate_count,
    const hand_letterbox_transform *transform,
    hand_postprocess_score_diagnostic *diagnostic,
    char *error_message,
    size_t error_message_size);

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
                            size_t error_message_size);

#endif
