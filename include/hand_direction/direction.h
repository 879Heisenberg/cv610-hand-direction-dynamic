#ifndef HAND_DIRECTION_DIRECTION_H
#define HAND_DIRECTION_DIRECTION_H

#include "hand_direction/trajectory.h"
#include "hand_direction/types.h"

#include <stdint.h>

typedef struct {
    uint64_t window_ms;
    uint64_t max_gap_ms;
    uint32_t min_samples;
    uint32_t min_span_ms;
    float min_displacement;
    float min_speed_per_second;
    float min_linearity;
    float axis_ratio;
    float x_scale;
    float min_segment_consistency;
    float min_half_cosine;
    uint32_t min_moving_segments;
    uint32_t confirm_count;
} hand_direction_config;

typedef struct {
    size_t point_count;
    size_t active_point_count;
    uint64_t span_ms;
    float dx;
    float dy;
    float displacement;
    float path_length;
    float linearity;
    float axis_ratio;
    float segment_consistency;
    float half_cosine;
    uint32_t moving_segments;
    uint32_t consistent_segments;
    uint32_t candidate_streak;
    hand_direction_code candidate_direction;
    hand_direction_code emitted_direction;
    const char *reject_reason;
} hand_direction_diagnostics;

typedef struct {
    hand_direction_config config;
    hand_trajectory trajectory;
    hand_direction_code candidate_direction;
    uint32_t candidate_streak;
    hand_direction_diagnostics last_diagnostics;
} hand_direction_recognizer;

void hand_direction_config_init_defaults(hand_direction_config *config);
void hand_direction_init(hand_direction_recognizer *recognizer,
                         const hand_direction_config *config);
void hand_direction_reset(hand_direction_recognizer *recognizer);

int hand_direction_update(hand_direction_recognizer *recognizer,
                          hand_point point,
                          hand_gesture_event *event);

void hand_direction_get_diagnostics(const hand_direction_recognizer *recognizer,
                                    hand_direction_diagnostics *diagnostics);

const char *hand_direction_name(hand_direction_code direction);

#endif
