#ifndef HAND_DIRECTION_TRACKER_H
#define HAND_DIRECTION_TRACKER_H

#include "hand_direction/types.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    float ema_alpha;
    uint64_t lost_timeout_ms;
    float max_speed_norm_per_second;
    float gate_margin_norm;
    float max_step_norm;
} hand_tracker_config;

typedef struct {
    int valid;
    hand_detection smoothed;
    hand_detection raw_detection;
    uint64_t timestamp_ms;
} hand_track;

typedef struct {
    hand_tracker_config config;
    hand_track current;
    int has_track;
} hand_tracker;

void hand_tracker_config_init_defaults(hand_tracker_config *config);
void hand_tracker_init(hand_tracker *tracker, const hand_tracker_config *config);
void hand_tracker_reset(hand_tracker *tracker);

int hand_tracker_update(hand_tracker *tracker,
                        const hand_detection *detections,
                        size_t detection_count,
                        uint64_t timestamp_ms,
                        hand_track *track);

#endif
