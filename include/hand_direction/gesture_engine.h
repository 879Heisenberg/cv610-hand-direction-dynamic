#ifndef HAND_DIRECTION_GESTURE_ENGINE_H
#define HAND_DIRECTION_GESTURE_ENGINE_H

#include "hand_direction/direction.h"
#include "hand_direction/tracker.h"
#include "hand_direction/types.h"

#include <stddef.h>
#include <stdint.h>

typedef enum {
    HAND_ENGINE_WAITING_FOR_HAND = 0,
    HAND_ENGINE_WAITING_FOR_ACTIVATION,
    HAND_ENGINE_CAPTURING,
    HAND_ENGINE_COOLDOWN
} hand_engine_state;

typedef struct {
    hand_tracker_config tracker;
    hand_direction_config direction;
    int activation_class_id;
    float activation_confidence;
    uint32_t activation_confirm_frames;
    float roi_x_min;
    float roi_x_max;
    float roi_y_min;
    float roi_y_max;
    float roi_hysteresis;
    uint64_t lost_reset_ms;
    uint64_t cooldown_ms;
    uint64_t event_animation_ms;
} hand_gesture_engine_config;

typedef struct {
    hand_engine_state state;
    int has_center;
    hand_point center;
    int has_tracked_box;
    hand_detection tracked_box;
    int roi_inside;
    uint64_t cooldown_remaining_ms;
    uint64_t cooldown_total_ms;
    float roi_x_min;
    float roi_x_max;
    float roi_y_min;
    float roi_y_max;
    int event_visible;
    hand_gesture_event event;
} hand_gesture_snapshot;

typedef struct {
    hand_engine_state state;
    int roi_inside;
    int has_raw_center;
    hand_point raw_center;
    int has_center;
    hand_point center;
    float activation_confidence;
    float roi_x_min;
    float roi_x_max;
    float roi_y_min;
    float roi_y_max;
    hand_direction_diagnostics direction;
} hand_gesture_diagnostics;

typedef struct {
    hand_gesture_engine_config config;
    hand_tracker tracker;
    hand_direction_recognizer recognizer;
    hand_engine_state state;
    uint64_t last_hand_ms;
    uint64_t last_event_ms;
    uint64_t cooldown_start_ms;
    int has_last_hand;
    int has_center;
    hand_point center;
    int has_raw_center;
    hand_point raw_center;
    float smoothed_confidence;
    uint32_t activation_streak;
    int has_pending_activation;
    hand_point pending_activation_point;
    int roi_inside;
    int has_last_event;
    int has_cooldown;
    hand_gesture_event last_event;
    hand_direction_diagnostics last_direction_diagnostics;
} hand_gesture_engine;

void hand_gesture_engine_config_init_defaults(hand_gesture_engine_config *config);
void hand_gesture_engine_init(hand_gesture_engine *engine,
                              const hand_gesture_engine_config *config);
void hand_gesture_engine_reset(hand_gesture_engine *engine);

int hand_gesture_engine_update(hand_gesture_engine *engine,
                               const hand_detection *detections,
                               size_t detection_count,
                               uint64_t timestamp_ms,
                               hand_gesture_event *event);

void hand_gesture_engine_snapshot(const hand_gesture_engine *engine,
                                  uint64_t timestamp_ms,
                                  hand_gesture_snapshot *snapshot);

const char *hand_gesture_engine_state_name(hand_engine_state state);
void hand_gesture_engine_get_diagnostics(const hand_gesture_engine *engine,
                                         hand_gesture_diagnostics *diagnostics);

#endif
