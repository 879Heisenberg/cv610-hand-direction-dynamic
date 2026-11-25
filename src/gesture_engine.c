#include "hand_direction/gesture_engine.h"

#include <math.h>
#include <string.h>

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static hand_point track_center(const hand_track *track)
{
    hand_point point;

    point.x = 0.5F * (track->smoothed.x0 + track->smoothed.x1);
    point.y = 0.5F * (track->smoothed.y0 + track->smoothed.y1);
    point.timestamp_ms = track->timestamp_ms;
    return point;
}

static hand_point raw_track_center(const hand_track *track)
{
    hand_point point;

    point.x = 0.5F * (track->raw_detection.x0 + track->raw_detection.x1);
    point.y = 0.5F * (track->raw_detection.y0 + track->raw_detection.y1);
    point.timestamp_ms = track->timestamp_ms;
    return point;
}

static void sanitize_roi_axis(float *minimum, float *maximum)
{
    float low = clamp_float(*minimum, 0.0F, 1.0F);
    float high = clamp_float(*maximum, 0.0F, 1.0F);

    if (!isfinite(low) || !isfinite(high) || low >= high) {
        low = 0.20F;
        high = 0.80F;
    }
    *minimum = low;
    *maximum = high;
}

static int point_in_roi(const hand_gesture_engine *engine,
                        hand_point point,
                        float margin)
{
    return point.x >= engine->config.roi_x_min - margin &&
           point.x <= engine->config.roi_x_max + margin &&
           point.y >= engine->config.roi_y_min - margin &&
           point.y <= engine->config.roi_y_max + margin;
}

static int update_roi_presence(hand_gesture_engine *engine, hand_point point)
{
    const int strictly_inside = point_in_roi(engine, point, 0.0F);
    const float hysteresis = engine->config.roi_hysteresis;

    if (engine->roi_inside) {
        if (!point_in_roi(engine, point, hysteresis)) {
            engine->roi_inside = 0;
        }
    } else if (strictly_inside) {
        engine->roi_inside = 1;
    }

    /* Gesture samples always use the strict rectangle. Hysteresis only
     * stabilizes the ROI state while the tracker jitters around a boundary. */
    return strictly_inside;
}

static void reset_tracking_state(hand_gesture_engine *engine)
{
    hand_tracker_reset(&engine->tracker);
    hand_direction_reset(&engine->recognizer);
    engine->state = HAND_ENGINE_WAITING_FOR_HAND;
    engine->last_hand_ms = 0U;
    engine->has_last_hand = 0;
    engine->has_center = 0;
    engine->has_raw_center = 0;
    engine->smoothed_confidence = 0.0F;
    engine->activation_streak = 0U;
    engine->has_pending_activation = 0;
    engine->roi_inside = 0;
    (void)memset(&engine->center, 0, sizeof(engine->center));
    (void)memset(&engine->raw_center, 0, sizeof(engine->raw_center));
    (void)memset(&engine->pending_activation_point, 0,
                 sizeof(engine->pending_activation_point));
}

void hand_gesture_engine_config_init_defaults(hand_gesture_engine_config *config)
{
    if (config == NULL) {
        return;
    }

    hand_tracker_config_init_defaults(&config->tracker);
    hand_direction_config_init_defaults(&config->direction);
    config->activation_class_id = 0;
    config->activation_confidence = 0.60F;
    config->activation_confirm_frames = 2U;
    config->roi_x_min = 0.20F;
    config->roi_x_max = 0.80F;
    config->roi_y_min = 0.20F;
    config->roi_y_max = 0.80F;
    config->roi_hysteresis = 0.05F;
    config->lost_reset_ms = 250U;
    config->cooldown_ms = 2000U;
    config->event_animation_ms = 1000U;
}

void hand_gesture_engine_reset(hand_gesture_engine *engine)
{
    if (engine == NULL) {
        return;
    }

    reset_tracking_state(engine);
    engine->last_event_ms = 0U;
    engine->cooldown_start_ms = 0U;
    engine->has_last_event = 0;
    engine->has_cooldown = 0;
    (void)memset(&engine->last_event, 0, sizeof(engine->last_event));
    hand_direction_get_diagnostics(NULL, &engine->last_direction_diagnostics);
}

void hand_gesture_engine_init(hand_gesture_engine *engine,
                              const hand_gesture_engine_config *config)
{
    if (engine == NULL) {
        return;
    }

    if (config != NULL) {
        engine->config = *config;
    } else {
        hand_gesture_engine_config_init_defaults(&engine->config);
    }

    engine->config.activation_confidence =
        clamp_float(engine->config.activation_confidence, 0.0F, 1.0F);
    if (engine->config.activation_confirm_frames == 0U) {
        engine->config.activation_confirm_frames = 1U;
    }
    sanitize_roi_axis(&engine->config.roi_x_min, &engine->config.roi_x_max);
    sanitize_roi_axis(&engine->config.roi_y_min, &engine->config.roi_y_max);
    engine->config.roi_hysteresis =
        clamp_float(engine->config.roi_hysteresis, 0.0F, 0.25F);
    if (engine->config.lost_reset_ms == 0U) {
        engine->config.lost_reset_ms = 250U;
    }
    if (engine->config.cooldown_ms == 0U) {
        engine->config.cooldown_ms = 2000U;
    }
    if (engine->config.event_animation_ms == 0U) {
        engine->config.event_animation_ms = 1000U;
    }

    hand_tracker_init(&engine->tracker, &engine->config.tracker);
    hand_direction_init(&engine->recognizer, &engine->config.direction);
    hand_gesture_engine_reset(engine);
}

static int lost_hand_expired(const hand_gesture_engine *engine,
                             uint64_t timestamp_ms)
{
    return engine->has_last_hand && timestamp_ms >= engine->last_hand_ms &&
           timestamp_ms - engine->last_hand_ms > engine->config.lost_reset_ms;
}

static int animation_active(const hand_gesture_engine *engine,
                            uint64_t timestamp_ms)
{
    return engine->has_last_event &&
           timestamp_ms >= engine->last_event_ms &&
           timestamp_ms - engine->last_event_ms <= engine->config.event_animation_ms;
}

static int cooldown_active(const hand_gesture_engine *engine,
                           uint64_t timestamp_ms)
{
    return engine->has_cooldown &&
           timestamp_ms >= engine->cooldown_start_ms &&
           timestamp_ms - engine->cooldown_start_ms < engine->config.cooldown_ms;
}

int hand_gesture_engine_update(hand_gesture_engine *engine,
                               const hand_detection *detections,
                               size_t detection_count,
                               uint64_t timestamp_ms,
                               hand_gesture_event *event)
{
    hand_track track;

    if (engine == NULL || event == NULL ||
        (detection_count > 0U && detections == NULL)) {
        return -1;
    }

    (void)memset(event, 0, sizeof(*event));
    hand_direction_get_diagnostics(NULL, &engine->last_direction_diagnostics);

    if ((engine->has_last_hand && timestamp_ms < engine->last_hand_ms) ||
        (engine->has_cooldown && timestamp_ms < engine->cooldown_start_ms) ||
        (engine->has_last_event && timestamp_ms < engine->last_event_ms)) {
        hand_gesture_engine_reset(engine);
    }

    if (engine->has_last_event && !animation_active(engine, timestamp_ms)) {
        engine->has_last_event = 0;
        (void)memset(&engine->last_event, 0, sizeof(engine->last_event));
    }
    if (engine->has_cooldown && !cooldown_active(engine, timestamp_ms)) {
        engine->has_cooldown = 0;
        engine->cooldown_start_ms = 0U;
    }

    if (hand_tracker_update(&engine->tracker,
                            detections,
                            detection_count,
                            timestamp_ms,
                            &track) != 0) {
        return -1;
    }

    if (!track.valid) {
        engine->activation_streak = 0U;
        engine->has_pending_activation = 0;
        (void)memset(&engine->pending_activation_point, 0,
                     sizeof(engine->pending_activation_point));
        if (lost_hand_expired(engine, timestamp_ms)) {
            /* Brief occlusions <= lost_reset_ms preserve tracker and trajectory;
             * only expired hand loss clears tracking while keeping active cooldown. */
            reset_tracking_state(engine);
            engine->state = cooldown_active(engine, timestamp_ms)
                                ? HAND_ENGINE_COOLDOWN
                                : HAND_ENGINE_WAITING_FOR_HAND;
        } else if (engine->state == HAND_ENGINE_COOLDOWN &&
                   !cooldown_active(engine, timestamp_ms)) {
            engine->state = engine->has_last_hand
                                ? HAND_ENGINE_WAITING_FOR_ACTIVATION
                                : HAND_ENGINE_WAITING_FOR_HAND;
        }
        return 0;
    }

    engine->last_hand_ms = timestamp_ms;
    engine->has_last_hand = 1;
    engine->center = track_center(&track);
    engine->has_center = 1;
    engine->raw_center = raw_track_center(&track);
    engine->has_raw_center = 1;
    engine->smoothed_confidence = track.smoothed.confidence;

    {
        const int inside_roi = update_roi_presence(engine, engine->raw_center);

        if (engine->state == HAND_ENGINE_WAITING_FOR_HAND) {
            engine->state = HAND_ENGINE_WAITING_FOR_ACTIVATION;
        }

        if (cooldown_active(engine, timestamp_ms)) {
            engine->state = HAND_ENGINE_COOLDOWN;
            engine->activation_streak = 0U;
            engine->has_pending_activation = 0;
            (void)memset(&engine->pending_activation_point, 0,
                         sizeof(engine->pending_activation_point));
            hand_direction_reset(&engine->recognizer);
            return 0;
        }

        if (engine->state == HAND_ENGINE_COOLDOWN) {
            engine->state = HAND_ENGINE_WAITING_FOR_ACTIVATION;
            engine->activation_streak = 0U;
            engine->has_pending_activation = 0;
            (void)memset(&engine->pending_activation_point, 0,
                         sizeof(engine->pending_activation_point));
            hand_direction_reset(&engine->recognizer);
            return 0;
        }

        if (!inside_roi) {
            engine->activation_streak = 0U;
            engine->has_pending_activation = 0;
            (void)memset(&engine->pending_activation_point, 0,
                         sizeof(engine->pending_activation_point));
            if (engine->state == HAND_ENGINE_CAPTURING) {
                hand_direction_reset(&engine->recognizer);
                engine->state = HAND_ENGINE_WAITING_FOR_ACTIVATION;
            }
            return 0;
        }
    }

    if (engine->state == HAND_ENGINE_WAITING_FOR_ACTIVATION) {
        if (track.smoothed.class_id == engine->config.activation_class_id &&
            track.smoothed.confidence >= engine->config.activation_confidence) {
            ++engine->activation_streak;
            if (engine->activation_streak >= engine->config.activation_confirm_frames) {
                engine->state = HAND_ENGINE_CAPTURING;
                engine->activation_streak = 0U;
                hand_direction_reset(&engine->recognizer);
                if (engine->has_pending_activation &&
                    engine->center.timestamp_ms >
                        engine->pending_activation_point.timestamp_ms) {
                    hand_gesture_event ignored_event;
                    if (hand_direction_update(&engine->recognizer,
                                              engine->pending_activation_point,
                                              &ignored_event) < 0) {
                        engine->has_pending_activation = 0;
                        return -1;
                    }
                }
                engine->has_pending_activation = 0;
                (void)memset(&engine->pending_activation_point, 0,
                             sizeof(engine->pending_activation_point));
            } else {
                if (!engine->has_pending_activation) {
                    engine->pending_activation_point = engine->center;
                    engine->has_pending_activation = 1;
                }
                return 0;
            }
        } else {
            engine->activation_streak = 0U;
            engine->has_pending_activation = 0;
            (void)memset(&engine->pending_activation_point, 0,
                         sizeof(engine->pending_activation_point));
            return 0;
        }
    }

    if (engine->state == HAND_ENGINE_CAPTURING) {
        hand_point point = engine->center;
        const int result = hand_direction_update(&engine->recognizer, point, event);

        hand_direction_get_diagnostics(&engine->recognizer,
                                       &engine->last_direction_diagnostics);
        if (result < 0) {
            return -1;
        }
        if (result > 0) {
            engine->state = HAND_ENGINE_COOLDOWN;
            engine->has_cooldown = 1;
            engine->cooldown_start_ms = timestamp_ms;
            engine->last_event_ms = timestamp_ms;
            engine->last_event = *event;
            engine->has_last_event = 1;
            engine->activation_streak = 0U;
            hand_direction_reset(&engine->recognizer);
        }
    }

    return 0;
}

void hand_gesture_engine_snapshot(const hand_gesture_engine *engine,
                                  uint64_t timestamp_ms,
                                  hand_gesture_snapshot *snapshot)
{
    int has_active_hand;

    if (snapshot == NULL) {
        return;
    }
    (void)memset(snapshot, 0, sizeof(*snapshot));
    if (engine == NULL) {
        return;
    }

    has_active_hand = engine->has_center &&
                      timestamp_ms >= engine->last_hand_ms &&
                      timestamp_ms - engine->last_hand_ms <= engine->config.lost_reset_ms;

    snapshot->state = engine->state;
    snapshot->cooldown_total_ms = engine->config.cooldown_ms;
    snapshot->roi_x_min = engine->config.roi_x_min;
    snapshot->roi_x_max = engine->config.roi_x_max;
    snapshot->roi_y_min = engine->config.roi_y_min;
    snapshot->roi_y_max = engine->config.roi_y_max;

    if (cooldown_active(engine, timestamp_ms)) {
        snapshot->state = HAND_ENGINE_COOLDOWN;
        snapshot->cooldown_remaining_ms =
            engine->config.cooldown_ms - (timestamp_ms - engine->cooldown_start_ms);
    } else if (engine->state == HAND_ENGINE_COOLDOWN) {
        snapshot->state = has_active_hand
                              ? HAND_ENGINE_WAITING_FOR_ACTIVATION
                              : HAND_ENGINE_WAITING_FOR_HAND;
        snapshot->cooldown_remaining_ms = 0U;
    }

    if (has_active_hand) {
        snapshot->has_center = 1;
        snapshot->center = engine->center;
        snapshot->roi_inside = engine->roi_inside;
        if (engine->tracker.has_track && engine->tracker.current.valid) {
            snapshot->has_tracked_box = 1;
            snapshot->tracked_box = engine->tracker.current.smoothed;
        }
    }

    if (engine->has_last_event && timestamp_ms >= engine->last_event_ms &&
        timestamp_ms - engine->last_event_ms <= engine->config.event_animation_ms) {
        snapshot->event_visible = 1;
        snapshot->event = engine->last_event;
    }
}

const char *hand_gesture_engine_state_name(hand_engine_state state)
{
    switch (state) {
    case HAND_ENGINE_WAITING_FOR_HAND:
        return "WAITING_FOR_HAND";
    case HAND_ENGINE_WAITING_FOR_ACTIVATION:
        return "WAITING_FOR_ACTIVATION";
    case HAND_ENGINE_CAPTURING:
        return "CAPTURING";
    case HAND_ENGINE_COOLDOWN:
        return "COOLDOWN";
    default:
        return "UNKNOWN";
    }
}

void hand_gesture_engine_get_diagnostics(const hand_gesture_engine *engine,
                                         hand_gesture_diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }
    (void)memset(diagnostics, 0, sizeof(*diagnostics));
    if (engine == NULL) {
        hand_direction_get_diagnostics(NULL, &diagnostics->direction);
        return;
    }

    diagnostics->state = engine->state;
    diagnostics->roi_inside = engine->roi_inside;
    diagnostics->has_raw_center = engine->has_raw_center;
    diagnostics->raw_center = engine->raw_center;
    diagnostics->has_center = engine->has_center;
    diagnostics->center = engine->center;
    diagnostics->activation_confidence = engine->smoothed_confidence;
    diagnostics->roi_x_min = engine->config.roi_x_min;
    diagnostics->roi_x_max = engine->config.roi_x_max;
    diagnostics->roi_y_min = engine->config.roi_y_min;
    diagnostics->roi_y_max = engine->config.roi_y_max;
    diagnostics->direction = engine->last_direction_diagnostics;
}
