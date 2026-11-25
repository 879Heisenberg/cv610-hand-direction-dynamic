#include "hand_direction/gesture_engine.h"
#include "hand_direction/trajectory.h"

#include <stdio.h>

#define CHECK(condition)                                                         \
    do {                                                                         \
        if (!(condition)) {                                                      \
            (void)fprintf(stderr, "CHECK failed at %s:%d: %s\n",               \
                          __FILE__, __LINE__, #condition);                       \
            return 1;                                                            \
        }                                                                        \
    } while (0)

static hand_detection make_detection_at(float center_x,
                                        float center_y,
                                        int class_id)
{
    hand_detection detection;

    detection.x0 = center_x - 0.05F;
    detection.y0 = center_y - 0.05F;
    detection.x1 = center_x + 0.05F;
    detection.y1 = center_y + 0.05F;
    detection.confidence = 0.92F;
    detection.class_id = class_id;
    return detection;
}

static hand_detection make_detection(float center_x, int class_id)
{
    return make_detection_at(center_x, 0.50F, class_id);
}

static int feed_detection(hand_gesture_engine *engine,
                          hand_detection detection,
                          uint64_t timestamp_ms,
                          hand_gesture_event *event)
{
    hand_gesture_event local_event;

    if (event == NULL) {
        event = &local_event;
    }
    return hand_gesture_engine_update(engine, &detection, 1U, timestamp_ms, event);
}

static int test_no_activation_with_wrong_class(void)
{
    hand_gesture_engine engine;
    unsigned int events = 0U;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 9U; ++index) {
        hand_gesture_event event;
        const float x = 0.20F + 0.07F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection(x, 1),
                             (uint64_t)index * 100U,
                             &event) == 0);
        if (event.valid) {
            ++events;
        }
    }
    CHECK(events == 0U);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    return 0;
}

static int test_hand_activation_and_right_swipe(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event = {0};
    hand_gesture_event last_event = {0};
    unsigned int events = 0U;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 9U; ++index) {
        const float x = 0.20F + 0.07F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection(x, 0),
                             (uint64_t)index * 100U,
                             &event) == 0);
        if (event.valid) {
            ++events;
            last_event = event;
        }
    }

    CHECK(events == 1U);
    CHECK(last_event.direction == HAND_DIRECTION_RIGHT);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);
    {
        hand_gesture_snapshot snapshot;
        hand_gesture_engine_snapshot(&engine, 900U, &snapshot);
        CHECK(snapshot.event_visible == 1);
        CHECK(snapshot.event.direction == HAND_DIRECTION_RIGHT);
    }
    return 0;
}

static int test_cooldown_blocks_duplicate_event(void)
{
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    unsigned int events = 0U;
    unsigned int index;

    hand_gesture_engine_config_init_defaults(&config);
    config.cooldown_ms = 5000U;
    hand_gesture_engine_init(&engine, &config);

    for (index = 0U; index < 16U; ++index) {
        const float x = index < 9U
                            ? 0.20F + 0.07F * (float)index
                            : (index % 2U == 0U ? 0.20F : 0.80F);
        hand_gesture_event event;

        CHECK(feed_detection(&engine,
                             make_detection(x, 0),
                             (uint64_t)index * 100U,
                             &event) == 0);
        if (event.valid) {
            ++events;
        }
    }

    CHECK(events == 1U);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);
    return 0;
}

static int test_default_cooldown_is_2000ms(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    uint64_t event_ms = 0U;
    int got_event = 0;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    CHECK(engine.config.cooldown_ms == 2000U);

    for (index = 0U; index < 12U; ++index) {
        const float x = 0.20F + 0.05F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection(x, 0),
                             (uint64_t)index * 100U,
                             &event) == 0);
        if (event.valid) {
            got_event = 1;
            event_ms = event.timestamp_ms;
            break;
        }
    }
    CHECK(got_event == 1);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);

    CHECK(feed_detection(&engine,
                         make_detection(0.44F, 0),
                         event_ms + 1999U,
                         &event) == 0);
    CHECK(event.valid == 0);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);

    CHECK(feed_detection(&engine,
                         make_detection(0.44F, 0),
                         event_ms + 2000U,
                         &event) == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    return 0;
}

static int test_detection_loss_resets_capture(void)
{
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    hand_gesture_event event;

    hand_gesture_engine_config_init_defaults(&config);
    config.lost_reset_ms = 300U;
    hand_gesture_engine_init(&engine, &config);

    CHECK(feed_detection(&engine, make_detection(0.20F, 0), 0U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(feed_detection(&engine, make_detection(0.22F, 0), 45U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_CAPTURING);

    CHECK(hand_gesture_engine_update(&engine, NULL, 0U, 400U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_HAND);
    CHECK(event.valid == 0);

    CHECK(feed_detection(&engine, make_detection(0.30F, 1), 500U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(event.valid == 0);
    return 0;
}

static int test_min_samples_and_animation_expiry(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    hand_gesture_snapshot snapshot;
    uint64_t event_ms = 0U;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    CHECK(engine.config.event_animation_ms == 1000U);

    for (index = 0U; index < 14U; ++index) {
        const uint64_t timestamp_ms = (uint64_t)index * 45U;
        const float x = 0.25F + 0.03F * (float)index;

        CHECK(feed_detection(&engine, make_detection(x, 0), timestamp_ms, &event) == 0);
        if (index < 6U) {
            CHECK(event.valid == 0);
        }
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
            event_ms = event.timestamp_ms;
            break;
        }
    }

    CHECK(event_ms != 0U);
    hand_gesture_engine_snapshot(&engine, event_ms, &snapshot);
    CHECK(snapshot.event_visible == 1);
    hand_gesture_engine_snapshot(&engine,
                                 event_ms + engine.config.event_animation_ms + 1U,
                                 &snapshot);
    CHECK(snapshot.event_visible == 0);
    return 0;
}

static int test_detection_loss_during_animation_resets_tracking(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    hand_gesture_snapshot snapshot;
    uint64_t event_ms = 0U;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 12U; ++index) {
        const float x = 0.20F + 0.05F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection(x, 0),
                             (uint64_t)index * 100U,
                             &event) == 0);
        if (event.valid) {
            event_ms = event.timestamp_ms;
            break;
        }
    }
    CHECK(event_ms != 0U);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);

    CHECK(hand_gesture_engine_update(&engine, NULL, 0U, event_ms + 300U, &event) == 0);
    CHECK(event.valid == 0);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);
    CHECK(engine.has_last_hand == 0);
    CHECK(engine.has_center == 0);
    CHECK(engine.recognizer.trajectory.count == 0U);

    hand_gesture_engine_snapshot(&engine, event_ms + 300U, &snapshot);
    CHECK(snapshot.event_visible == 1);
    CHECK(snapshot.event.direction == HAND_DIRECTION_RIGHT);

    CHECK(hand_gesture_engine_update(&engine, NULL, 0U, event_ms + 1100U, &event) == 0);
    CHECK(engine.has_last_event == 0);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);
    hand_gesture_engine_snapshot(&engine, event_ms + 1100U, &snapshot);
    CHECK(snapshot.event_visible == 0);
    return 0;
}

static int test_tracker_rejects_remote_high_confidence_box(void)
{
    hand_tracker tracker;
    hand_track track;
    hand_detection first = make_detection(0.30F, 0);
    hand_detection candidates[2];

    hand_tracker_init(&tracker, NULL);
    CHECK(hand_tracker_update(&tracker, &first, 1U, 1000U, &track) == 0);
    CHECK(track.valid);

    /* A remote, higher-confidence box must not steal the current track. */
    candidates[0] = make_detection(0.34F, 0);
    candidates[0].confidence = 0.70F;
    candidates[1] = make_detection(0.90F, 0);
    candidates[1].confidence = 0.99F;
    CHECK(hand_tracker_update(&tracker, candidates, 2U, 1100U, &track) == 0);
    CHECK(track.valid);
    CHECK(track.smoothed.x0 < 0.40F);
    return 0;
}

static int test_tracker_rejects_full_screen_jump(void)
{
    hand_tracker tracker;
    hand_track track;
    hand_detection first = make_detection(0.20F, 0);
    hand_detection remote = make_detection(0.80F, 0);

    hand_tracker_init(&tracker, NULL);
    CHECK(hand_tracker_update(&tracker, &first, 1U, 1000U, &track) == 0);
    CHECK(track.valid);

    CHECK(hand_tracker_update(&tracker, &remote, 1U, 1100U, &track) == 0);
    CHECK(!track.valid);
    return 0;
}

static int test_trajectory_starts_fresh_after_jump(void)
{
    hand_trajectory trajectory;
    hand_point first = {0.20F, 0.50F, 1000U};
    hand_point remote = {0.80F, 0.50F, 1100U};

    hand_trajectory_reset(&trajectory);
    CHECK(hand_trajectory_push(&trajectory, first, 250U, 700U) == 0);
    CHECK(hand_trajectory_push(&trajectory, remote, 250U, 700U) == 0);
    CHECK(hand_trajectory_count(&trajectory) == 1U);
    CHECK(hand_trajectory_at(&trajectory, 0U)->x == remote.x);
    return 0;
}

static int test_tracker_jump_does_not_create_gesture_event(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    hand_gesture_snapshot snapshot;

    hand_gesture_engine_init(&engine, NULL);
    CHECK(feed_detection(&engine, make_detection(0.20F, 0), 100U, &event) == 0);
    CHECK(feed_detection(&engine, make_detection(0.22F, 0), 145U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_CAPTURING);

    CHECK(feed_detection(&engine, make_detection(0.80F, 0), 200U, &event) == 0);
    CHECK(event.valid == 0);
    hand_gesture_engine_snapshot(&engine, 200U, &snapshot);
    CHECK(snapshot.event_visible == 0);
    return 0;
}

static int test_tracker_accepts_fast_reversal(void)
{
    hand_tracker tracker;
    hand_track track;
    hand_detection detection;

    hand_tracker_init(&tracker, NULL);
    detection = make_detection(0.30F, 0);
    CHECK(hand_tracker_update(&tracker, &detection, 1U, 1000U, &track) == 0);
    detection = make_detection(0.45F, 0);
    CHECK(hand_tracker_update(&tracker, &detection, 1U, 1104U, &track) == 0);
    CHECK(track.valid);
    detection = make_detection(0.30F, 0);
    CHECK(hand_tracker_update(&tracker, &detection, 1U, 1208U, &track) == 0);
    CHECK(track.valid);
    CHECK(track.smoothed.x0 < 0.40F);
    return 0;
}

static int test_horizontal_edge_does_not_create_gesture(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 8U; ++index) {
        const float y = 0.30F + 0.06F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection_at(0.88F, y, 0),
                             (uint64_t)index * 100U,
                             &event) == 0);
        CHECK(event.valid == 0);
    }

    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(engine.recognizer.trajectory.count == 0U);
    return 0;
}

static int test_vertical_edge_does_not_create_gesture(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 8U; ++index) {
        const float x = 0.25F + 0.07F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection_at(x, 0.88F, 0),
                             (uint64_t)index * 100U,
                             &event) == 0);
        CHECK(event.valid == 0);
    }

    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(engine.recognizer.trajectory.count == 0U);
    return 0;
}

static int test_edge_exit_clears_partial_trajectory(void)
{
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    hand_gesture_event event;
    unsigned int index;

    hand_gesture_engine_config_init_defaults(&config);
    config.activation_confirm_frames = 1U;
    config.tracker.ema_alpha = 1.0F;
    config.tracker.max_speed_norm_per_second = 10.0F;
    config.tracker.max_step_norm = 1.0F;
    config.direction.min_samples = 5U;
    config.direction.min_span_ms = 250U;
    config.direction.confirm_count = 1U;
    hand_gesture_engine_init(&engine, &config);

    for (index = 0U; index < 4U; ++index) {
        const float x = 0.30F + 0.10F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection(x, 0),
                             100U + (uint64_t)index * 100U,
                             &event) == 0);
        CHECK(event.valid == 0);
    }
    CHECK(engine.state == HAND_ENGINE_CAPTURING);
    CHECK(engine.recognizer.trajectory.count == 4U);
    CHECK(engine.roi_inside == 1);

    /* The first point outside the strict ROI invalidates the whole partial
     * trajectory, even though it remains inside the hysteresis band. */
    CHECK(feed_detection(&engine, make_detection(0.84F, 0), 500U, &event) == 0);
    CHECK(event.valid == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(engine.recognizer.trajectory.count == 0U);
    CHECK(engine.roi_inside == 1);

    CHECK(feed_detection(&engine, make_detection(0.90F, 0), 600U, &event) == 0);
    CHECK(engine.roi_inside == 0);

    /* Returning to the centre starts a new trajectory and must collect all
     * five samples again; the four points before the edge cannot be reused. */
    for (index = 0U; index < 4U; ++index) {
        const float x = 0.30F + 0.10F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection(x, 0),
                             700U + (uint64_t)index * 100U,
                             &event) == 0);
        CHECK(event.valid == 0);
    }
    CHECK(engine.state == HAND_ENGINE_CAPTURING);
    CHECK(engine.recognizer.trajectory.count == 4U);

    CHECK(feed_detection(&engine, make_detection(0.70F, 0), 1100U, &event) == 0);
    CHECK(event.valid == 1);
    CHECK(event.direction == HAND_DIRECTION_RIGHT);
    return 0;
}

static int test_lcd_visible_edges_do_not_create_gesture(void)
{
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    hand_gesture_event event;
    unsigned int index;
    int saw_gesture = 0;

    hand_gesture_engine_config_init_defaults(&config);
    /* 640x360 source, LCD-visible crop x=184..454, central 60% of crop. */
    config.roi_x_min = 0.371875F;
    config.roi_x_max = 0.625000F;
    config.roi_y_min = 0.200000F;
    config.roi_y_max = 0.800000F;
    config.roi_hysteresis = 0.05F;
    hand_gesture_engine_init(&engine, &config);

    /* x=0.35 is visibly near the left edge but was accepted by the old
     * full-frame 0.20..0.80 ROI. A vertical swipe there must be ignored. */
    for (index = 0U; index < 8U; ++index) {
        const float y = 0.72F - 0.05F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection_at(0.35F, y, 0),
                             100U + (uint64_t)index * 100U,
                             &event) == 0);
        CHECK(event.valid == 0);
    }
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(engine.recognizer.trajectory.count == 0U);

    /* The same swipe inside the centre may produce a gesture. */
    for (index = 0U; index < 11U; ++index) {
        const float y = 0.72F - 0.04F * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection_at(0.50F, y, 0),
                             1000U + (uint64_t)index * 100U,
                             &event) == 0);
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_UP);
            saw_gesture = 1;
            break;
        }
    }
    CHECK(saw_gesture == 1);
    return 0;
}

static int test_brief_occlusion_under_250ms_preserves_capture(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    int saw_gesture = 0;
    unsigned int index;
    uint64_t t = 0U;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 14U; ++index) {
        const float x = 0.25F + 0.03F * (float)index;

        if (index == 6U || index == 7U || index == 8U) {
            /* 3 dropped frames (~135 ms gap <= 250 ms) must NOT reset capture. */
            CHECK(hand_gesture_engine_update(&engine, NULL, 0U, t, &event) == 0);
            CHECK(event.valid == 0);
            if (index >= 2U) {
                CHECK(engine.state == HAND_ENGINE_CAPTURING);
            }
        } else {
            CHECK(feed_detection(&engine, make_detection(x, 0), t, &event) == 0);
            if (event.valid) {
                CHECK(event.direction == HAND_DIRECTION_RIGHT);
                saw_gesture = 1;
                break;
            }
        }
        t += 45U;
    }
    CHECK(saw_gesture == 1);
    return 0;
}

static int test_long_occlusion_over_250ms_resets_capture(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 6U; ++index) {
        CHECK(feed_detection(&engine,
                             make_detection(0.25F + 0.03F * (float)index, 0),
                             (uint64_t)index * 45U,
                             &event) == 0);
        CHECK(event.valid == 0);
    }
    CHECK(engine.state == HAND_ENGINE_CAPTURING);
    CHECK(engine.recognizer.trajectory.count > 0U);

    /* Last valid detection was at 225 ms; occlusion to 500 ms (gap = 275 ms > 250 ms) resets. */
    CHECK(hand_gesture_engine_update(&engine, NULL, 0U, 500U, &event) == 0);
    CHECK(event.valid == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_HAND);
    CHECK(engine.recognizer.trajectory.count == 0U);
    return 0;
}

static int test_missed_frames_after_animation_preserve_2000ms_cooldown(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    uint64_t event_ms = 0U;
    int second_event = 0;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    for (index = 0U; index < 14U; ++index) {
        CHECK(feed_detection(&engine,
                             make_detection(0.25F + 0.03F * (float)index, 0),
                             (uint64_t)index * 45U,
                             &event) == 0);
        if (event.valid) {
            event_ms = event.timestamp_ms;
            break;
        }
    }
    CHECK(event_ms != 0U);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);

    /* After 1000 ms animation expires, drop detections at 1200..1500 ms. */
    CHECK(hand_gesture_engine_update(&engine, NULL, 0U, event_ms + 1150U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);
    CHECK(hand_gesture_engine_update(&engine, NULL, 0U, event_ms + 1500U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_COOLDOWN);

    /* Reappearing at 1600 ms must still be in cooldown until 2000 ms passes. */
    for (index = 0U; index < 7U; ++index) {
        CHECK(feed_detection(&engine,
                             make_detection(0.75F - 0.04F * (float)index, 0),
                             event_ms + 1600U + (uint64_t)index * 45U,
                             &event) == 0);
        CHECK(event.valid == 0);
        CHECK(engine.state == HAND_ENGINE_COOLDOWN);
    }

    /* Once 2000 ms passes, a fresh swipe triggers cleanly. */
    for (index = 0U; index < 15U; ++index) {
        CHECK(feed_detection(&engine,
                             make_detection(0.75F - 0.03F * (float)index, 0),
                             event_ms + 2050U + (uint64_t)index * 45U,
                             &event) == 0);
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_LEFT);
            second_event = 1;
            break;
        }
    }
    CHECK(second_event == 1);
    return 0;
}

static int test_single_frame_activation_flash_ignored(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;

    hand_gesture_engine_init(&engine, NULL);
    CHECK(feed_detection(&engine, make_detection(0.50F, 0), 100U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(engine.activation_streak == 1U);

    CHECK(hand_gesture_engine_update(&engine, NULL, 0U, 145U, &event) == 0);
    CHECK(engine.state == HAND_ENGINE_WAITING_FOR_ACTIVATION);
    CHECK(engine.activation_streak == 0U);
    CHECK(engine.recognizer.trajectory.count == 0U);
    return 0;
}

static int test_hand_enter_and_exit_frame_no_false_trigger(void)
{
    hand_gesture_engine engine;
    hand_gesture_event event;
    unsigned int index;

    hand_gesture_engine_init(&engine, NULL);
    /* Hand enters from left edge into ROI (3 frames) then immediately exits left. */
    {
        static const float xs[7] = {
            0.10F, 0.16F, 0.22F, 0.26F, 0.24F, 0.18F, 0.12F
        };
        for (index = 0U; index < 7U; ++index) {
            CHECK(feed_detection(&engine,
                                 make_detection(xs[index], 0),
                                 (uint64_t)index * 45U,
                                 &event) == 0);
            CHECK(event.valid == 0);
        }
    }
    return 0;
}

static int test_box_size_change_during_swipe_preserves_direction(void)
{
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    hand_gesture_event event;
    int saw_gesture = 0;
    unsigned int index;

    hand_gesture_engine_config_init_defaults(&config);
    config.direction.x_scale = 640.0F / 360.0F;
    config.roi_x_min = 0.371875F;
    config.roi_x_max = 0.625000F;
    config.roi_y_min = 0.200000F;
    config.roi_y_max = 0.800000F;
    hand_gesture_engine_init(&engine, &config);

    for (index = 0U; index < 14U; ++index) {
        hand_detection det;
        const float cx = 0.41F + 0.015F * (float)index;
        const float half_w = 0.05F;
        /* Suddenly expand box vertically downwards mid-swipe (e.g. forearm enters box). */
        const float top = 0.42F;
        const float bottom = (index >= 5U && index <= 8U) ? 0.72F : 0.56F;

        det.x0 = cx - half_w;
        det.x1 = cx + half_w;
        det.y0 = top;
        det.y1 = bottom;
        det.confidence = 0.90F;
        det.class_id = 0;

        CHECK(feed_detection(&engine, det, (uint64_t)index * 45U, &event) == 0);
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
            saw_gesture = 1;
            break;
        }
    }
    CHECK(saw_gesture == 1);
    return 0;
}

static int run_22hz_roi_swipe(float start_x,
                              float start_y,
                              float step_x,
                              float step_y,
                              hand_direction_code expected_dir,
                              uint64_t *latency_ms)
{
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    hand_gesture_event event;
    int saw_gesture = 0;
    unsigned int index;

    hand_gesture_engine_config_init_defaults(&config);
    config.direction.x_scale = 640.0F / 360.0F;
    config.roi_x_min = 0.371875F;
    config.roi_x_max = 0.625000F;
    config.roi_y_min = 0.200000F;
    config.roi_y_max = 0.800000F;
    hand_gesture_engine_init(&engine, &config);

    for (index = 0U; index < 14U; ++index) {
        const float x = start_x + step_x * (float)index;
        const float y = start_y + step_y * (float)index;

        CHECK(feed_detection(&engine,
                             make_detection_at(x, y, 0),
                             (uint64_t)index * 45U,
                             &event) == 0);
        if (event.valid) {
            CHECK(event.direction == expected_dir);
            if (latency_ms != NULL) {
                *latency_ms = event.timestamp_ms;
            }
            saw_gesture = 1;
            break;
        }
    }
    return saw_gesture;
}

static int test_lcd_60pct_roi_four_directions_with_x_scale_and_edge_rejection(void)
{
    uint64_t latency = 0U;

    CHECK(run_22hz_roi_swipe(0.40F, 0.50F, 0.017F, 0.0F, HAND_DIRECTION_RIGHT, &latency) == 1);
    CHECK(latency >= 130U && latency <= 420U);

    CHECK(run_22hz_roi_swipe(0.60F, 0.50F, -0.017F, 0.0F, HAND_DIRECTION_LEFT, &latency) == 1);
    CHECK(latency >= 130U && latency <= 420U);

    CHECK(run_22hz_roi_swipe(0.50F, 0.70F, 0.0F, -0.030F, HAND_DIRECTION_UP, &latency) == 1);
    CHECK(latency >= 130U && latency <= 420U);

    CHECK(run_22hz_roi_swipe(0.50F, 0.30F, 0.0F, 0.030F, HAND_DIRECTION_DOWN, &latency) == 1);
    CHECK(latency >= 130U && latency <= 420U);

    /* Four edge bands outside the strict central 60% ROI must all reject. */
    CHECK(run_22hz_roi_swipe(0.35F, 0.70F, 0.0F, -0.030F, HAND_DIRECTION_UP, NULL) == 0);
    CHECK(run_22hz_roi_swipe(0.65F, 0.70F, 0.0F, -0.030F, HAND_DIRECTION_UP, NULL) == 0);
    CHECK(run_22hz_roi_swipe(0.40F, 0.15F, 0.017F, 0.0F, HAND_DIRECTION_RIGHT, NULL) == 0);
    CHECK(run_22hz_roi_swipe(0.40F, 0.85F, 0.017F, 0.0F, HAND_DIRECTION_RIGHT, NULL) == 0);
    return 0;
}

static int test_fast_swipe_confirms_before_strict_roi_exit(void)
{
    /* Reproduces board frame 600: fast LEFT swipe triggers right before exiting
     * strict 60% LCD ROI (x=0.371875..0.625000). */
    static const float xs[9] = {
        0.610F, 0.590F, 0.560F, 0.525F, 0.490F,
        0.455F, 0.420F, 0.385F, 0.320F
    };
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    hand_gesture_event event;
    int saw_gesture = 0;
    unsigned int index;

    hand_gesture_engine_config_init_defaults(&config);
    config.direction.x_scale = 640.0F / 360.0F;
    config.roi_x_min = 0.371875F;
    config.roi_x_max = 0.625000F;
    config.roi_y_min = 0.200000F;
    config.roi_y_max = 0.800000F;
    hand_gesture_engine_init(&engine, &config);

    for (index = 0U; index < 9U; ++index) {
        CHECK(feed_detection(&engine,
                             make_detection_at(xs[index], 0.50F, 0),
                             (uint64_t)index * 45U,
                             &event) == 0);
        if (event.valid) {
            CHECK(index <= 7U);
            CHECK(event.direction == HAND_DIRECTION_LEFT);
            saw_gesture = 1;
            break;
        }
    }
    CHECK(saw_gesture == 1);
    return 0;
}

static int test_brisk_fast_vertical_and_horizontal_swipes_in_60pct_roi(void)
{
    uint64_t latency = 0U;

    /* Fast vertical DOWN/UP (3.0 norm/s, 135-180 ms) inside strict 60% ROI. */
    CHECK(run_22hz_roi_swipe(0.50F, 0.24F, 0.0F, 0.135F, HAND_DIRECTION_DOWN, &latency) == 1);
    CHECK(latency <= 180U);

    CHECK(run_22hz_roi_swipe(0.50F, 0.76F, 0.0F, -0.135F, HAND_DIRECTION_UP, &latency) == 1);
    CHECK(latency <= 180U);

    /* Fast horizontal LEFT/RIGHT inside strict 60% ROI (135-180 ms). */
    CHECK(run_22hz_roi_swipe(0.61F, 0.50F, -0.055F, 0.0F, HAND_DIRECTION_LEFT, &latency) == 1);
    CHECK(latency <= 180U);

    CHECK(run_22hz_roi_swipe(0.39F, 0.50F, 0.055F, 0.0F, HAND_DIRECTION_RIGHT, &latency) == 1);
    CHECK(latency <= 180U);
    return 0;
}

static int test_snapshot_tracked_box_and_cooldown_progress(void)
{
    hand_gesture_engine_config config;
    hand_gesture_engine engine;
    hand_gesture_event event;
    hand_gesture_snapshot snapshot;
    uint64_t event_ms = 0U;
    int saw_capturing_snapshot = 0;
    unsigned int index;

    hand_gesture_engine_config_init_defaults(&config);
    config.direction.x_scale = 640.0F / 360.0F;
    config.roi_x_min = 0.371875F;
    config.roi_x_max = 0.625000F;
    config.roi_y_min = 0.200000F;
    config.roi_y_max = 0.800000F;
    hand_gesture_engine_init(&engine, &config);

    hand_gesture_engine_snapshot(&engine, 0U, &snapshot);
    CHECK(snapshot.state == HAND_ENGINE_WAITING_FOR_HAND);
    CHECK(snapshot.has_tracked_box == 0);
    CHECK(snapshot.cooldown_total_ms == 2000U);
    CHECK(snapshot.cooldown_remaining_ms == 0U);
    CHECK(snapshot.roi_x_min == config.roi_x_min);
    CHECK(snapshot.roi_x_max == config.roi_x_max);

    for (index = 0U; index < 14U; ++index) {
        const uint64_t t = 100U + (uint64_t)index * 45U;
        const float x = 0.40F + 0.017F * (float)index;

        CHECK(feed_detection(&engine, make_detection_at(x, 0.50F, 0), t, &event) == 0);
        hand_gesture_engine_snapshot(&engine, t, &snapshot);
        CHECK(snapshot.has_tracked_box == 1);
        CHECK(snapshot.tracked_box.confidence >= 0.85F);
        CHECK(snapshot.roi_inside == 1);

        if (engine.state == HAND_ENGINE_CAPTURING) {
            saw_capturing_snapshot = 1;
            CHECK(snapshot.state == HAND_ENGINE_CAPTURING);
            CHECK(snapshot.cooldown_remaining_ms == 0U);
        }
        if (event.valid) {
            event_ms = event.timestamp_ms;
            break;
        }
    }

    CHECK(saw_capturing_snapshot == 1);
    CHECK(event_ms != 0U);

    hand_gesture_engine_snapshot(&engine, event_ms, &snapshot);
    CHECK(snapshot.state == HAND_ENGINE_COOLDOWN);
    CHECK(snapshot.cooldown_remaining_ms == 2000U);

    hand_gesture_engine_snapshot(&engine, event_ms + 750U, &snapshot);
    CHECK(snapshot.state == HAND_ENGINE_COOLDOWN);
    CHECK(snapshot.cooldown_remaining_ms == 1250U);

    hand_gesture_engine_snapshot(&engine, event_ms + 2000U, &snapshot);
    CHECK(snapshot.state == HAND_ENGINE_WAITING_FOR_HAND);
    CHECK(snapshot.cooldown_remaining_ms == 0U);
    return 0;
}

int main(void)
{
    if (test_no_activation_with_wrong_class() != 0 ||
        test_hand_activation_and_right_swipe() != 0 ||
        test_cooldown_blocks_duplicate_event() != 0 ||
        test_default_cooldown_is_2000ms() != 0 ||
        test_detection_loss_resets_capture() != 0 ||
        test_min_samples_and_animation_expiry() != 0 ||
        test_detection_loss_during_animation_resets_tracking() != 0 ||
        test_tracker_rejects_remote_high_confidence_box() != 0 ||
        test_tracker_rejects_full_screen_jump() != 0 ||
        test_trajectory_starts_fresh_after_jump() != 0 ||
        test_tracker_jump_does_not_create_gesture_event() != 0 ||
        test_tracker_accepts_fast_reversal() != 0 ||
        test_horizontal_edge_does_not_create_gesture() != 0 ||
        test_vertical_edge_does_not_create_gesture() != 0 ||
        test_edge_exit_clears_partial_trajectory() != 0 ||
        test_lcd_visible_edges_do_not_create_gesture() != 0 ||
        test_brief_occlusion_under_250ms_preserves_capture() != 0 ||
        test_long_occlusion_over_250ms_resets_capture() != 0 ||
        test_missed_frames_after_animation_preserve_2000ms_cooldown() != 0 ||
        test_single_frame_activation_flash_ignored() != 0 ||
        test_hand_enter_and_exit_frame_no_false_trigger() != 0 ||
        test_box_size_change_during_swipe_preserves_direction() != 0 ||
        test_lcd_60pct_roi_four_directions_with_x_scale_and_edge_rejection() != 0 ||
        test_fast_swipe_confirms_before_strict_roi_exit() != 0 ||
        test_brisk_fast_vertical_and_horizontal_swipes_in_60pct_roi() != 0 ||
        test_snapshot_tracked_box_and_cooldown_progress() != 0) {
        return 1;
    }

    (void)printf("gesture engine tests passed\n");
    return 0;
}
