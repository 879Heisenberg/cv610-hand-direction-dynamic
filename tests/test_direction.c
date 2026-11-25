#include "hand_direction/direction.h"
#include "hand_direction/trajectory.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition)                                                         \
    do {                                                                         \
        if (!(condition)) {                                                      \
            (void)fprintf(stderr, "CHECK failed at %s:%d: %s\n",               \
                          __FILE__, __LINE__, #condition);                       \
            return 1;                                                            \
        }                                                                        \
    } while (0)

static int feed_line(hand_direction_recognizer *recognizer,
                     float start_x,
                     float start_y,
                     float end_x,
                     float end_y,
                     size_t sample_count,
                     hand_gesture_event *last_event)
{
    int recognized = 0;
    size_t index;

    for (index = 0U; index < sample_count; ++index) {
        const float ratio = sample_count > 1U
                                ? (float)index / (float)(sample_count - 1U)
                                : 0.0F;
        hand_point point;
        hand_gesture_event event;
        int result;

        point.x = start_x + (end_x - start_x) * ratio;
        point.y = start_y + (end_y - start_y) * ratio;
        point.timestamp_ms = (uint64_t)index * 100U;
        result = hand_direction_update(recognizer, point, &event);
        if (result < 0) {
            return -1;
        }
        if (event.valid) {
            recognized = 1;
            if (last_event != NULL) {
                *last_event = event;
            }
        }
    }
    return recognized;
}

static int line_is_recognized_as(float start_x,
                                 float start_y,
                                 float end_x,
                                 float end_y,
                                 hand_direction_code expected)
{
    hand_direction_recognizer recognizer;
    hand_gesture_event event;

    hand_direction_init(&recognizer, NULL);
    CHECK(feed_line(&recognizer, start_x, start_y, end_x, end_y, 9U, &event) == 1);
    CHECK(event.valid);
    CHECK(event.direction == expected);
    CHECK(event.confidence > 0.0F && event.confidence <= 1.0F);
    return 0;
}

static int test_four_directions(void)
{
    if (line_is_recognized_as(0.20F, 0.50F, 0.80F, 0.50F, HAND_DIRECTION_RIGHT) != 0) {
        return 1;
    }
    if (line_is_recognized_as(0.80F, 0.50F, 0.20F, 0.50F, HAND_DIRECTION_LEFT) != 0) {
        return 1;
    }
    if (line_is_recognized_as(0.50F, 0.75F, 0.50F, 0.15F, HAND_DIRECTION_UP) != 0) {
        return 1;
    }
    if (line_is_recognized_as(0.50F, 0.15F, 0.50F, 0.75F, HAND_DIRECTION_DOWN) != 0) {
        return 1;
    }
    return 0;
}

static int test_min_samples_and_confirmation_required(void)
{
    hand_direction_recognizer recognizer;
    hand_direction_diagnostics diag;
    size_t index;

    hand_direction_init(&recognizer, NULL);
    for (index = 0U; index < 7U; ++index) {
        hand_point point;
        hand_gesture_event event;
        int result;

        /* Moderate displacement (<= 0.15 < 0.22) must still wait for 2-frame confirm. */
        point.x = 0.25F + 0.025F * (float)index;
        point.y = 0.50F;
        point.timestamp_ms = (uint64_t)index * 45U;
        result = hand_direction_update(&recognizer, point, &event);

        if (index < 6U) {
            CHECK(result == 0);
            CHECK(event.valid == 0);
        } else {
            CHECK(result == 1);
            CHECK(event.valid == 1);
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
        }
    }

    hand_direction_get_diagnostics(&recognizer, &diag);
    CHECK(diag.emitted_direction == HAND_DIRECTION_RIGHT);
    CHECK(diag.moving_segments >= 4U);
    CHECK(diag.segment_consistency >= 0.75F);
    return 0;
}

static int test_high_conf_fast_swipe_single_frame_confirm(void)
{
    hand_direction_recognizer recognizer;
    size_t index;

    hand_direction_init(&recognizer, NULL);
    for (index = 0U; index < 6U; ++index) {
        hand_point point = {
            0.25F + 0.045F * (float)index,
            0.50F,
            (uint64_t)index * 45U
        };
        hand_gesture_event event;
        const int result = hand_direction_update(&recognizer, point, &event);

        if (index < 5U) {
            CHECK(result == 0);
            CHECK(event.valid == 0);
        } else {
            CHECK(result == 1);
            CHECK(event.valid == 1);
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
        }
    }
    return 0;
}

static int test_insufficient_displacement(void)
{
    hand_direction_recognizer recognizer;

    hand_direction_init(&recognizer, NULL);
    CHECK(feed_line(&recognizer, 0.45F, 0.50F, 0.54F, 0.50F, 9U, NULL) == 0);
    return 0;
}

static int test_slow_swipe_is_recognized(void)
{
    hand_direction_recognizer recognizer;
    int recognized = 0;
    size_t index;

    hand_direction_init(&recognizer, NULL);
    for (index = 0U; index < 15U; ++index) {
        hand_point point;
        hand_gesture_event event;

        point.x = 0.38F + 0.0125F * (float)index;
        point.y = 0.50F;
        point.timestamp_ms = (uint64_t)index * 45U;
        CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
            recognized = 1;
            break;
        }
    }
    CHECK(recognized == 1);
    return 0;
}

static int test_stationary_jitter_is_rejected(void)
{
    hand_direction_recognizer recognizer;
    size_t index;

    hand_direction_init(&recognizer, NULL);
    for (index = 0U; index < 660U; ++index) {
        hand_point point;
        hand_gesture_event event;

        point.x = 0.50F + 0.012F * sinf((float)index * 0.7F);
        point.y = 0.50F + 0.012F * cosf((float)index * 0.9F);
        point.timestamp_ms = (uint64_t)index * 45U;
        CHECK(hand_direction_update(&recognizer, point, &event) == 0);
        CHECK(event.valid == 0);
    }
    return 0;
}

static int test_curved_motion_is_rejected(void)
{
    const float radius = 0.15F;
    size_t trajectory_index;

    for (trajectory_index = 0U; trajectory_index < 2U; ++trajectory_index) {
        hand_direction_recognizer recognizer;
        size_t index;

        hand_direction_init(&recognizer, NULL);
        for (index = 0U; index < 15U; ++index) {
            const float angle = 3.14159265F * (1.0F - (float)index / 14.0F);
            hand_point point;
            hand_gesture_event event;
            const float direction_sign = trajectory_index == 0U ? 1.0F : -1.0F;

            point.x = 0.50F + radius * cosf(angle);
            point.y = 0.50F + direction_sign * radius * sinf(angle);
            point.timestamp_ms = (uint64_t)index * 45U;
            CHECK(hand_direction_update(&recognizer, point, &event) == 0);
            CHECK(event.valid == 0);
        }
    }

    /* Asymmetric J/L bend must also be rejected rather than classified straight. */
    {
        static const float xs[9] = {
            0.35F, 0.39F, 0.43F, 0.47F, 0.51F,
            0.51F, 0.51F, 0.51F, 0.51F
        };
        static const float ys[9] = {
            0.50F, 0.50F, 0.50F, 0.50F, 0.50F,
            0.525F, 0.550F, 0.575F, 0.600F
        };
        hand_direction_recognizer recognizer;
        size_t index;

        hand_direction_init(&recognizer, NULL);
        for (index = 0U; index < 9U; ++index) {
            hand_point point = {xs[index], ys[index], (uint64_t)index * 45U};
            hand_gesture_event event;

            CHECK(hand_direction_update(&recognizer, point, &event) == 0);
            CHECK(event.valid == 0);
        }
    }
    return 0;
}

static int test_prefix_offset_then_true_direction(void)
{
    static const float xs[12] = {
        0.30F, 0.312F, 0.324F, 0.350F, 0.378F, 0.406F,
        0.434F, 0.462F, 0.490F, 0.518F, 0.546F, 0.574F
    };
    static const float ys[12] = {
        0.50F, 0.515F, 0.525F, 0.525F, 0.525F, 0.525F,
        0.525F, 0.525F, 0.525F, 0.525F, 0.525F, 0.525F
    };
    hand_direction_recognizer recognizer;
    int recognized = 0;
    size_t index;

    hand_direction_init(&recognizer, NULL);
    for (index = 0U; index < 12U; ++index) {
        hand_point point = {xs[index], ys[index], (uint64_t)index * 45U};
        hand_gesture_event event;

        CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
            recognized = 1;
            break;
        }
    }
    CHECK(recognized == 1);
    return 0;
}

static int test_leading_stationary_dwell_then_swipe(void)
{
    hand_direction_recognizer recognizer;
    int recognized = 0;
    uint64_t trigger_ms = 0U;
    size_t index;

    hand_direction_init(&recognizer, NULL);
    for (index = 0U; index < 15U; ++index) {
        hand_point point;
        hand_gesture_event event;

        if (index < 5U) {
            point.x = 0.35F + ((index % 2U == 0U) ? 0.001F : 0.0F);
        } else {
            point.x = 0.35F + 0.025F * (float)(index - 4U);
        }
        point.y = 0.50F;
        point.timestamp_ms = (uint64_t)index * 45U;
        CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
            recognized = 1;
            trigger_ms = event.timestamp_ms;
            break;
        }
    }
    CHECK(recognized == 1);
    CHECK(trigger_ms <= 675U);
    return 0;
}

static int test_reversing_segments_rejected(void)
{
    static const float xs[9] = {
        0.25F, 0.29F, 0.33F, 0.37F, 0.41F,
        0.45F, 0.49F, 0.45F, 0.41F
    };
    hand_direction_config config;
    hand_direction_recognizer recognizer;
    size_t index;

    hand_direction_config_init_defaults(&config);
    config.min_linearity = 0.55F;
    config.min_half_cosine = 0.50F;
    config.confirm_count = 1U;
    hand_direction_init(&recognizer, &config);

    for (index = 0U; index < 9U; ++index) {
        hand_point point = {xs[index], 0.50F, (uint64_t)index * 45U};
        hand_gesture_event event;

        if (index < 7U) {
            (void)hand_direction_update(&recognizer, point, &event);
        } else {
            CHECK(hand_direction_update(&recognizer, point, &event) == 0);
            CHECK(event.valid == 0);
        }
    }
    return 0;
}

static int test_aspect_ratio_x_scale_symmetry(void)
{
    const float x_scale = 640.0F / 360.0F;
    hand_direction_config config;
    hand_direction_recognizer recognizer;
    size_t index;

    hand_direction_config_init_defaults(&config);
    config.x_scale = x_scale;

    /* Narrow horizontal swipe inside 60% LCD ROI (dx_norm=0.112 -> 0.20 scaled). */
    {
        int recognized = 0;

        hand_direction_init(&recognizer, &config);
        for (index = 0U; index < 10U; ++index) {
            hand_point point = {
                0.42F + 0.013F * (float)index,
                0.50F,
                (uint64_t)index * 45U
            };
            hand_gesture_event event;

            CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
            if (event.valid) {
                CHECK(event.direction == HAND_DIRECTION_RIGHT);
                recognized = 1;
                break;
            }
        }
        CHECK(recognized == 1);
    }

    /* Physically 45-degree diagonal must be rejected (without x_scale it looked vertical). */
    hand_direction_init(&recognizer, &config);
    for (index = 0U; index < 10U; ++index) {
        const float step = 0.025F * (float)index;
        hand_point point = {
            0.40F + step / x_scale,
            0.35F + step,
            (uint64_t)index * 45U
        };
        hand_gesture_event event;

        CHECK(hand_direction_update(&recognizer, point, &event) == 0);
        CHECK(event.valid == 0);
    }

    /* Physically 20-degree slanted horizontal swipe must still be accepted as RIGHT. */
    {
        int recognized = 0;

        hand_direction_init(&recognizer, &config);
        for (index = 0U; index < 10U; ++index) {
            const float phys_x = 0.025F * (float)index;
            const float phys_y = phys_x * 0.36F;
            hand_point point = {
                0.40F + phys_x / x_scale,
                0.45F + phys_y,
                (uint64_t)index * 45U
            };
            hand_gesture_event event;

            CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
            if (event.valid) {
                CHECK(event.direction == HAND_DIRECTION_RIGHT);
                recognized = 1;
                break;
            }
        }
        CHECK(recognized == 1);
    }
    return 0;
}

static int test_trajectory_reset_and_expiry(void)
{
    hand_trajectory trajectory;
    hand_point point;

    hand_trajectory_reset(&trajectory);

    point.x = 0.20F;
    point.y = 0.50F;
    point.timestamp_ms = 1000U;
    CHECK(hand_trajectory_push(&trajectory, point, 250U, 800U) == 0);

    point.timestamp_ms = 900U;
    CHECK(hand_trajectory_push(&trajectory, point, 250U, 800U) == 0);
    CHECK(hand_trajectory_count(&trajectory) == 1U);
    CHECK(hand_trajectory_at(&trajectory, 0U)->timestamp_ms == 900U);

    point.timestamp_ms = 1300U;
    CHECK(hand_trajectory_push(&trajectory, point, 250U, 800U) == 0);
    CHECK(hand_trajectory_count(&trajectory) == 1U);
    CHECK(hand_trajectory_at(&trajectory, 0U)->timestamp_ms == 1300U);

    hand_trajectory_reset(&trajectory);
    CHECK(hand_trajectory_push(&trajectory, point, 500U, 300U) == 0);
    point.timestamp_ms = 1700U;
    CHECK(hand_trajectory_push(&trajectory, point, 500U, 300U) == 0);
    CHECK(hand_trajectory_count(&trajectory) == 1U);
    return 0;
}

static int test_trajectory_rejects_spike_and_starts_new_stroke(void)
{
    hand_trajectory trajectory;
    hand_point point;

    hand_trajectory_reset(&trajectory);
    point.x = 0.20F; point.y = 0.50F; point.timestamp_ms = 1000U;
    CHECK(hand_trajectory_push(&trajectory, point, 250U, 800U) == 0);
    point.x = 0.24F; point.timestamp_ms = 1033U;
    CHECK(hand_trajectory_push(&trajectory, point, 250U, 800U) == 0);
    CHECK(hand_trajectory_count(&trajectory) == 2U);

    /* Isolated jump clears the contaminated stroke; it is not joined by a line. */
    point.x = 0.90F; point.timestamp_ms = 1066U;
    CHECK(hand_trajectory_push(&trajectory, point, 250U, 800U) == 0);
    CHECK(hand_trajectory_count(&trajectory) == 1U);
    CHECK(fabsf(hand_trajectory_at(&trajectory, 0U)->x - 0.90F) < 0.001F);
    return 0;
}

static int test_hesitation_then_swipe_arc_midpoint(void)
{
    /* Reproduces board frame 720: initial slight upward/jitter hesitation
     * followed by clean LEFT swipe; arc-length midpoint keeps cos aligned. */
    static const float xs[13] = {
        0.620F, 0.618F, 0.616F, 0.615F, 0.613F, 0.610F, 0.606F,
        0.580F, 0.552F, 0.524F, 0.496F, 0.465F, 0.435F
    };
    static const float ys[13] = {
        0.579F, 0.575F, 0.571F, 0.568F, 0.564F, 0.560F, 0.556F,
        0.546F, 0.534F, 0.525F, 0.519F, 0.517F, 0.516F
    };
    hand_direction_config config;
    hand_direction_recognizer recognizer;
    int recognized = 0;
    size_t index;

    hand_direction_config_init_defaults(&config);
    config.x_scale = 640.0F / 360.0F;
    hand_direction_init(&recognizer, &config);

    for (index = 0U; index < 13U; ++index) {
        hand_point point = {xs[index], ys[index], (uint64_t)index * 45U};
        hand_gesture_event event;

        CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
        if (event.valid) {
            CHECK(event.direction == HAND_DIRECTION_LEFT);
            recognized = 1;
            break;
        }
    }
    CHECK(recognized == 1);
    return 0;
}

static int test_wrist_arc_and_borderline_displacement(void)
{
    hand_direction_config config;
    hand_direction_recognizer recognizer;
    size_t index;

    hand_direction_config_init_defaults(&config);
    config.x_scale = 640.0F / 360.0F;

    /* Natural wrist lift during LEFT swipe (frame 420 style). */
    {
        static const float xs[9] = {
            0.580F, 0.560F, 0.538F, 0.514F, 0.490F,
            0.465F, 0.440F, 0.415F, 0.390F
        };
        static const float ys[9] = {
            0.585F, 0.558F, 0.538F, 0.526F, 0.520F,
            0.518F, 0.517F, 0.516F, 0.516F
        };
        int recognized = 0;

        hand_direction_init(&recognizer, &config);
        for (index = 0U; index < 9U; ++index) {
            hand_point point = {xs[index], ys[index], (uint64_t)index * 45U};
            hand_gesture_event event;

            CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
            if (event.valid) {
                CHECK(event.direction == HAND_DIRECTION_LEFT);
                recognized = 1;
                break;
            }
        }
        CHECK(recognized == 1);
    }

    /* Borderline ~0.137..0.156 scaled displacement RIGHT swipe (frame 1710 style). */
    {
        int recognized = 0;

        hand_direction_init(&recognizer, &config);
        for (index = 0U; index < 9U; ++index) {
            hand_point point = {
                0.420F + 0.011F * (float)index,
                0.300F,
                (uint64_t)index * 45U
            };
            hand_gesture_event event;

            CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
            if (event.valid) {
                CHECK(event.direction == HAND_DIRECTION_RIGHT);
                recognized = 1;
                break;
            }
        }
        CHECK(recognized == 1);
    }
    return 0;
}

static int test_brisk_fast_swipe_4_frames_135ms(void)
{
    hand_direction_recognizer recognizer;
    int recognized = 0;
    size_t index;

    /* Crisp 4-frame swipe (135 ms, ~2.0 norm/s) qualifies via strict fast path. */
    hand_direction_init(&recognizer, NULL);
    for (index = 0U; index < 4U; ++index) {
        hand_point point = {
            0.30F + 0.09F * (float)index,
            0.50F,
            (uint64_t)index * 45U
        };
        hand_gesture_event event;

        CHECK(hand_direction_update(&recognizer, point, &event) >= 0);
        if (event.valid) {
            CHECK(index == 3U);
            CHECK(event.direction == HAND_DIRECTION_RIGHT);
            recognized = 1;
        }
    }
    CHECK(recognized == 1);

    /* Large fast semicircle (radius 0.25 at 45 ms steps) must still reject. */
    for (index = 0U; index < 2U; ++index) {
        size_t p;
        const float dir_sign = index == 0U ? 1.0F : -1.0F;

        hand_direction_init(&recognizer, NULL);
        for (p = 0U; p < 15U; ++p) {
            const float angle = 3.14159265F * (1.0F - (float)p / 14.0F);
            hand_point point = {
                0.50F + 0.25F * cosf(angle),
                0.50F + dir_sign * 0.25F * sinf(angle),
                (uint64_t)p * 45U
            };
            hand_gesture_event event;

            CHECK(hand_direction_update(&recognizer, point, &event) == 0);
            CHECK(event.valid == 0);
        }
    }
    return 0;
}

int main(void)
{
    if (test_four_directions() != 0 ||
        test_min_samples_and_confirmation_required() != 0 ||
        test_high_conf_fast_swipe_single_frame_confirm() != 0 ||
        test_insufficient_displacement() != 0 ||
        test_slow_swipe_is_recognized() != 0 ||
        test_stationary_jitter_is_rejected() != 0 ||
        test_curved_motion_is_rejected() != 0 ||
        test_prefix_offset_then_true_direction() != 0 ||
        test_leading_stationary_dwell_then_swipe() != 0 ||
        test_reversing_segments_rejected() != 0 ||
        test_aspect_ratio_x_scale_symmetry() != 0 ||
        test_hesitation_then_swipe_arc_midpoint() != 0 ||
        test_wrist_arc_and_borderline_displacement() != 0 ||
        test_brisk_fast_swipe_4_frames_135ms() != 0 ||
        test_trajectory_reset_and_expiry() != 0 ||
        test_trajectory_rejects_spike_and_starts_new_stroke() != 0) {
        return 1;
    }

    (void)printf("direction tests passed\n");
    return 0;
}
