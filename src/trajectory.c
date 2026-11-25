#include "hand_direction/trajectory.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TRAJECTORY_GATE_MARGIN_NORM 0.05F
#define TRAJECTORY_MAX_SPEED_NORM_PER_SECOND 3.2F
#define TRAJECTORY_MAX_STEP_NORM 0.38F

static int valid_point(const hand_point *point)
{
    return point != NULL &&
           isfinite(point->x) && isfinite(point->y) &&
           point->x >= 0.0F && point->x <= 1.0F &&
           point->y >= 0.0F && point->y <= 1.0F;
}

static void remove_front(hand_trajectory *trajectory, size_t count)
{
    if (count == 0U || count > trajectory->count) {
        return;
    }

    trajectory->count -= count;
    if (trajectory->count > 0U) {
        (void)memmove(&trajectory->points[0],
                      &trajectory->points[count],
                      trajectory->count * sizeof(trajectory->points[0]));
    }
}

void hand_trajectory_reset(hand_trajectory *trajectory)
{
    if (trajectory != NULL) {
        trajectory->count = 0U;
    }
}

int hand_trajectory_push(hand_trajectory *trajectory,
                         hand_point point,
                         uint64_t max_gap_ms,
                         uint64_t window_ms)
{
    size_t expired_count = 0U;

    if (trajectory == NULL || !valid_point(&point)) {
        return -1;
    }

    if (trajectory->count > 0U) {
        const hand_point *last = &trajectory->points[trajectory->count - 1U];
        const uint64_t gap_ms = point.timestamp_ms >= last->timestamp_ms
                                    ? point.timestamp_ms - last->timestamp_ms
                                    : 0U;

        if (point.timestamp_ms < last->timestamp_ms ||
            (max_gap_ms > 0U && gap_ms > max_gap_ms)) {
            hand_trajectory_reset(trajectory);
        } else {
            const float dx = point.x - last->x;
            const float dy = point.y - last->y;
            const float distance = sqrtf(dx * dx + dy * dy);
            const float elapsed_seconds = (float)gap_ms / 1000.0F;
            /* Reject isolated detector jumps; a rejected point starts a fresh stroke. */
            float max_step = TRAJECTORY_GATE_MARGIN_NORM +
                TRAJECTORY_MAX_SPEED_NORM_PER_SECOND * elapsed_seconds;
            if (max_step > TRAJECTORY_MAX_STEP_NORM) {
                max_step = TRAJECTORY_MAX_STEP_NORM;
            }
            if (distance > max_step) {
                hand_trajectory_reset(trajectory);
            }
        }
        if (trajectory->count > 0U && window_ms > 0U) {
            while (expired_count < trajectory->count &&
                   point.timestamp_ms >= trajectory->points[expired_count].timestamp_ms &&
                   point.timestamp_ms - trajectory->points[expired_count].timestamp_ms >
                       window_ms) {
                ++expired_count;
            }
            remove_front(trajectory, expired_count);
        }
    }

    if (trajectory->count == HAND_TRAJECTORY_CAPACITY) {
        remove_front(trajectory, 1U);
    }

    trajectory->points[trajectory->count] = point;
    ++trajectory->count;
    return 0;
}

size_t hand_trajectory_count(const hand_trajectory *trajectory)
{
    return trajectory != NULL ? trajectory->count : 0U;
}

const hand_point *hand_trajectory_at(const hand_trajectory *trajectory, size_t index)
{
    if (trajectory == NULL || index >= trajectory->count) {
        return NULL;
    }
    return &trajectory->points[index];
}
