#include "hand_direction/tracker.h"

#include <math.h>
#include <stddef.h>

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

static int valid_detection(const hand_detection *detection)
{
    return detection != NULL &&
           isfinite(detection->x0) && isfinite(detection->y0) &&
           isfinite(detection->x1) && isfinite(detection->y1) &&
           isfinite(detection->confidence) &&
           detection->x1 > detection->x0 && detection->y1 > detection->y0;
}

static float intersection_over_union(const hand_detection *a, const hand_detection *b)
{
    const float left = a->x0 > b->x0 ? a->x0 : b->x0;
    const float top = a->y0 > b->y0 ? a->y0 : b->y0;
    const float right = a->x1 < b->x1 ? a->x1 : b->x1;
    const float bottom = a->y1 < b->y1 ? a->y1 : b->y1;
    const float intersection_width = right - left;
    const float intersection_height = bottom - top;
    const float area_a = (a->x1 - a->x0) * (a->y1 - a->y0);
    const float area_b = (b->x1 - b->x0) * (b->y1 - b->y0);
    float intersection_area;
    float union_area;

    if (intersection_width <= 0.0F || intersection_height <= 0.0F) {
        return 0.0F;
    }

    intersection_area = intersection_width * intersection_height;
    union_area = area_a + area_b - intersection_area;
    if (union_area <= 0.0F) {
        return 0.0F;
    }
    return intersection_area / union_area;
}

static float center_distance(const hand_detection *a, const hand_detection *b)
{
    const float ax = 0.5F * (a->x0 + a->x1);
    const float ay = 0.5F * (a->y0 + a->y1);
    const float bx = 0.5F * (b->x0 + b->x1);
    const float by = 0.5F * (b->y0 + b->y1);
    const float dx = bx - ax;
    const float dy = by - ay;

    return sqrtf(dx * dx + dy * dy);
}

void hand_tracker_config_init_defaults(hand_tracker_config *config)
{
    if (config == NULL) {
        return;
    }
    config->ema_alpha = 0.35F;
    config->lost_timeout_ms = 250U;
    config->max_speed_norm_per_second = 3.2F;
    config->gate_margin_norm = 0.05F;
    config->max_step_norm = 0.38F;
}

void hand_tracker_init(hand_tracker *tracker, const hand_tracker_config *config)
{
    if (tracker == NULL) {
        return;
    }

    if (config != NULL) {
        tracker->config = *config;
    } else {
        hand_tracker_config_init_defaults(&tracker->config);
    }

    tracker->config.ema_alpha = clamp_float(tracker->config.ema_alpha, 0.0F, 1.0F);
    if (tracker->config.lost_timeout_ms == 0U) {
        tracker->config.lost_timeout_ms = 250U;
    }
    if (!isfinite(tracker->config.max_speed_norm_per_second) ||
        tracker->config.max_speed_norm_per_second <= 0.0F) {
        tracker->config.max_speed_norm_per_second = 3.2F;
    }
    if (!isfinite(tracker->config.gate_margin_norm) ||
        tracker->config.gate_margin_norm < 0.0F) {
        tracker->config.gate_margin_norm = 0.05F;
    }
    if (!isfinite(tracker->config.max_step_norm) ||
        tracker->config.max_step_norm <= 0.0F) {
        tracker->config.max_step_norm = 0.38F;
    }
    tracker->has_track = 0;
    tracker->current.valid = 0;
}

void hand_tracker_reset(hand_tracker *tracker)
{
    if (tracker == NULL) {
        return;
    }
    tracker->has_track = 0;
    tracker->current.valid = 0;
    tracker->current.timestamp_ms = 0U;
}

int hand_tracker_update(hand_tracker *tracker,
                        const hand_detection *detections,
                        size_t detection_count,
                        uint64_t timestamp_ms,
                        hand_track *track)
{
    size_t best_index = 0U;
    size_t index;
    float best_score = -1.0F;
    int found = 0;
    int has_prior_track;
    const hand_detection *selected;

    if (tracker == NULL || track == NULL || (detection_count > 0U && detections == NULL)) {
        return -1;
    }

    track->valid = 0;
    track->timestamp_ms = timestamp_ms;

    if (tracker->has_track &&
        (timestamp_ms < tracker->current.timestamp_ms ||
         timestamp_ms - tracker->current.timestamp_ms > tracker->config.lost_timeout_ms)) {
        hand_tracker_reset(tracker);
    }

    has_prior_track = tracker->has_track && tracker->current.valid;
    for (index = 0U; index < detection_count; ++index) {
        float score;
        if (!valid_detection(&detections[index])) {
            continue;
        }

        if (has_prior_track) {
            const float distance = center_distance(&tracker->current.smoothed,
                                                   &detections[index]);
            const uint64_t elapsed_ms = timestamp_ms - tracker->current.timestamp_ms;
            const float elapsed_seconds = (float)elapsed_ms / 1000.0F;
            float max_distance = tracker->config.gate_margin_norm +
                tracker->config.max_speed_norm_per_second * elapsed_seconds;

            /* Clamp before scoring: otherwise proximity can score against a gate
             * wider than the hard one-step limit. */
            if (max_distance > tracker->config.max_step_norm) {
                max_distance = tracker->config.max_step_norm;
            }
            if (distance > max_distance) {
                continue;
            }
            {
                const float proximity = 1.0F -
                    clamp_float(distance / max_distance, 0.0F, 1.0F);
                score = 0.35F * detections[index].confidence +
                    0.40F * intersection_over_union(&tracker->current.smoothed,
                                                    &detections[index]) +
                    0.25F * proximity;
            }
        } else {
            score = detections[index].confidence;
        }

        if (!found || score > best_score) {
            best_index = index;
            best_score = score;
            found = 1;
        }
    }

    if (!found) {
        if (tracker->has_track && timestamp_ms >= tracker->current.timestamp_ms &&
            timestamp_ms - tracker->current.timestamp_ms > tracker->config.lost_timeout_ms) {
            hand_tracker_reset(tracker);
        }
        return 0;
    }

    selected = &detections[best_index];
    tracker->current.raw_detection = *selected;
    if (!has_prior_track) {
        tracker->current.smoothed = *selected;
    } else {
        const float distance = center_distance(&tracker->current.smoothed, selected);
        const uint64_t elapsed_ms = timestamp_ms - tracker->current.timestamp_ms;
        const float speed = elapsed_ms > 0U
            ? distance * 1000.0F / (float)elapsed_ms : 0.0F;
        /* Be calm for small detector jitter, but follow genuine fast swipes promptly. */
        const float alpha = clamp_float(tracker->config.ema_alpha +
                                        0.22F * speed, 0.25F, 0.88F);
        hand_detection *smoothed = &tracker->current.smoothed;
        const float old_w = smoothed->x1 - smoothed->x0;
        const float old_h = smoothed->y1 - smoothed->y0;
        const float new_w = selected->x1 - selected->x0;
        const float new_h = selected->y1 - selected->y0;
        const float min_scale_alpha = tracker->config.ema_alpha < 0.15F
            ? tracker->config.ema_alpha : 0.15F;
        float w_ratio = 1.0F;
        float h_ratio = 1.0F;
        float alpha_x = alpha;
        float alpha_y = alpha;

        if (old_w > 1e-6F && new_w > 1e-6F) {
            w_ratio = new_w > old_w ? (new_w / old_w) : (old_w / new_w);
        }
        if (old_h > 1e-6F && new_h > 1e-6F) {
            h_ratio = new_h > old_h ? (new_h / old_h) : (old_h / new_h);
        }
        /* Suppress sudden single-axis bbox inflation/shrinking (e.g. forearm entry)
         * without slowing genuine motion on the other axis. */
        if (w_ratio > 1.25F) {
            alpha_x = clamp_float(tracker->config.ema_alpha / w_ratio,
                                  min_scale_alpha,
                                  tracker->config.ema_alpha);
        }
        if (h_ratio > 1.25F) {
            alpha_y = clamp_float(tracker->config.ema_alpha / h_ratio,
                                  min_scale_alpha,
                                  tracker->config.ema_alpha);
        }

        smoothed->x0 += alpha_x * (selected->x0 - smoothed->x0);
        smoothed->y0 += alpha_y * (selected->y0 - smoothed->y0);
        smoothed->x1 += alpha_x * (selected->x1 - smoothed->x1);
        smoothed->y1 += alpha_y * (selected->y1 - smoothed->y1);
        smoothed->confidence += alpha * (selected->confidence - smoothed->confidence);
        smoothed->class_id = selected->class_id;
    }

    tracker->has_track = 1;
    tracker->current.valid = 1;
    tracker->current.timestamp_ms = timestamp_ms;
    *track = tracker->current;
    return 0;
}
