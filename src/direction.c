#include "hand_direction/direction.h"

#include <math.h>
#include <stddef.h>
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

void hand_direction_config_init_defaults(hand_direction_config *config)
{
    if (config == NULL) {
        return;
    }

    config->window_ms = 700U;
    config->max_gap_ms = 250U;
    config->min_samples = 6U;
    config->min_span_ms = 220U;
    config->min_displacement = 0.12F;
    config->min_speed_per_second = 0.20F;
    config->min_linearity = 0.82F;
    config->axis_ratio = 1.65F;
    config->x_scale = 1.0F;
    config->min_segment_consistency = 0.75F;
    config->min_half_cosine = 0.65F;
    config->min_moving_segments = 4U;
    config->confirm_count = 2U;
}

static void reset_candidate_state(hand_direction_recognizer *recognizer)
{
    recognizer->candidate_direction = HAND_DIRECTION_NONE;
    recognizer->candidate_streak = 0U;
}

static void reset_diagnostics(hand_direction_diagnostics *diagnostics)
{
    if (diagnostics != NULL) {
        (void)memset(diagnostics, 0, sizeof(*diagnostics));
        diagnostics->candidate_direction = HAND_DIRECTION_NONE;
        diagnostics->emitted_direction = HAND_DIRECTION_NONE;
        diagnostics->reject_reason = "none";
    }
}

void hand_direction_init(hand_direction_recognizer *recognizer,
                         const hand_direction_config *config)
{
    if (recognizer == NULL) {
        return;
    }

    if (config != NULL) {
        recognizer->config = *config;
    } else {
        hand_direction_config_init_defaults(&recognizer->config);
    }

    if (recognizer->config.window_ms == 0U) {
        recognizer->config.window_ms = 700U;
    }
    if (recognizer->config.max_gap_ms == 0U ||
        recognizer->config.max_gap_ms > recognizer->config.window_ms) {
        recognizer->config.max_gap_ms = recognizer->config.window_ms;
    }
    if (recognizer->config.min_samples < 2U) {
        recognizer->config.min_samples = 2U;
    }
    if (recognizer->config.min_span_ms < 1U) {
        recognizer->config.min_span_ms = 1U;
    }
    if (!isfinite(recognizer->config.min_displacement)) {
        recognizer->config.min_displacement = 0.12F;
    }
    recognizer->config.min_displacement =
        clamp_float(recognizer->config.min_displacement, 0.0F, 4.0F);
    if (!isfinite(recognizer->config.min_speed_per_second)) {
        recognizer->config.min_speed_per_second = 0.20F;
    }
    recognizer->config.min_speed_per_second =
        clamp_float(recognizer->config.min_speed_per_second, 0.0F, 100.0F);
    if (!isfinite(recognizer->config.min_linearity)) {
        recognizer->config.min_linearity = 0.82F;
    }
    recognizer->config.min_linearity =
        clamp_float(recognizer->config.min_linearity, 0.0F, 1.0F);
    if (!isfinite(recognizer->config.axis_ratio) ||
        recognizer->config.axis_ratio < 1.0F) {
        recognizer->config.axis_ratio = 1.0F;
    }
    if (!isfinite(recognizer->config.x_scale) ||
        recognizer->config.x_scale <= 0.0F) {
        recognizer->config.x_scale = 1.0F;
    }
    if (!isfinite(recognizer->config.min_segment_consistency)) {
        recognizer->config.min_segment_consistency = 0.75F;
    }
    recognizer->config.min_segment_consistency =
        clamp_float(recognizer->config.min_segment_consistency, 0.0F, 1.0F);
    if (!isfinite(recognizer->config.min_half_cosine)) {
        recognizer->config.min_half_cosine = 0.65F;
    }
    recognizer->config.min_half_cosine =
        clamp_float(recognizer->config.min_half_cosine, -1.0F, 1.0F);
    if (recognizer->config.min_moving_segments < 1U) {
        recognizer->config.min_moving_segments = 1U;
    }
    if (recognizer->config.confirm_count < 1U) {
        recognizer->config.confirm_count = 1U;
    }

    hand_direction_reset(recognizer);
}

void hand_direction_reset(hand_direction_recognizer *recognizer)
{
    if (recognizer != NULL) {
        hand_trajectory_reset(&recognizer->trajectory);
        reset_candidate_state(recognizer);
        reset_diagnostics(&recognizer->last_diagnostics);
    }
}

static size_t trim_leading_dwell(const hand_direction_recognizer *recognizer)
{
    size_t start_idx = 0U;
    const size_t count = recognizer->trajectory.count;
    const hand_point *last = hand_trajectory_at(&recognizer->trajectory, count - 1U);

    if (last == NULL) {
        return 0U;
    }

    while (count - start_idx > recognizer->config.min_samples) {
        const hand_point *p0 = &recognizer->trajectory.points[start_idx];
        const hand_point *p1 = &recognizer->trajectory.points[start_idx + 1U];
        const float sdx = (p1->x - p0->x) * recognizer->config.x_scale;
        const float sdy = p1->y - p0->y;
        const float step_dist = sqrtf(sdx * sdx + sdy * sdy);
        const uint64_t step_dt_ms = p1->timestamp_ms > p0->timestamp_ms
            ? (p1->timestamp_ms - p0->timestamp_ms) : 0U;
        const float step_speed = step_dt_ms > 0U
            ? step_dist / ((float)step_dt_ms / 1000.0F) : 0.0F;

        if (last->timestamp_ms <= p1->timestamp_ms ||
            last->timestamp_ms - p1->timestamp_ms < recognizer->config.min_span_ms) {
            break;
        }
        if (step_dist <= 0.006F ||
            step_speed < 0.5F * recognizer->config.min_speed_per_second) {
            ++start_idx;
        } else {
            break;
        }
    }
    return start_idx;
}

static float trajectory_path_length_scaled(const hand_trajectory *trajectory,
                                           size_t start_idx,
                                           float x_scale)
{
    float length = 0.0F;
    size_t index;

    for (index = start_idx + 1U; index < trajectory->count; ++index) {
        const float dx = (trajectory->points[index].x - trajectory->points[index - 1U].x) * x_scale;
        const float dy = trajectory->points[index].y - trajectory->points[index - 1U].y;
        length += sqrtf(dx * dx + dy * dy);
    }
    return length;
}

static size_t find_arclength_midpoint(const hand_trajectory *trajectory,
                                      size_t start_idx,
                                      float path_length,
                                      float x_scale)
{
    const size_t count = trajectory->count;
    const float half_length = 0.5F * path_length;
    float accum = 0.0F;
    size_t index;

    if (count <= start_idx + 2U) {
        return start_idx;
    }

    for (index = start_idx + 1U; index < count - 1U; ++index) {
        const float dx = (trajectory->points[index].x - trajectory->points[index - 1U].x) * x_scale;
        const float dy = trajectory->points[index].y - trajectory->points[index - 1U].y;

        accum += sqrtf(dx * dx + dy * dy);
        if (accum >= half_length) {
            return index;
        }
    }
    return count - 2U;
}

static void project_vector_onto_direction(hand_direction_code direction,
                                          float vx,
                                          float vy,
                                          float *major_delta,
                                          float *minor_abs)
{
    if (direction == HAND_DIRECTION_RIGHT || direction == HAND_DIRECTION_LEFT) {
        const float sign = direction == HAND_DIRECTION_RIGHT ? 1.0F : -1.0F;
        *major_delta = sign * vx;
        *minor_abs = fabsf(vy);
    } else {
        const float sign = direction == HAND_DIRECTION_DOWN ? 1.0F : -1.0F;
        *major_delta = sign * vy;
        *minor_abs = fabsf(vx);
    }
}

static int reject_update(hand_direction_recognizer *recognizer,
                         hand_direction_diagnostics *diag,
                         const char *reason)
{
    reset_candidate_state(recognizer);
    diag->candidate_direction = HAND_DIRECTION_NONE;
    diag->candidate_streak = 0U;
    diag->reject_reason = reason;
    recognizer->last_diagnostics = *diag;
    return 0;
}

int hand_direction_update(hand_direction_recognizer *recognizer,
                          hand_point point,
                          hand_gesture_event *event)
{
    size_t prev_count;
    size_t start_idx;
    size_t count;
    const hand_point *start;
    const hand_point *end;
    const hand_point *mid;
    hand_direction_diagnostics diag;
    size_t min_eval_samples;
    int is_short_fast_candidate;
    uint64_t req_min_span_ms;
    float req_min_disp;
    float req_min_speed;
    float req_min_linearity;
    float req_axis_ratio;
    float req_min_half_cosine;
    float req_tail_ratio;
    uint32_t req_min_moving_segs;
    float dx;
    float dy;
    float abs_dx;
    float abs_dy;
    float displacement;
    uint64_t span_u64;
    float span_ms;
    float speed;
    float path_length;
    float linearity;
    float minor_axis;
    float major_axis;
    float v1x;
    float v1y;
    float v2x;
    float v2y;
    float len1;
    float len2;
    float half_cosine;
    float v2_major;
    float v2_minor;
    uint32_t moving_segments = 0U;
    uint32_t consistent_segments = 0U;
    float forward_major = 0.0F;
    float backward_major = 0.0F;
    float consistency = 0.0F;
    float axis_score;
    float displacement_score;
    float confidence;
    hand_direction_code direction;
    size_t idx;

    if (recognizer == NULL || event == NULL) {
        return -1;
    }

    (void)memset(event, 0, sizeof(*event));
    reset_diagnostics(&diag);
    prev_count = recognizer->trajectory.count;
    if (hand_trajectory_push(&recognizer->trajectory,
                             point,
                             recognizer->config.max_gap_ms,
                             recognizer->config.window_ms) != 0) {
        reset_candidate_state(recognizer);
        diag.point_count = recognizer->trajectory.count;
        diag.active_point_count = recognizer->trajectory.count;
        diag.reject_reason = "invalid_point";
        recognizer->last_diagnostics = diag;
        return -1;
    }

    if (prev_count > 0U && recognizer->trajectory.count == 1U) {
        reset_candidate_state(recognizer);
    }

    count = recognizer->trajectory.count;
    diag.point_count = count;
    diag.active_point_count = count;
    min_eval_samples = recognizer->config.min_samples < 4U
        ? (size_t)recognizer->config.min_samples : 4U;
    if (count < min_eval_samples) {
        return reject_update(recognizer, &diag, "min_samples");
    }

    start_idx = trim_leading_dwell(recognizer);
    diag.active_point_count = count - start_idx;

    start = hand_trajectory_at(&recognizer->trajectory, start_idx);
    end = hand_trajectory_at(&recognizer->trajectory, count - 1U);
    if (start == NULL || end == NULL || end->timestamp_ms <= start->timestamp_ms) {
        return reject_update(recognizer, &diag, "invalid_span");
    }

    span_u64 = end->timestamp_ms - start->timestamp_ms;
    diag.span_ms = span_u64;
    span_ms = (float)span_u64;

    is_short_fast_candidate =
        (diag.active_point_count < (size_t)recognizer->config.min_samples ||
         span_u64 < (uint64_t)recognizer->config.min_span_ms);

    if (is_short_fast_candidate) {
        req_min_span_ms = recognizer->config.min_span_ms < 130U
            ? recognizer->config.min_span_ms : 130U;
        req_min_disp = recognizer->config.min_displacement > 0.20F
            ? recognizer->config.min_displacement : 0.20F;
        req_min_speed = recognizer->config.min_speed_per_second > 1.10F
            ? recognizer->config.min_speed_per_second : 1.10F;
        req_min_linearity = recognizer->config.min_linearity > 0.94F
            ? recognizer->config.min_linearity : 0.94F;
        req_axis_ratio = recognizer->config.axis_ratio > 2.20F
            ? recognizer->config.axis_ratio : 2.20F;
        req_min_half_cosine = recognizer->config.min_half_cosine > 0.90F
            ? recognizer->config.min_half_cosine : 0.90F;
        req_tail_ratio = 1.35F;
        req_min_moving_segs = 3U;
    } else {
        req_min_span_ms = recognizer->config.min_span_ms;
        req_min_disp = recognizer->config.min_displacement;
        req_min_speed = recognizer->config.min_speed_per_second;
        req_min_linearity = recognizer->config.min_linearity;
        req_axis_ratio = recognizer->config.axis_ratio;
        req_min_half_cosine = recognizer->config.min_half_cosine;
        req_tail_ratio = 1.15F;
        req_min_moving_segs = recognizer->config.min_moving_segments;
    }

    if (span_u64 < req_min_span_ms) {
        return reject_update(recognizer, &diag, "min_span");
    }

    dx = (end->x - start->x) * recognizer->config.x_scale;
    dy = end->y - start->y;
    abs_dx = fabsf(dx);
    abs_dy = fabsf(dy);
    displacement = sqrtf(dx * dx + dy * dy);
    diag.dx = dx;
    diag.dy = dy;
    diag.displacement = displacement;
    if (displacement < req_min_disp) {
        return reject_update(recognizer, &diag, "min_displacement");
    }

    speed = displacement / (span_ms / 1000.0F);
    if (speed < req_min_speed) {
        return reject_update(recognizer, &diag, "min_speed");
    }

    path_length = trajectory_path_length_scaled(&recognizer->trajectory,
                                                start_idx,
                                                recognizer->config.x_scale);
    diag.path_length = path_length;
    if (path_length <= 0.0F) {
        return reject_update(recognizer, &diag, "zero_path");
    }

    linearity = displacement / path_length;
    diag.linearity = linearity;
    if (linearity < req_min_linearity) {
        return reject_update(recognizer, &diag, "min_linearity");
    }

    major_axis = abs_dx > abs_dy ? abs_dx : abs_dy;
    minor_axis = abs_dx > abs_dy ? abs_dy : abs_dx;
    diag.axis_ratio = minor_axis > 0.0F
        ? (major_axis / minor_axis)
        : (major_axis > 0.0F ? 999.0F : 0.0F);
    if (abs_dx >= abs_dy * req_axis_ratio) {
        direction = dx >= 0.0F ? HAND_DIRECTION_RIGHT : HAND_DIRECTION_LEFT;
    } else if (abs_dy >= abs_dx * req_axis_ratio) {
        direction = dy >= 0.0F ? HAND_DIRECTION_DOWN : HAND_DIRECTION_UP;
    } else {
        return reject_update(recognizer, &diag, "axis_ratio");
    }

    mid = hand_trajectory_at(&recognizer->trajectory,
                             find_arclength_midpoint(&recognizer->trajectory,
                                                     start_idx,
                                                     path_length,
                                                     recognizer->config.x_scale));
    if (mid == NULL) {
        return reject_update(recognizer, &diag, "invalid_mid");
    }
    v1x = (mid->x - start->x) * recognizer->config.x_scale;
    v1y = mid->y - start->y;
    v2x = (end->x - mid->x) * recognizer->config.x_scale;
    v2y = end->y - mid->y;
    len1 = sqrtf(v1x * v1x + v1y * v1y);
    len2 = sqrtf(v2x * v2x + v2y * v2y);
    half_cosine = (len1 > 1e-6F && len2 > 1e-6F)
        ? ((v1x * v2x + v1y * v2y) / (len1 * len2))
        : 0.0F;
    diag.half_cosine = half_cosine;
    if (len1 <= 1e-6F || len2 <= 1e-6F || half_cosine < req_min_half_cosine) {
        return reject_update(recognizer, &diag, "half_cosine");
    }

    project_vector_onto_direction(direction, v2x, v2y, &v2_major, &v2_minor);
    if (v2_major <= 0.0F || v2_major < v2_minor * req_tail_ratio) {
        return reject_update(recognizer, &diag, "tail_direction");
    }

    idx = count - 1U;
    while (idx > start_idx && moving_segments < 7U) {
        const hand_point *p_curr = &recognizer->trajectory.points[idx];
        const hand_point *p_prev = &recognizer->trajectory.points[idx - 1U];
        const float seg_dx = (p_curr->x - p_prev->x) * recognizer->config.x_scale;
        const float seg_dy = p_curr->y - p_prev->y;
        const float seg_len = sqrtf(seg_dx * seg_dx + seg_dy * seg_dy);

        if (seg_len >= 0.004F) {
            float seg_major;
            float seg_minor;

            project_vector_onto_direction(direction, seg_dx, seg_dy, &seg_major, &seg_minor);
            ++moving_segments;
            if (seg_major > 0.0F) {
                forward_major += seg_major;
                if (seg_major >= seg_minor) {
                    ++consistent_segments;
                }
            } else {
                backward_major += -seg_major;
            }
        }
        --idx;
    }

    consistency = moving_segments > 0U
        ? ((float)consistent_segments / (float)moving_segments)
        : 0.0F;
    diag.moving_segments = moving_segments;
    diag.consistent_segments = consistent_segments;
    diag.segment_consistency = consistency;

    if (moving_segments < req_min_moving_segs) {
        return reject_update(recognizer, &diag, "min_moving_segments");
    }
    if (is_short_fast_candidate
            ? (consistent_segments < moving_segments)
            : (consistency < recognizer->config.min_segment_consistency)) {
        return reject_update(recognizer, &diag, "segment_consistency");
    }
    if (is_short_fast_candidate
            ? (backward_major > 0.0F)
            : (backward_major > 0.15F * forward_major)) {
        return reject_update(recognizer, &diag, "reversing_segments");
    }

    if (recognizer->candidate_direction == direction) {
        ++recognizer->candidate_streak;
    } else {
        recognizer->candidate_direction = direction;
        recognizer->candidate_streak = 1U;
    }
    diag.candidate_direction = recognizer->candidate_direction;
    diag.candidate_streak = recognizer->candidate_streak;

    {
        uint32_t effective_confirm = recognizer->config.confirm_count;

        if (is_short_fast_candidate ||
            (displacement >= 0.22F &&
             linearity >= 0.90F &&
             consistency >= 0.85F &&
             moving_segments >= 5U &&
             half_cosine >= 0.80F)) {
            effective_confirm = 1U;
        }
        if (recognizer->candidate_streak < effective_confirm) {
            diag.reject_reason = "confirm_pending";
            recognizer->last_diagnostics = diag;
            return 0;
        }
    }

    axis_score = major_axis > 0.0F ? 1.0F - minor_axis / (major_axis + minor_axis) : 0.0F;
    displacement_score = clamp_float(displacement /
                                         (2.0F * recognizer->config.min_displacement),
                                     0.0F,
                                     1.0F);
    confidence = clamp_float((linearity + axis_score + displacement_score + consistency) / 4.0F,
                             0.0F,
                             1.0F);

    event->valid = 1;
    event->direction = direction;
    event->confidence = confidence;
    event->x = end->x;
    event->y = end->y;
    event->timestamp_ms = end->timestamp_ms;

    diag.emitted_direction = direction;
    diag.reject_reason = "emitted";
    recognizer->last_diagnostics = diag;
    reset_candidate_state(recognizer);
    return 1;
}

void hand_direction_get_diagnostics(const hand_direction_recognizer *recognizer,
                                    hand_direction_diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }
    if (recognizer == NULL) {
        reset_diagnostics(diagnostics);
        return;
    }
    *diagnostics = recognizer->last_diagnostics;
}

const char *hand_direction_name(hand_direction_code direction)
{
    switch (direction) {
    case HAND_DIRECTION_LEFT:
        return "LEFT";
    case HAND_DIRECTION_RIGHT:
        return "RIGHT";
    case HAND_DIRECTION_UP:
        return "UP";
    case HAND_DIRECTION_DOWN:
        return "DOWN";
    case HAND_DIRECTION_NONE:
    default:
        return "NONE";
    }
}
