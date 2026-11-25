#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include "hand_direction/direction.h"
#include "hand_direction/gesture_engine.h"
#include "hand_direction/trajectory.h"
#include "hand_direction/types.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define BENCH_DEFAULT_ROI_X_MIN 0.3296875F
#define BENCH_DEFAULT_ROI_X_MAX 0.6671875F
#define BENCH_DEFAULT_ROI_Y_MIN 0.200000F
#define BENCH_DEFAULT_ROI_Y_MAX 0.800000F
#define BENCH_DEFAULT_X_SCALE (640.0F / 360.0F)
#define BENCH_DEFAULT_WINDOW_MS 700U
#define BENCH_COOLDOWN_MS 2000U
#define BENCH_SESSION_GAP_MS 1200U
#define BENCH_CLASSES 5U

typedef struct {
    uint64_t timestamp_ms;
    size_t detection_count;
    hand_detection detections[HAND_DETECTION_MAX_COUNT];
} bench_frame;

typedef struct {
    char name[96];
    hand_direction_code expected;
    size_t expected_trials;
    int has_explicit_onset;
    uint64_t explicit_onset_ms;
    bench_frame *frames;
    size_t frame_count;
    size_t frame_cap;
} bench_clip;

typedef struct {
    bench_clip *clips;
    size_t clip_count;
    size_t clip_cap;
    float roi_x_min;
    float roi_y_min;
    float roi_x_max;
    float roi_y_max;
    int has_roi_override;
} bench_suite;

typedef struct {
    char name[96];
    hand_direction_code expected;
    size_t expected_trials;
    double duration_sec;
    size_t matched;
    size_t opposite_return;
    size_t other_wrong;
} bench_stream_stat;

typedef struct {
    uint32_t confusion[BENCH_CLASSES][BENCH_CLASSES];
    int stream_mode;
    size_t total_clips;
    size_t pos_clips;
    size_t pos_correct;
    size_t pos_missed;
    size_t pos_wrong_dir;
    size_t opposite_return_fp;
    size_t other_wrong_fp;
    size_t neg_clips;
    size_t neg_false_triggers;
    size_t duplicate_events;
    double pos_duration_sec;
    double neg_duration_sec;
    bench_stream_stat *stream_stats;
    size_t stream_stat_count;
    uint64_t *latencies_ms;
    size_t latency_count;
    size_t latency_cap;
} bench_metrics;

/* ============================================================================
 * Historical Baseline Engine (legacy_v1, matching CV610 Experiment Report s5)
 * ============================================================================ */

typedef struct {
    int has_track;
    hand_detection smoothed;
    hand_detection raw_detection;
    uint64_t timestamp_ms;
} legacy_v1_tracker;

typedef struct {
    legacy_v1_tracker tracker;
    hand_trajectory trajectory;
    int state; /* 0=WAIT_HAND, 1=WAIT_ACT, 2=CAPTURING, 3=COOLDOWN */
    int has_cooldown;
    uint64_t cooldown_start_ms;
    uint64_t last_event_span_ms;
    float roi_x_min;
    float roi_y_min;
    float roi_x_max;
    float roi_y_max;
} legacy_v1_engine;

static float bench_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float det_iou(const hand_detection *a, const hand_detection *b)
{
    const float left = a->x0 > b->x0 ? a->x0 : b->x0;
    const float top = a->y0 > b->y0 ? a->y0 : b->y0;
    const float right = a->x1 < b->x1 ? a->x1 : b->x1;
    const float bottom = a->y1 < b->y1 ? a->y1 : b->y1;
    const float iw = right - left;
    const float ih = bottom - top;
    const float area_a = (a->x1 - a->x0) * (a->y1 - a->y0);
    const float area_b = (b->x1 - b->x0) * (b->y1 - b->y0);
    float inter;
    float uni;

    if (iw <= 0.0F || ih <= 0.0F) {
        return 0.0F;
    }
    inter = iw * ih;
    uni = area_a + area_b - inter;
    return uni > 0.0F ? inter / uni : 0.0F;
}

static float det_center_dist(const hand_detection *a, const hand_detection *b)
{
    const float ax = 0.5F * (a->x0 + a->x1);
    const float ay = 0.5F * (a->y0 + a->y1);
    const float bx = 0.5F * (b->x0 + b->x1);
    const float by = 0.5F * (b->y0 + b->y1);
    const float dx = bx - ax;
    const float dy = by - ay;
    return sqrtf(dx * dx + dy * dy);
}

static void legacy_v1_init(legacy_v1_engine *eng,
                           float roi_x_min,
                           float roi_y_min,
                           float roi_x_max,
                           float roi_y_max)
{
    (void)memset(eng, 0, sizeof(*eng));
    hand_trajectory_reset(&eng->trajectory);
    eng->roi_x_min = roi_x_min;
    eng->roi_y_min = roi_y_min;
    eng->roi_x_max = roi_x_max;
    eng->roi_y_max = roi_y_max;
}

static int legacy_v1_in_roi(const legacy_v1_engine *eng, float x, float y)
{
    return x >= eng->roi_x_min && x <= eng->roi_x_max &&
           y >= eng->roi_y_min && y <= eng->roi_y_max;
}

static float legacy_v1_path_length(const hand_trajectory *traj)
{
    float plen = 0.0F;
    size_t i;

    for (i = 1U; i < traj->count; ++i) {
        const float dx = traj->points[i].x - traj->points[i - 1U].x;
        const float dy = traj->points[i].y - traj->points[i - 1U].y;
        plen += sqrtf(dx * dx + dy * dy);
    }
    return plen;
}

static void legacy_v1_update(legacy_v1_engine *eng,
                             const hand_detection *detections,
                             size_t detection_count,
                             uint64_t timestamp_ms,
                             hand_gesture_event *event)
{
    size_t best_idx = 0U;
    float best_score = -1.0F;
    int found = 0;
    size_t i;
    const hand_detection *sel;
    hand_point pt;

    (void)memset(event, 0, sizeof(*event));

    if (eng->has_cooldown &&
        (timestamp_ms < eng->cooldown_start_ms ||
         timestamp_ms - eng->cooldown_start_ms >= BENCH_COOLDOWN_MS)) {
        eng->has_cooldown = 0;
        eng->cooldown_start_ms = 0U;
    }

    if (eng->tracker.has_track &&
        (timestamp_ms < eng->tracker.timestamp_ms ||
         timestamp_ms - eng->tracker.timestamp_ms > 180U)) {
        eng->tracker.has_track = 0;
    }

    for (i = 0U; i < detection_count; ++i) {
        const hand_detection *d = &detections[i];
        float score;

        if (!isfinite(d->x0) || !isfinite(d->y0) ||
            !isfinite(d->x1) || !isfinite(d->y1) ||
            !isfinite(d->confidence) || d->x1 <= d->x0 || d->y1 <= d->y0) {
            continue;
        }
        if (eng->tracker.has_track) {
            const uint64_t dt_ms = timestamp_ms - eng->tracker.timestamp_ms;
            const float dist = det_center_dist(&eng->tracker.smoothed, d);
            float max_d = 0.035F + 1.8F * ((float)dt_ms / 1000.0F);
            float prox;
            if (max_d > 0.24F) {
                max_d = 0.24F;
            }
            if (dist > max_d) {
                continue;
            }
            prox = 1.0F - bench_clampf(dist / max_d, 0.0F, 1.0F);
            score = 0.35F * d->confidence +
                    0.40F * det_iou(&eng->tracker.smoothed, d) +
                    0.25F * prox;
        } else {
            score = d->confidence;
        }
        if (!found || score > best_score) {
            best_score = score;
            best_idx = i;
            found = 1;
        }
    }

    if (!found) {
        /* Pre-dirv2 reset on missed detection when outside active cooldown */
        eng->tracker.has_track = 0;
        hand_trajectory_reset(&eng->trajectory);
        eng->state = eng->has_cooldown ? 3 : 0;
        return;
    }

    sel = &detections[best_idx];
    eng->tracker.raw_detection = *sel;
    if (!eng->tracker.has_track) {
        eng->tracker.smoothed = *sel;
        eng->tracker.has_track = 1;
    } else {
        const uint64_t dt_ms = timestamp_ms - eng->tracker.timestamp_ms;
        const float dist = det_center_dist(&eng->tracker.smoothed, sel);
        const float speed = dt_ms > 0U ? dist * 1000.0F / (float)dt_ms : 0.0F;
        const float alpha = bench_clampf(0.35F + 0.10F * speed, 0.25F, 0.78F);
        hand_detection *sm = &eng->tracker.smoothed;

        sm->x0 += alpha * (sel->x0 - sm->x0);
        sm->y0 += alpha * (sel->y0 - sm->y0);
        sm->x1 += alpha * (sel->x1 - sm->x1);
        sm->y1 += alpha * (sel->y1 - sm->y1);
        sm->confidence += alpha * (sel->confidence - sm->confidence);
        sm->class_id = sel->class_id;
    }
    eng->tracker.timestamp_ms = timestamp_ms;

    if (eng->has_cooldown) {
        eng->state = 3;
        hand_trajectory_reset(&eng->trajectory);
        return;
    }
    if (eng->state == 3) {
        eng->state = 1;
        hand_trajectory_reset(&eng->trajectory);
        return;
    }

    {
        const float raw_cx = 0.5F * (sel->x0 + sel->x1);
        const float raw_cy = 0.5F * (sel->y0 + sel->y1);
        if (!legacy_v1_in_roi(eng, raw_cx, raw_cy)) {
            hand_trajectory_reset(&eng->trajectory);
            eng->state = 1;
            return;
        }
    }

    if (eng->state <= 1) {
        if (eng->tracker.smoothed.class_id != 0 ||
            eng->tracker.smoothed.confidence < 0.60F) {
            eng->state = 1;
            return;
        }
        eng->state = 2;
        hand_trajectory_reset(&eng->trajectory);
    }

    pt.x = 0.5F * (eng->tracker.smoothed.x0 + eng->tracker.smoothed.x1);
    pt.y = 0.5F * (eng->tracker.smoothed.y0 + eng->tracker.smoothed.y1);
    pt.timestamp_ms = timestamp_ms;
    (void)hand_trajectory_push(&eng->trajectory, pt, 250U, 667U);

    if (eng->trajectory.count >= 5U) {
        const hand_point *first = hand_trajectory_at(&eng->trajectory, 0U);
        const hand_point *last =
            hand_trajectory_at(&eng->trajectory, eng->trajectory.count - 1U);
        if (first != NULL && last != NULL && last->timestamp_ms > first->timestamp_ms) {
            const uint64_t span_ms = last->timestamp_ms - first->timestamp_ms;
            if (span_ms >= 250U) {
                const float dx = last->x - first->x; /* unscaled x_scale = 1.0 */
                const float dy = last->y - first->y;
                const float disp = sqrtf(dx * dx + dy * dy);
                const float speed = disp / ((float)span_ms / 1000.0F);
                const float plen = legacy_v1_path_length(&eng->trajectory);
                const float lin = plen > 0.0F ? disp / plen : 0.0F;
                const float adx = fabsf(dx);
                const float ady = fabsf(dy);

                if (disp >= 0.10F && speed >= 0.20F && lin >= 0.80F) {
                    hand_direction_code dir = HAND_DIRECTION_NONE;
                    if (adx >= ady * 1.50F) {
                        dir = dx >= 0.0F ? HAND_DIRECTION_RIGHT : HAND_DIRECTION_LEFT;
                    } else if (ady >= adx * 1.50F) {
                        dir = dy >= 0.0F ? HAND_DIRECTION_DOWN : HAND_DIRECTION_UP;
                    }
                    if (dir != HAND_DIRECTION_NONE) {
                        event->valid = 1;
                        event->direction = dir;
                        event->confidence = lin;
                        event->x = last->x;
                        event->y = last->y;
                        event->timestamp_ms = timestamp_ms;
                        eng->last_event_span_ms = span_ms;
                        eng->state = 3;
                        eng->has_cooldown = 1;
                        eng->cooldown_start_ms = timestamp_ms;
                        hand_trajectory_reset(&eng->trajectory);
                    }
                }
            }
        }
    }
}

/* ============================================================================
 * Suite / Memory Management
 * ============================================================================ */

static void bench_suite_init(bench_suite *suite)
{
    (void)memset(suite, 0, sizeof(*suite));
    suite->roi_x_min = BENCH_DEFAULT_ROI_X_MIN;
    suite->roi_y_min = BENCH_DEFAULT_ROI_Y_MIN;
    suite->roi_x_max = BENCH_DEFAULT_ROI_X_MAX;
    suite->roi_y_max = BENCH_DEFAULT_ROI_Y_MAX;
}

static void bench_suite_free(bench_suite *suite)
{
    size_t i;
    if (suite == NULL) return;
    for (i = 0U; i < suite->clip_count; ++i) {
        free(suite->clips[i].frames);
    }
    free(suite->clips);
    suite->clips = NULL;
    suite->clip_count = 0U;
    suite->clip_cap = 0U;
}

static bench_clip *bench_suite_add_clip(bench_suite *suite,
                                        const char *name,
                                        hand_direction_code expected)
{
    bench_clip *clip;

    if (suite->clip_count == suite->clip_cap) {
        const size_t new_cap = suite->clip_cap == 0U ? 32U : suite->clip_cap * 2U;
        bench_clip *grown =
            (bench_clip *)realloc(suite->clips, new_cap * sizeof(*grown));
        if (grown == NULL) {
            return NULL;
        }
        suite->clips = grown;
        suite->clip_cap = new_cap;
    }
    clip = &suite->clips[suite->clip_count++];
    (void)memset(clip, 0, sizeof(*clip));
    (void)snprintf(clip->name, sizeof(clip->name), "%s", name != NULL ? name : "clip");
    clip->expected = expected;
    return clip;
}

static int bench_clip_add_frame(bench_clip *clip, const bench_frame *frame)
{
    if (clip->frame_count == clip->frame_cap) {
        const size_t new_cap = clip->frame_cap == 0U ? 32U : clip->frame_cap * 2U;
        bench_frame *grown =
            (bench_frame *)realloc(clip->frames, new_cap * sizeof(*grown));
        if (grown == NULL) {
            return -1;
        }
        clip->frames = grown;
        clip->frame_cap = new_cap;
    }
    clip->frames[clip->frame_count++] = *frame;
    return 0;
}

/* ============================================================================
 * Deterministic Synthetic 130-Clip Ablation Suite (80 Positives + 50 Negatives)
 * ============================================================================ */

static uint32_t xorshift32(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static float rand_signed(uint32_t *rng, float magnitude)
{
    const uint32_t r = xorshift32(rng) & 0xFFFFU;
    return (((float)r / 32767.5F) - 1.0F) * magnitude;
}

static bench_frame make_det_frame(uint64_t ts_ms,
                                  float cx,
                                  float cy,
                                  float half_w,
                                  float half_h,
                                  float conf)
{
    bench_frame f;
    (void)memset(&f, 0, sizeof(f));
    f.timestamp_ms = ts_ms;
    f.detection_count = 1U;
    f.detections[0].x0 = cx - half_w;
    f.detections[0].y0 = cy - half_h;
    f.detections[0].x1 = cx + half_w;
    f.detections[0].y1 = cy + half_h;
    f.detections[0].confidence = conf;
    f.detections[0].class_id = 0;
    return f;
}

static bench_frame make_empty_frame(uint64_t ts_ms)
{
    bench_frame f;
    (void)memset(&f, 0, sizeof(f));
    f.timestamp_ms = ts_ms;
    f.detection_count = 0U;
    return f;
}

static void dir_unit_vector(hand_direction_code dir, float *ux, float *uy)
{
    *ux = 0.0F;
    *uy = 0.0F;
    if (dir == HAND_DIRECTION_RIGHT) *ux = 1.0F;
    else if (dir == HAND_DIRECTION_LEFT) *ux = -1.0F;
    else if (dir == HAND_DIRECTION_DOWN) *uy = 1.0F;
    else if (dir == HAND_DIRECTION_UP) *uy = -1.0F;
}

static int add_synthetic_positive_clip(bench_suite *suite,
                                       hand_direction_code dir,
                                       unsigned int variant,
                                       uint32_t *rng)
{
    char name[96];
    bench_clip *clip;
    float ux = 0.0F;
    float uy = 0.0F;
    float px;
    float py;
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    size_t num_frames;
    size_t dwell_frames = 0U;
    uint64_t dt_ms = 38U;
    int drop_mid = 0;
    int expand_box = 0;
    size_t i;
    const float mid_x = 0.498F;
    const float mid_y = 0.500F;

    (void)snprintf(name, sizeof(name), "syn_%s_%02u",
                   hand_direction_name(dir), variant + 1U);
    clip = bench_suite_add_clip(suite, name, dir);
    if (clip == NULL) return -1;

    dir_unit_vector(dir, &ux, &uy);
    px = -uy;
    py = ux;

    if (variant < 6U) {
        /* Medium smooth swipe (~340-418 ms) */
        const float reach = (ux != 0.0F) ? 0.125F : 0.210F;
        start_x = mid_x - ux * reach;
        start_y = mid_y - uy * reach;
        end_x = mid_x + ux * reach;
        end_y = mid_y + uy * reach;
        num_frames = 10U + (variant % 2U);
    } else if (variant < 11U) {
        /* Brisk fast swipe (5 frames = 152 ms or 6 frames = 190 ms) */
        const float reach = (ux != 0.0F) ? 0.135F : 0.230F;
        start_x = mid_x - ux * reach;
        start_y = mid_y - uy * reach;
        end_x = mid_x + ux * reach;
        end_y = mid_y + uy * reach;
        num_frames = (variant % 2U == 0U) ? 5U : 6U;
    } else if (variant < 15U) {
        /* 2 dropped detection frames + slight ~20 deg off-axis slant */
        const float reach = (ux != 0.0F) ? 0.125F : 0.210F;
        const float slant = (ux != 0.0F) ? 0.095F : 0.055F;
        const float sgn = ((variant & 1U) == 0U) ? 1.0F : -1.0F;
        start_x = mid_x - ux * reach - px * (0.5F * slant * sgn);
        start_y = mid_y - uy * reach - py * (0.5F * slant * sgn);
        end_x = mid_x + ux * reach + px * (0.5F * slant * sgn);
        end_y = mid_y + uy * reach + py * (0.5F * slant * sgn);
        num_frames = 11U;
        drop_mid = 1;
    } else if (variant < 18U) {
        /* 230 ms leading dwell/hesitation inside ROI followed by swipe */
        const float reach = (ux != 0.0F) ? 0.125F : 0.200F;
        start_x = mid_x - ux * reach;
        start_y = mid_y - uy * reach;
        end_x = mid_x + ux * reach;
        end_y = mid_y + uy * reach;
        dwell_frames = 6U;
        num_frames = 10U;
    } else {
        /* Forearm sudden vertical/horizontal bbox growth mid-swipe */
        const float reach = (ux != 0.0F) ? 0.125F : 0.205F;
        start_x = mid_x - ux * reach;
        start_y = mid_y - uy * reach;
        end_x = mid_x + ux * reach;
        end_y = mid_y + uy * reach;
        num_frames = 11U;
        expand_box = 1;
    }

    clip->has_explicit_onset = 1;
    clip->explicit_onset_ms = 100U + (uint64_t)dwell_frames * dt_ms;

    for (i = 0U; i < dwell_frames; ++i) {
        const uint64_t ts = 100U + (uint64_t)i * dt_ms;
        const float jx = rand_signed(rng, 0.0025F);
        const float jy = rand_signed(rng, 0.0025F);
        const bench_frame f = make_det_frame(ts, start_x + jx, start_y + jy,
                                             0.050F, 0.070F, 0.88F);
        if (bench_clip_add_frame(clip, &f) != 0) return -1;
    }

    for (i = 0U; i < num_frames; ++i) {
        const uint64_t ts = 100U + (uint64_t)(dwell_frames + i) * dt_ms;
        const float t = num_frames > 1U ? (float)i / (float)(num_frames - 1U) : 1.0F;
        if (drop_mid && (i == 4U || i == 5U)) {
            const bench_frame empty = make_empty_frame(ts);
            if (bench_clip_add_frame(clip, &empty) != 0) return -1;
            continue;
        }
        {
            const float jx = rand_signed(rng, 0.0020F);
            const float jy = rand_signed(rng, 0.0025F);
            const float cx = start_x + (end_x - start_x) * t + jx;
            const float cy = start_y + (end_y - start_y) * t + jy;
            float hw = 0.050F + rand_signed(rng, 0.003F);
            float hh = 0.070F + rand_signed(rng, 0.004F);
            bench_frame f;

            if (expand_box && i >= 4U && i <= 6U) {
                /* Asymmetric expansion mimicking forearm entering detector box */
                if (ux != 0.0F) {
                    f = make_det_frame(ts, cx, cy + 0.085F, hw, hh + 0.085F, 0.87F);
                } else {
                    f = make_det_frame(ts, cx + 0.045F, cy, hw + 0.045F, hh, 0.87F);
                }
            } else {
                f = make_det_frame(ts, cx, cy, hw, hh, 0.90F);
            }
            if (bench_clip_add_frame(clip, &f) != 0) return -1;
        }
    }
    return 0;
}

static int add_synthetic_negative_clips(bench_suite *suite, uint32_t *rng)
{
    unsigned int k;
    size_t i;
    const uint64_t dt_ms = 38U;

    /* 1. 10 static / micro-jitter holds inside ROI */
    for (k = 0U; k < 10U; ++k) {
        char name[96];
        bench_clip *clip;
        const float bx = 0.43F + 0.015F * (float)(k % 5U);
        const float by = 0.35F + 0.070F * (float)(k / 5U);
        (void)snprintf(name, sizeof(name), "neg_static_%02u", k + 1U);
        clip = bench_suite_add_clip(suite, name, HAND_DIRECTION_NONE);
        if (clip == NULL) return -1;
        for (i = 0U; i < 16U; ++i) {
            const bench_frame f = make_det_frame(
                100U + (uint64_t)i * dt_ms,
                bx + rand_signed(rng, 0.004F),
                by + rand_signed(rng, 0.006F),
                0.050F, 0.070F, 0.89F);
            if (bench_clip_add_frame(clip, &f) != 0) return -1;
        }
    }

    /* 2. 12 edge swipes outside strict 80%x60% ROI */
    for (k = 0U; k < 12U; ++k) {
        char name[96];
        bench_clip *clip;
        const unsigned int band = k % 4U;
        (void)snprintf(name, sizeof(name), "neg_edge_%02u", k + 1U);
        clip = bench_suite_add_clip(suite, name, HAND_DIRECTION_NONE);
        if (clip == NULL) return -1;
        for (i = 0U; i < 10U; ++i) {
            const float t = (float)i / 9.0F;
            float cx = 0.50F;
            float cy = 0.50F;
            if (band == 0U) { cx = 0.295F; cy = 0.72F - 0.44F * t; }
            else if (band == 1U) { cx = 0.700F; cy = 0.28F + 0.44F * t; }
            else if (band == 2U) { cx = 0.35F + 0.28F * t; cy = 0.145F; }
            else { cx = 0.63F - 0.28F * t; cy = 0.855F; }
            {
                const bench_frame f = make_det_frame(
                    100U + (uint64_t)i * dt_ms, cx, cy, 0.050F, 0.070F, 0.88F);
                if (bench_clip_add_frame(clip, &f) != 0) return -1;
            }
        }
    }

    /* 3. 10 curved / L-hook trajectories (naive chord fools v1, dirv4 rejects) */
    for (k = 0U; k < 10U; ++k) {
        char name[96];
        bench_clip *clip;
        static const float pts_a[11][2] = {
            {0.400F, 0.600F}, {0.408F, 0.583F}, {0.416F, 0.566F},
            {0.425F, 0.549F}, {0.433F, 0.532F}, {0.442F, 0.515F},
            {0.450F, 0.500F}, {0.483F, 0.502F}, {0.516F, 0.505F},
            {0.550F, 0.508F}, {0.585F, 0.510F}
        };
        const float flip_x = (k % 2U == 0U) ? 1.0F : -1.0F;
        const float flip_y = ((k / 2U) % 2U == 0U) ? 1.0F : -1.0F;
        (void)snprintf(name, sizeof(name), "neg_hook_%02u", k + 1U);
        clip = bench_suite_add_clip(suite, name, HAND_DIRECTION_NONE);
        if (clip == NULL) return -1;
        for (i = 0U; i < 11U; ++i) {
            const float dx = (pts_a[i][0] - 0.498F) * flip_x;
            const float dy = (pts_a[i][1] - 0.500F) * flip_y;
            const bench_frame f = make_det_frame(
                100U + (uint64_t)i * dt_ms,
                0.498F + dx + rand_signed(rng, 0.0015F),
                0.500F + dy + rand_signed(rng, 0.0015F),
                0.050F, 0.070F, 0.90F);
            if (bench_clip_add_frame(clip, &f) != 0) return -1;
        }
    }

    /* 4. 10 back-and-forth / recoil reversal strokes inside ROI */
    for (k = 0U; k < 10U; ++k) {
        char name[96];
        bench_clip *clip;
        static const float wave[12] = {
            0.000F, 0.040F, 0.080F, 0.120F, 0.090F, 0.075F,
            0.110F, 0.145F, 0.180F, 0.210F, 0.235F, 0.255F
        };
        (void)snprintf(name, sizeof(name), "neg_recoil_%02u", k + 1U);
        clip = bench_suite_add_clip(suite, name, HAND_DIRECTION_NONE);
        if (clip == NULL) return -1;
        for (i = 0U; i < 12U; ++i) {
            const bench_frame f = make_det_frame(
                100U + (uint64_t)i * dt_ms,
                0.498F + rand_signed(rng, 0.002F),
                0.350F + wave[i],
                0.050F, 0.070F, 0.88F);
            if (bench_clip_add_frame(clip, &f) != 0) return -1;
        }
    }

    /* 5. 8 brief flashes & incomplete edge-exit strokes */
    for (k = 0U; k < 8U; ++k) {
        char name[96];
        bench_clip *clip;
        (void)snprintf(name, sizeof(name), "neg_flash_exit_%02u", k + 1U);
        clip = bench_suite_add_clip(suite, name, HAND_DIRECTION_NONE);
        if (clip == NULL) return -1;
        if (k < 4U) {
            /* 1-frame FP flash followed by empty frames */
            for (i = 0U; i < 8U; ++i) {
                const bench_frame f = (i == 2U)
                    ? make_det_frame(100U + (uint64_t)i * dt_ms, 0.50F, 0.50F, 0.05F, 0.07F, 0.68F)
                    : make_empty_frame(100U + (uint64_t)i * dt_ms);
                if (bench_clip_add_frame(clip, &f) != 0) return -1;
            }
        } else {
            /* Enters ROI briefly (3 frames) then exits strict ROI before finishing */
            static const float xs[7] = {0.31F, 0.35F, 0.39F, 0.43F, 0.36F, 0.30F, 0.25F};
            for (i = 0U; i < 7U; ++i) {
                const bench_frame f = make_det_frame(
                    100U + (uint64_t)i * dt_ms, xs[i], 0.50F, 0.05F, 0.07F, 0.87F);
                if (bench_clip_add_frame(clip, &f) != 0) return -1;
            }
        }
    }

    return 0;
}

static int build_synthetic_ablation_suite(bench_suite *suite)
{
    static const hand_direction_code dirs[4] = {
        HAND_DIRECTION_UP,
        HAND_DIRECTION_DOWN,
        HAND_DIRECTION_LEFT,
        HAND_DIRECTION_RIGHT
    };
    uint32_t rng = 0x20261124U;
    size_t d;
    unsigned int v;

    for (d = 0U; d < 4U; ++d) {
        for (v = 0U; v < 20U; ++v) {
            if (add_synthetic_positive_clip(suite, dirs[d], v, &rng) != 0) {
                return -1;
            }
        }
    }
    if (add_synthetic_negative_clips(suite, &rng) != 0) {
        return -1;
    }
    return 0;
}

/* ============================================================================
 * CSV Recording Loader / Session Splitter / Exporter
 * ============================================================================ */

static hand_direction_code parse_direction_token(const char *token)
{
    char buf[16];
    size_t i = 0U;

    if (token == NULL) return HAND_DIRECTION_NONE;
    while (token[i] != '\0' && !isspace((unsigned char)token[i]) &&
           token[i] != ',' && token[i] != ';' && i + 1U < sizeof(buf)) {
        buf[i] = (char)toupper((unsigned char)token[i]);
        ++i;
    }
    buf[i] = '\0';

    if (strcmp(buf, "UP") == 0) return HAND_DIRECTION_UP;
    if (strcmp(buf, "DOWN") == 0) return HAND_DIRECTION_DOWN;
    if (strcmp(buf, "LEFT") == 0) return HAND_DIRECTION_LEFT;
    if (strcmp(buf, "RIGHT") == 0) return HAND_DIRECTION_RIGHT;
    return HAND_DIRECTION_NONE;
}

static hand_direction_code infer_label_from_filename(const char *path)
{
    const char *base;
    char lower[128];
    size_t i = 0U;

    if (path == NULL) return HAND_DIRECTION_NONE;
    base = strrchr(path, '/');
    base = base != NULL ? base + 1 : path;
    while (base[i] != '\0' && i + 1U < sizeof(lower)) {
        lower[i] = (char)tolower((unsigned char)base[i]);
        ++i;
    }
    lower[i] = '\0';

    if (strncmp(lower, "up", 2U) == 0 || strstr(lower, "_up") != NULL) return HAND_DIRECTION_UP;
    if (strncmp(lower, "down", 4U) == 0 || strstr(lower, "_down") != NULL) return HAND_DIRECTION_DOWN;
    if (strncmp(lower, "left", 4U) == 0 || strstr(lower, "_left") != NULL) return HAND_DIRECTION_LEFT;
    if (strncmp(lower, "right", 5U) == 0 || strstr(lower, "_right") != NULL) return HAND_DIRECTION_RIGHT;
    return HAND_DIRECTION_NONE;
}

static int parse_roi_quad(const char *text,
                          float *x0,
                          float *y0,
                          float *x1,
                          float *y1)
{
    float a, b, c, d;
    if (text == NULL || sscanf(text, "%f,%f,%f,%f", &a, &b, &c, &d) != 4) {
        return -1;
    }
    if (a < 0.0F || b < 0.0F || c > 1.0F || d > 1.0F || a >= c || b >= d) {
        return -1;
    }
    *x0 = a;
    *y0 = b;
    *x1 = c;
    *y1 = d;
    return 0;
}

static int parse_csv_frame_line(const char *line, bench_frame *out)
{
    const char *p = line;
    char *endp = NULL;
    unsigned long long seq;
    unsigned long long ts_ms;
    unsigned long det_count;
    double dummy;
    size_t i;

    (void)memset(out, 0, sizeof(*out));
    seq = strtoull(p, &endp, 10);
    (void)seq;
    if (endp == p || *endp != ',') return -1;
    p = endp + 1;

    ts_ms = strtoull(p, &endp, 10);
    if (endp == p || *endp != ',') return -1;
    p = endp + 1;
    out->timestamp_ms = (uint64_t)ts_ms;

    for (i = 0U; i < 3U; ++i) {
        dummy = strtod(p, &endp);
        (void)dummy;
        if (endp == p || *endp != ',') return -1;
        p = endp + 1;
    }

    det_count = strtoul(p, &endp, 10);
    if (endp == p) return -1;
    p = endp;

    if (det_count > HAND_DETECTION_MAX_COUNT) {
        det_count = HAND_DETECTION_MAX_COUNT;
    }
    for (i = 0U; i < det_count; ++i) {
        hand_detection det;
        long cls_id;

        if (*p != ',') break;
        ++p; det.x0 = (float)strtod(p, &endp); if (endp == p || *endp != ',') break;
        ++p; det.y0 = (float)strtod(endp + 1, &endp); if (endp == p || *endp != ',') break;
        ++p; det.x1 = (float)strtod(endp + 1, &endp); if (endp == p || *endp != ',') break;
        ++p; det.y1 = (float)strtod(endp + 1, &endp); if (endp == p || *endp != ',') break;
        ++p; det.confidence = (float)strtod(endp + 1, &endp); if (endp == p || *endp != ',') break;
        cls_id = strtol(endp + 1, &endp, 10);
        det.class_id = (int)cls_id;
        p = endp;

        out->detections[out->detection_count++] = det;
    }
    return 0;
}

static int frame_has_active_hand_in_roi(const bench_frame *frame,
                                        float rx0,
                                        float ry0,
                                        float rx1,
                                        float ry1)
{
    size_t i;
    for (i = 0U; i < frame->detection_count; ++i) {
        const hand_detection *d = &frame->detections[i];
        const float cx = 0.5F * (d->x0 + d->x1);
        const float cy = 0.5F * (d->y0 + d->y1);
        if (d->class_id == 0 && d->confidence >= 0.55F &&
            cx >= rx0 && cx <= rx1 && cy >= ry0 && cy <= ry1) {
            return 1;
        }
    }
    return 0;
}

static int split_clip_sessions(bench_suite *suite,
                               const bench_clip *raw,
                               const char *base_name)
{
    size_t n = raw->frame_count;
    size_t i = 0U;
    unsigned int seg_idx = 0U;

    while (i < n) {
        size_t start;
        size_t end;
        size_t last_active;
        char seg_name[96];
        bench_clip *dst;
        size_t k;

        while (i < n && !frame_has_active_hand_in_roi(&raw->frames[i],
                                                      suite->roi_x_min,
                                                      suite->roi_y_min,
                                                      suite->roi_x_max,
                                                      suite->roi_y_max)) {
            ++i;
        }
        if (i >= n) break;

        start = i > 2U ? i - 2U : i;
        last_active = i;
        ++i;
        while (i < n) {
            if (frame_has_active_hand_in_roi(&raw->frames[i],
                                             suite->roi_x_min,
                                             suite->roi_y_min,
                                             suite->roi_x_max,
                                             suite->roi_y_max)) {
                last_active = i;
            } else if (raw->frames[i].timestamp_ms > raw->frames[last_active].timestamp_ms &&
                       raw->frames[i].timestamp_ms - raw->frames[last_active].timestamp_ms >=
                           BENCH_SESSION_GAP_MS) {
                break;
            }
            ++i;
        }
        end = (last_active + 3U < n) ? last_active + 3U : n;

        ++seg_idx;
        (void)snprintf(seg_name, sizeof(seg_name), "%s#%u", base_name, seg_idx);
        dst = bench_suite_add_clip(suite, seg_name, raw->expected);
        if (dst == NULL) return -1;
        dst->has_explicit_onset = 1;
        dst->explicit_onset_ms = raw->frames[start].timestamp_ms;
        for (k = start; k < end; ++k) {
            if (bench_clip_add_frame(dst, &raw->frames[k]) != 0) return -1;
        }
    }
    return 0;
}

static int load_csv_file(bench_suite *suite, const char *path, int split_sessions)
{
    FILE *file = fopen(path, "r");
    char line[4096];
    const char *base = strrchr(path, '/');
    const hand_direction_code default_dir = infer_label_from_filename(path);
    bench_clip *current_clip = NULL;
    int saw_explicit_clip = 0;

    base = base != NULL ? base + 1 : path;
    if (file == NULL) {
        (void)fprintf(stderr, "cannot open CSV '%s': %s\n", path, strerror(errno));
        return -1;
    }

    while (fgets(line, (int)sizeof(line), file) != NULL) {
        if (line[0] == '\n' || line[0] == '\r') continue;
        if (line[0] == '#') {
            const char *roi_pos = strstr(line, "roi=");
            if (!suite->has_roi_override && roi_pos != NULL) {
                float x0, y0, x1, y1;
                if (parse_roi_quad(roi_pos + 4, &x0, &y0, &x1, &y1) == 0) {
                    suite->roi_x_min = x0;
                    suite->roi_y_min = y0;
                    suite->roi_x_max = x1;
                    suite->roi_y_max = y1;
                }
            }
            if (strncmp(line, "# clip", 6U) == 0) {
                const char *lbl = strstr(line, "label=");
                const char *nm = strstr(line, "name=");
                const char *onset = strstr(line, "onset_ms=");
                const char *tr = strstr(line, "trials=");
                hand_direction_code dir = lbl != NULL
                    ? parse_direction_token(lbl + 6)
                    : default_dir;
                char clip_name[96];

                if (nm != NULL) {
                    size_t n = 0U;
                    const char *s = nm + 5;
                    while (s[n] != '\0' && !isspace((unsigned char)s[n]) &&
                           n + 1U < sizeof(clip_name)) {
                        clip_name[n] = s[n];
                        ++n;
                    }
                    clip_name[n] = '\0';
                } else {
                    (void)snprintf(clip_name, sizeof(clip_name), "%s#%zu",
                                   base, suite->clip_count + 1U);
                }
                current_clip = bench_suite_add_clip(suite, clip_name, dir);
                if (current_clip == NULL) {
                    (void)fclose(file);
                    return -1;
                }
                if (tr != NULL) {
                    current_clip->expected_trials =
                        (size_t)strtoull(tr + 7, NULL, 10);
                }
                if (onset != NULL) {
                    current_clip->has_explicit_onset = 1;
                    current_clip->explicit_onset_ms =
                        (uint64_t)strtoull(onset + 9, NULL, 10);
                }
                saw_explicit_clip = 1;
            }
            continue;
        }

        {
            bench_frame frame;
            if (parse_csv_frame_line(line, &frame) != 0) {
                continue;
            }
            if (current_clip == NULL) {
                current_clip = bench_suite_add_clip(suite, base, default_dir);
                if (current_clip == NULL) {
                    (void)fclose(file);
                    return -1;
                }
            }
            if (bench_clip_add_frame(current_clip, &frame) != 0) {
                (void)fclose(file);
                return -1;
            }
        }
    }
    (void)fclose(file);

    if (split_sessions && !saw_explicit_clip && current_clip != NULL &&
        current_clip->expected != HAND_DIRECTION_NONE &&
        suite->clip_count > 0U) {
        bench_clip raw_copy = *current_clip;
        suite->clip_count--; /* pop single raw clip; frames now owned by raw_copy */
        if (split_clip_sessions(suite, &raw_copy, base) != 0) {
            free(raw_copy.frames);
            return -1;
        }
        free(raw_copy.frames);
    }
    return 0;
}

static int cmp_str_ptr(const void *a, const void *b)
{
    const char *const *sa = (const char *const *)a;
    const char *const *sb = (const char *const *)b;
    return strcmp(*sa, *sb);
}

static int load_path_into_suite(bench_suite *suite, const char *path, int split_sessions)
{
    struct stat st;
    if (stat(path, &st) != 0) {
        (void)fprintf(stderr, "cannot stat '%s': %s\n", path, strerror(errno));
        return -1;
    }
    if (!S_ISDIR(st.st_mode)) {
        return load_csv_file(suite, path, split_sessions);
    }

    {
        DIR *dir = opendir(path);
        struct dirent *ent;
        char **names = NULL;
        size_t count = 0U;
        size_t cap = 0U;
        size_t i;

        if (dir == NULL) {
            (void)fprintf(stderr, "cannot open dir '%s': %s\n", path, strerror(errno));
            return -1;
        }
        while ((ent = readdir(dir)) != NULL) {
            const size_t len = strlen(ent->d_name);
            if (len > 4U && strcmp(ent->d_name + len - 4U, ".csv") == 0) {
                char *full;
                const size_t plen = strlen(path) + len + 2U;
                if (count == cap) {
                    const size_t nc = cap == 0U ? 16U : cap * 2U;
                    char **grown = (char **)realloc(names, nc * sizeof(*grown));
                    if (grown == NULL) {
                        (void)closedir(dir);
                        for (i = 0U; i < count; ++i) free(names[i]);
                        free(names);
                        return -1;
                    }
                    names = grown;
                    cap = nc;
                }
                full = (char *)malloc(plen);
                if (full == NULL) {
                    (void)closedir(dir);
                    for (i = 0U; i < count; ++i) free(names[i]);
                    free(names);
                    return -1;
                }
                (void)snprintf(full, plen, "%s/%s", path, ent->d_name);
                names[count++] = full;
            }
        }
        (void)closedir(dir);
        if (count > 1U) {
            qsort(names, count, sizeof(*names), cmp_str_ptr);
        }
        for (i = 0U; i < count; ++i) {
            const int rc = load_csv_file(suite, names[i], split_sessions);
            free(names[i]);
            if (rc != 0) {
                size_t j;
                for (j = i + 1U; j < count; ++j) free(names[j]);
                free(names);
                return -1;
            }
        }
        free(names);
    }
    return 0;
}

static int export_suite_to_csv(const bench_suite *suite, const char *path)
{
    FILE *file = fopen(path, "w");
    size_t c;
    uint64_t seq = 0U;

    if (file == NULL) {
        (void)fprintf(stderr, "cannot write '%s': %s\n", path, strerror(errno));
        return -1;
    }
    (void)fprintf(file,
                  "# hand_direction_record_v1 width=640 height=360 "
                  "roi=%.6f,%.6f,%.6f,%.6f\n",
                  (double)suite->roi_x_min,
                  (double)suite->roi_y_min,
                  (double)suite->roi_x_max,
                  (double)suite->roi_y_max);
    (void)fprintf(file,
                  "# columns: seq,capture_ms,infer_ms,prep_ms,npu_ms,det_count"
                  "[,x0,y0,x1,y1,conf,class_id]...\n");

    for (c = 0U; c < suite->clip_count; ++c) {
        const bench_clip *clip = &suite->clips[c];
        size_t f;

        (void)fprintf(file, "# clip label=%s name=%s",
                      hand_direction_name(clip->expected),
                      clip->name);
        if (clip->expected_trials > 0U) {
            (void)fprintf(file, " trials=%zu", clip->expected_trials);
        }
        if (clip->has_explicit_onset) {
            (void)fprintf(file, " onset_ms=%llu",
                          (unsigned long long)clip->explicit_onset_ms);
        }
        (void)fputc('\n', file);

        for (f = 0U; f < clip->frame_count; ++f) {
            const bench_frame *fr = &clip->frames[f];
            size_t d;

            ++seq;
            (void)fprintf(file, "%llu,%llu,37.000,15.500,21.000,%zu",
                          (unsigned long long)seq,
                          (unsigned long long)fr->timestamp_ms,
                          fr->detection_count);
            for (d = 0U; d < fr->detection_count; ++d) {
                (void)fprintf(file, ",%.5f,%.5f,%.5f,%.5f,%.4f,%d",
                              (double)fr->detections[d].x0,
                              (double)fr->detections[d].y0,
                              (double)fr->detections[d].x1,
                              (double)fr->detections[d].y1,
                              (double)fr->detections[d].confidence,
                              fr->detections[d].class_id);
            }
            (void)fputc('\n', file);
        }
    }
    (void)fclose(file);
    return 0;
}

/* ============================================================================
 * Evaluation & Markdown Report Formatting
 * ============================================================================ */

static size_t dir_to_class_index(hand_direction_code dir)
{
    switch (dir) {
    case HAND_DIRECTION_UP: return 0U;
    case HAND_DIRECTION_DOWN: return 1U;
    case HAND_DIRECTION_LEFT: return 2U;
    case HAND_DIRECTION_RIGHT: return 3U;
    case HAND_DIRECTION_NONE:
    default: return 4U;
    }
}

static uint64_t clip_onset_ms(const bench_clip *clip,
                              float rx0,
                              float ry0,
                              float rx1,
                              float ry1)
{
    size_t i;
    if (clip->has_explicit_onset) {
        return clip->explicit_onset_ms;
    }
    for (i = 0U; i < clip->frame_count; ++i) {
        if (frame_has_active_hand_in_roi(&clip->frames[i], rx0, ry0, rx1, ry1)) {
            return clip->frames[i].timestamp_ms;
        }
    }
    return clip->frame_count > 0U ? clip->frames[0].timestamp_ms : 0U;
}

static void bench_metrics_free(bench_metrics *m)
{
    if (m == NULL) return;
    free(m->latencies_ms);
    free(m->stream_stats);
    m->latencies_ms = NULL;
    m->stream_stats = NULL;
    m->latency_count = 0U;
    m->latency_cap = 0U;
    m->stream_stat_count = 0U;
}

static hand_direction_code opposite_direction(hand_direction_code dir)
{
    switch (dir) {
    case HAND_DIRECTION_UP: return HAND_DIRECTION_DOWN;
    case HAND_DIRECTION_DOWN: return HAND_DIRECTION_UP;
    case HAND_DIRECTION_LEFT: return HAND_DIRECTION_RIGHT;
    case HAND_DIRECTION_RIGHT: return HAND_DIRECTION_LEFT;
    case HAND_DIRECTION_NONE:
    default: return HAND_DIRECTION_NONE;
    }
}

static void init_bench_dirv4_engine(const bench_suite *suite,
                                    hand_gesture_engine *eng)
{
    hand_gesture_engine_config cfg;

    hand_gesture_engine_config_init_defaults(&cfg);
    cfg.roi_x_min = suite->roi_x_min;
    cfg.roi_y_min = suite->roi_y_min;
    cfg.roi_x_max = suite->roi_x_max;
    cfg.roi_y_max = suite->roi_y_max;
    cfg.direction.window_ms = BENCH_DEFAULT_WINDOW_MS;
    cfg.direction.x_scale = BENCH_DEFAULT_X_SCALE;
    cfg.cooldown_ms = BENCH_COOLDOWN_MS;
    hand_gesture_engine_init(eng, &cfg);
}

static int record_latency(bench_metrics *m, uint64_t lat_ms)
{
    if (m->latency_count == m->latency_cap) {
        const size_t nc = m->latency_cap == 0U ? 64U : m->latency_cap * 2U;
        uint64_t *grown = (uint64_t *)realloc(m->latencies_ms, nc * sizeof(*grown));
        if (grown == NULL) return -1;
        m->latencies_ms = grown;
        m->latency_cap = nc;
    }
    m->latencies_ms[m->latency_count++] = lat_ms;
    return 0;
}

static int evaluate_suite(const bench_suite *suite, int use_dirv4, bench_metrics *out)
{
    size_t c;

    (void)memset(out, 0, sizeof(*out));
    out->total_clips = suite->clip_count;

    for (c = 0U; c < suite->clip_count; ++c) {
        const bench_clip *clip = &suite->clips[c];
        const uint64_t onset = clip_onset_ms(clip,
                                             suite->roi_x_min,
                                             suite->roi_y_min,
                                             suite->roi_x_max,
                                             suite->roi_y_max);
        hand_direction_code predicted = HAND_DIRECTION_NONE;
        uint64_t emit_ms = 0U;
        size_t event_count = 0U;
        size_t f;

        hand_gesture_engine dirv4_eng;
        legacy_v1_engine v1_eng;

        if (use_dirv4) {
            init_bench_dirv4_engine(suite, &dirv4_eng);
        } else {
            legacy_v1_init(&v1_eng,
                           suite->roi_x_min,
                           suite->roi_y_min,
                           suite->roi_x_max,
                           suite->roi_y_max);
        }

        for (f = 0U; f < clip->frame_count; ++f) {
            const bench_frame *fr = &clip->frames[f];
            hand_gesture_event ev;

            if (use_dirv4) {
                if (hand_gesture_engine_update(&dirv4_eng,
                                               fr->detection_count > 0U ? fr->detections : NULL,
                                               fr->detection_count,
                                               fr->timestamp_ms,
                                               &ev) != 0) {
                    return -1;
                }
            } else {
                legacy_v1_update(&v1_eng,
                                 fr->detection_count > 0U ? fr->detections : NULL,
                                 fr->detection_count,
                                 fr->timestamp_ms,
                                 &ev);
            }

            if (ev.valid) {
                ++event_count;
                if (predicted == HAND_DIRECTION_NONE) {
                    predicted = ev.direction;
                    emit_ms = ev.timestamp_ms;
                }
            }
        }

        if (event_count > 1U) {
            out->duplicate_events += event_count - 1U;
        }
        ++out->confusion[dir_to_class_index(clip->expected)]
                        [dir_to_class_index(predicted)];

        if (clip->expected != HAND_DIRECTION_NONE) {
            ++out->pos_clips;
            if (predicted == clip->expected) {
                ++out->pos_correct;
                if (record_latency(out, emit_ms >= onset ? (emit_ms - onset) : 0U) != 0) {
                    return -1;
                }
            } else if (predicted == HAND_DIRECTION_NONE) {
                ++out->pos_missed;
            } else {
                ++out->pos_wrong_dir;
            }
        } else {
            ++out->neg_clips;
            if (predicted != HAND_DIRECTION_NONE) {
                ++out->neg_false_triggers;
            }
        }
    }
    return 0;
}

static int evaluate_suite_stream(const bench_suite *suite,
                                 int use_dirv4,
                                 size_t default_trials,
                                 bench_metrics *out)
{
    size_t c;

    (void)memset(out, 0, sizeof(*out));
    out->stream_mode = 1;
    out->total_clips = suite->clip_count;
    if (suite->clip_count > 0U) {
        out->stream_stats =
            (bench_stream_stat *)calloc(suite->clip_count, sizeof(*out->stream_stats));
        if (out->stream_stats == NULL) {
            return -1;
        }
        out->stream_stat_count = suite->clip_count;
    }

    for (c = 0U; c < suite->clip_count; ++c) {
        const bench_clip *clip = &suite->clips[c];
        const size_t trials =
            clip->expected_trials > 0U ? clip->expected_trials : default_trials;
        const hand_direction_code opp_dir = opposite_direction(clip->expected);
        const size_t exp_idx = dir_to_class_index(clip->expected);
        double dur_sec = 0.0;
        size_t matched = 0U;
        size_t opp_return = 0U;
        size_t other_wrong = 0U;
        size_t f;

        hand_gesture_engine dirv4_eng;
        legacy_v1_engine v1_eng;

        if (clip->frame_count >= 2U &&
            clip->frames[clip->frame_count - 1U].timestamp_ms > clip->frames[0].timestamp_ms) {
            dur_sec = (double)(clip->frames[clip->frame_count - 1U].timestamp_ms -
                               clip->frames[0].timestamp_ms) / 1000.0;
        }
        if (clip->expected != HAND_DIRECTION_NONE) {
            out->pos_duration_sec += dur_sec;
        } else {
            out->neg_duration_sec += dur_sec;
        }

        if (use_dirv4) {
            init_bench_dirv4_engine(suite, &dirv4_eng);
        } else {
            legacy_v1_init(&v1_eng,
                           suite->roi_x_min,
                           suite->roi_y_min,
                           suite->roi_x_max,
                           suite->roi_y_max);
        }

        for (f = 0U; f < clip->frame_count; ++f) {
            const bench_frame *fr = &clip->frames[f];
            hand_gesture_event ev;
            uint64_t stroke_span_ms = 0U;

            if (use_dirv4) {
                if (hand_gesture_engine_update(&dirv4_eng,
                                               fr->detection_count > 0U ? fr->detections : NULL,
                                               fr->detection_count,
                                               fr->timestamp_ms,
                                               &ev) != 0) {
                    bench_metrics_free(out);
                    return -1;
                }
                if (ev.valid) {
                    hand_gesture_diagnostics diag;
                    hand_gesture_engine_get_diagnostics(&dirv4_eng, &diag);
                    stroke_span_ms = diag.direction.span_ms;
                }
            } else {
                legacy_v1_update(&v1_eng,
                                 fr->detection_count > 0U ? fr->detections : NULL,
                                 fr->detection_count,
                                 fr->timestamp_ms,
                                 &ev);
                if (ev.valid) {
                    stroke_span_ms = v1_eng.last_event_span_ms;
                }
            }

            if (ev.valid) {
                const size_t pred_idx = dir_to_class_index(ev.direction);

                if (clip->expected != HAND_DIRECTION_NONE) {
                    if (ev.direction == clip->expected) {
                        ++matched;
                        if (record_latency(out, stroke_span_ms) != 0) {
                            bench_metrics_free(out);
                            return -1;
                        }
                    } else if (ev.direction == opp_dir) {
                        ++opp_return;
                        ++out->opposite_return_fp;
                        ++out->pos_wrong_dir;
                        ++out->confusion[exp_idx][pred_idx];
                    } else {
                        ++other_wrong;
                        ++out->other_wrong_fp;
                        ++out->pos_wrong_dir;
                        ++out->confusion[exp_idx][pred_idx];
                    }
                } else {
                    ++other_wrong;
                    ++out->neg_false_triggers;
                    ++out->confusion[4U][pred_idx];
                }
            }
        }

        {
            bench_stream_stat *st = &out->stream_stats[c];
            (void)snprintf(st->name, sizeof(st->name), "%s", clip->name);
            st->expected = clip->expected;
            st->expected_trials = trials;
            st->duration_sec = dur_sec;
            st->matched = matched;
            st->opposite_return = opp_return;
            st->other_wrong = other_wrong;
        }

        if (clip->expected != HAND_DIRECTION_NONE) {
            if (trials > 0U) {
                const size_t eff_hits = matched > trials ? trials : matched;
                const size_t missed = trials > matched ? trials - matched : 0U;

                out->pos_clips += trials;
                out->pos_correct += eff_hits;
                out->pos_missed += missed;
                out->confusion[exp_idx][exp_idx] += (uint32_t)eff_hits;
                out->confusion[exp_idx][4U] += (uint32_t)missed;
                if (matched > trials) {
                    out->duplicate_events += matched - trials;
                }
            } else {
                out->pos_clips += matched + opp_return + other_wrong;
                out->pos_correct += matched;
                out->confusion[exp_idx][exp_idx] += (uint32_t)matched;
            }
        } else {
            ++out->neg_clips;
            if (other_wrong == 0U) {
                ++out->confusion[4U][4U];
            }
        }
    }
    return 0;
}

static int cmp_u64(const void *a, const void *b)
{
    const uint64_t ua = *(const uint64_t *)a;
    const uint64_t ub = *(const uint64_t *)b;
    if (ua < ub) return -1;
    if (ua > ub) return 1;
    return 0;
}

static double percentile_ms(uint64_t *vals, size_t count, double p)
{
    size_t idx;
    if (count == 0U) return 0.0;
    idx = (size_t)floor(p * (double)(count - 1U) + 0.5);
    if (idx >= count) idx = count - 1U;
    return (double)vals[idx];
}

static double pct(size_t num, size_t den)
{
    return den > 0U ? (double)num * 100.0 / (double)den : 0.0;
}

static void dir_recall(const bench_metrics *m,
                       size_t cls_idx,
                       size_t *hits,
                       size_t *total)
{
    size_t col;
    size_t sum = 0U;
    for (col = 0U; col < BENCH_CLASSES; ++col) {
        sum += m->confusion[cls_idx][col];
    }
    *hits = m->confusion[cls_idx][cls_idx];
    *total = sum;
}

static void print_confusion_matrix(FILE *out,
                                   const char *title,
                                   const bench_metrics *m)
{
    static const char *const labels[BENCH_CLASSES] = {
        "UP", "DOWN", "LEFT", "RIGHT", "NONE"
    };
    size_t r;

    (void)fprintf(out, "### %s 5x5 混淆矩阵\n\n", title);
    (void)fprintf(out, "| 真实 \\ 预测 | UP | DOWN | LEFT | RIGHT | NONE |\n");
    (void)fprintf(out, "|---|---:|---:|---:|---:|---:|\n");
    for (r = 0U; r < BENCH_CLASSES; ++r) {
        (void)fprintf(out, "| **%s** | %u | %u | %u | %u | %u |\n",
                      labels[r],
                      m->confusion[r][0],
                      m->confusion[r][1],
                      m->confusion[r][2],
                      m->confusion[r][3],
                      m->confusion[r][4]);
    }
    (void)fputc('\n', out);
}

static void print_markdown_report(FILE *out,
                                  const bench_suite *suite,
                                  bench_metrics *v1,
                                  bench_metrics *v4)
{
    double v1_p50 = 0.0, v1_p95 = 0.0, v1_max = 0.0;
    double v4_p50 = 0.0, v4_p95 = 0.0, v4_max = 0.0;
    size_t i;

    if (v1->latency_count > 0U) {
        qsort(v1->latencies_ms, v1->latency_count, sizeof(uint64_t), cmp_u64);
        v1_p50 = percentile_ms(v1->latencies_ms, v1->latency_count, 0.50);
        v1_p95 = percentile_ms(v1->latencies_ms, v1->latency_count, 0.95);
        v1_max = (double)v1->latencies_ms[v1->latency_count - 1U];
    }
    if (v4->latency_count > 0U) {
        qsort(v4->latencies_ms, v4->latency_count, sizeof(uint64_t), cmp_u64);
        v4_p50 = percentile_ms(v4->latencies_ms, v4->latency_count, 0.50);
        v4_p95 = percentile_ms(v4->latencies_ms, v4->latency_count, 0.95);
        v4_max = (double)v4->latencies_ms[v4->latency_count - 1U];
    }

    (void)fprintf(out, "# hand_direction_bench 消融回放报告\n\n");
    if (v4->stream_mode) {
        (void)fprintf(out,
                      "- **评测模式**：连续流逐事件统计（支持框内立即回中）\n"
                      "- **流文件总数**：%zu 个（正样本预期总次数 %zu，负样本流 %zu 个 / %.1f 秒）\n"
                      "- **评测 ROI**：`(%.3f, %.3f) - (%.3f, %.3f)`，横纵比校正 `x_scale=%.3f`\n\n",
                      suite->clip_count,
                      v4->pos_clips,
                      v4->neg_clips,
                      v4->neg_duration_sec,
                      (double)suite->roi_x_min,
                      (double)suite->roi_y_min,
                      (double)suite->roi_x_max,
                      (double)suite->roi_y_max,
                      (double)BENCH_DEFAULT_X_SCALE);
    } else {
        (void)fprintf(out,
                      "- **评测样本总数**：%zu 条（正样本 %zu，负样本 %zu）\n"
                      "- **评测 ROI**：`(%.3f, %.3f) - (%.3f, %.3f)`，横纵比校正 `x_scale=%.3f`\n\n",
                      suite->clip_count,
                      v4->pos_clips,
                      v4->neg_clips,
                      (double)suite->roi_x_min,
                      (double)suite->roi_y_min,
                      (double)suite->roi_x_max,
                      (double)suite->roi_y_max,
                      (double)BENCH_DEFAULT_X_SCALE);
    }

    (void)fprintf(out, "## 总指标对比（`legacy_v1` vs `current_dirv4`）\n\n");
    (void)fprintf(out, "| 指标 | `legacy_v1`（旧基线） | `current_dirv4`（当前） |\n");
    (void)fprintf(out, "|---|---:|---:|\n");
    (void)fprintf(out, "| 正样本总准确率 | %.1f%% (%zu/%zu) | %.1f%% (%zu/%zu) |\n",
                  pct(v1->pos_correct, v1->pos_clips), v1->pos_correct, v1->pos_clips,
                  pct(v4->pos_correct, v4->pos_clips), v4->pos_correct, v4->pos_clips);

    {
        static const char *const dir_names[4] = {"UP", "DOWN", "LEFT", "RIGHT"};
        for (i = 0U; i < 4U; ++i) {
            size_t h1 = 0U, t1 = 0U, h4 = 0U, t4 = 0U;
            if (v4->stream_mode && v4->stream_stat_count > 0U &&
                v1->stream_stat_count == v4->stream_stat_count) {
                size_t k;
                for (k = 0U; k < v4->stream_stat_count; ++k) {
                    const bench_stream_stat *s1 = &v1->stream_stats[k];
                    const bench_stream_stat *s4 = &v4->stream_stats[k];
                    if (dir_to_class_index(s4->expected) != i) {
                        continue;
                    }
                    if (s4->expected_trials > 0U) {
                        h1 += s1->matched > s1->expected_trials ? s1->expected_trials : s1->matched;
                        t1 += s1->expected_trials;
                        h4 += s4->matched > s4->expected_trials ? s4->expected_trials : s4->matched;
                        t4 += s4->expected_trials;
                    } else {
                        h1 += s1->matched;
                        t1 += s1->matched + s1->opposite_return + s1->other_wrong;
                        h4 += s4->matched;
                        t4 += s4->matched + s4->opposite_return + s4->other_wrong;
                    }
                }
            } else {
                dir_recall(v1, i, &h1, &t1);
                dir_recall(v4, i, &h4, &t4);
            }
            (void)fprintf(out, "| %s 召回率 | %.1f%% (%zu/%zu) | %.1f%% (%zu/%zu) |\n",
                          dir_names[i],
                          pct(h1, t1), h1, t1,
                          pct(h4, t4), h4, t4);
        }
    }

    (void)fprintf(out, "| 正样本漏触发率 | %.1f%% (%zu/%zu) | %.1f%% (%zu/%zu) |\n",
                  pct(v1->pos_missed, v1->pos_clips), v1->pos_missed, v1->pos_clips,
                  pct(v4->pos_missed, v4->pos_clips), v4->pos_missed, v4->pos_clips);

    if (v4->stream_mode) {
        const double v1_neg_rate =
            v1->neg_duration_sec > 0.0 ? (double)v1->neg_false_triggers * 60.0 / v1->neg_duration_sec : 0.0;
        const double v4_neg_rate =
            v4->neg_duration_sec > 0.0 ? (double)v4->neg_false_triggers * 60.0 / v4->neg_duration_sec : 0.0;

        (void)fprintf(out, "| 回中反向误触发 (Return FP) | %zu | %zu |\n",
                      v1->opposite_return_fp, v4->opposite_return_fp);
        (void)fprintf(out, "| 其他误向触发 | %zu | %zu |\n",
                      v1->other_wrong_fp, v4->other_wrong_fp);
        (void)fprintf(out, "| 负样本误触发 (次 / 次·分) | %zu (%.2f/min) | %zu (%.2f/min) |\n",
                      v1->neg_false_triggers, v1_neg_rate,
                      v4->neg_false_triggers, v4_neg_rate);
        (void)fprintf(out, "| 超额重复触发数 | %zu | %zu |\n",
                      v1->duplicate_events, v4->duplicate_events);
        (void)fprintf(out, "| 有效划动时长 P50 / P95 / Max (ms) | %.0f / %.0f / %.0f | %.0f / %.0f / %.0f |\n\n",
                      v1_p50, v1_p95, v1_max,
                      v4_p50, v4_p95, v4_max);

        if (v4->stream_stat_count > 0U && v1->stream_stat_count == v4->stream_stat_count) {
            (void)fprintf(out, "## 分文件明细\n\n");
            (void)fprintf(out, "| Stream | 预期 | 试验次数 | 时长 (s) | `legacy_v1` (命中/回中/他向) | `current_dirv4` (命中/回中/他向) |\n");
            (void)fprintf(out, "|---|---|---:|---:|---:|---:|\n");
            for (i = 0U; i < v4->stream_stat_count; ++i) {
                const bench_stream_stat *s1 = &v1->stream_stats[i];
                const bench_stream_stat *s4 = &v4->stream_stats[i];
                if (s4->expected == HAND_DIRECTION_NONE) {
                    (void)fprintf(out,
                                  "| %s | NONE | - | %.1f | 误触=%zu | 误触=%zu |\n",
                                  s4->name,
                                  s4->duration_sec,
                                  s1->other_wrong,
                                  s4->other_wrong);
                } else {
                    (void)fprintf(out,
                                  "| %s | %s | %zu | %.1f | %zu / %zu / %zu | %zu / %zu / %zu |\n",
                                  s4->name,
                                  hand_direction_name(s4->expected),
                                  s4->expected_trials,
                                  s4->duration_sec,
                                  s1->matched, s1->opposite_return, s1->other_wrong,
                                  s4->matched, s4->opposite_return, s4->other_wrong);
                }
            }
            (void)fputc('\n', out);
        }
    } else {
        (void)fprintf(out, "| 正样本误判他向 | %.1f%% (%zu/%zu) | %.1f%% (%zu/%zu) |\n",
                      pct(v1->pos_wrong_dir, v1->pos_clips), v1->pos_wrong_dir, v1->pos_clips,
                      pct(v4->pos_wrong_dir, v4->pos_clips), v4->pos_wrong_dir, v4->pos_clips);
        (void)fprintf(out, "| 负样本误触发率 | %.1f%% (%zu/%zu) | %.1f%% (%zu/%zu) |\n",
                      pct(v1->neg_false_triggers, v1->neg_clips),
                      v1->neg_false_triggers, v1->neg_clips,
                      pct(v4->neg_false_triggers, v4->neg_clips),
                      v4->neg_false_triggers, v4->neg_clips);
        (void)fprintf(out, "| 同片段重复触发数 | %zu | %zu |\n",
                      v1->duplicate_events, v4->duplicate_events);
        (void)fprintf(out, "| 触发延迟 P50 / P95 / Max (ms) | %.0f / %.0f / %.0f | %.0f / %.0f / %.0f |\n\n",
                      v1_p50, v1_p95, v1_max,
                      v4_p50, v4_p95, v4_max);
    }

    print_confusion_matrix(out, "`legacy_v1`", v1);
    print_confusion_matrix(out, "`current_dirv4`", v4);
}

static int run_stream_self_test(void)
{
    bench_suite suite_a;
    bench_suite suite_b;
    bench_metrics v1_a, v4_a, v1_b, v4_b;
    bench_clip *up_stream;
    bench_clip *neg_stream;
    const char *tmp_dir = getenv("TMPDIR");
    char tmp_path[512];
    const uint64_t dt_ms = 38U;
    uint32_t rng = 0x20261124U;
    uint64_t t = 1000U;
    unsigned int s;
    size_t i;
    int status = 1;

    if (tmp_dir == NULL || tmp_dir[0] == '\0') {
        tmp_dir = "/tmp";
    }
    (void)snprintf(tmp_path, sizeof(tmp_path),
                   "%s/hand_direction_bench_stream_selftest.csv", tmp_dir);

    bench_suite_init(&suite_a);
    bench_suite_init(&suite_b);
    (void)memset(&v1_a, 0, sizeof(v1_a));
    (void)memset(&v4_a, 0, sizeof(v4_a));
    (void)memset(&v1_b, 0, sizeof(v1_b));
    (void)memset(&v4_b, 0, sizeof(v4_b));

    up_stream = bench_suite_add_clip(&suite_a, "stream_up_return", HAND_DIRECTION_UP);
    if (up_stream == NULL) goto cleanup;
    up_stream->expected_trials = 10U;

    /* 10 UP swipes with immediate in-ROI DOWN return-to-center during 2000ms cooldown */
    for (s = 0U; s < 10U; ++s) {
        const size_t n_swipe = (s % 3U == 1U) ? 5U : ((s % 3U == 2U) ? 11U : 10U);
        const int drop_mid = (s % 3U == 2U);
        const float y_start = 0.72F;
        const float y_end = (s % 3U == 1U) ? 0.28F : 0.30F;

        for (i = 0U; i < n_swipe; ++i) {
            const float prog = (float)i / (float)(n_swipe - 1U);
            if (drop_mid && i == 4U) {
                const bench_frame empty = make_empty_frame(t);
                if (bench_clip_add_frame(up_stream, &empty) != 0) goto cleanup;
            } else {
                const bench_frame f = make_det_frame(
                    t, 0.498F + rand_signed(&rng, 0.002F),
                    y_start + (y_end - y_start) * prog + rand_signed(&rng, 0.0015F),
                    0.050F, 0.070F, 0.90F);
                if (bench_clip_add_frame(up_stream, &f) != 0) goto cleanup;
            }
            t += dt_ms;
        }

        /* Immediate return DOWN inside strict ROI (~300ms total), well within 2000ms cooldown */
        for (i = 0U; i < 8U; ++i) {
            const float prog = (float)i / 7.0F;
            const bench_frame f = make_det_frame(
                t, 0.498F + rand_signed(&rng, 0.0025F),
                y_end + (y_start - y_end) * prog, 0.050F, 0.070F, 0.88F);
            if (bench_clip_add_frame(up_stream, &f) != 0) goto cleanup;
            t += dt_ms;
        }

        /* Hold steady in ROI until cooldown clears (~2.2s = 58 frames) */
        for (i = 0U; i < 58U; ++i) {
            const bench_frame f = make_det_frame(
                t, 0.498F + rand_signed(&rng, 0.0015F),
                y_start + rand_signed(&rng, 0.0015F), 0.050F, 0.070F, 0.89F);
            if (bench_clip_add_frame(up_stream, &f) != 0) goto cleanup;
            t += dt_ms;
        }
    }

    /* Continuous negative stream (~15s): idle hold + edge swipes + 2 L-hooks */
    neg_stream = bench_suite_add_clip(&suite_a, "stream_neg", HAND_DIRECTION_NONE);
    if (neg_stream == NULL) goto cleanup;
    t = 1000U;
    for (i = 0U; i < 120U; ++i) {
        const bench_frame f = make_det_frame(
            t, 0.498F + rand_signed(&rng, 0.003F),
            0.500F + rand_signed(&rng, 0.004F), 0.050F, 0.070F, 0.88F);
        if (bench_clip_add_frame(neg_stream, &f) != 0) goto cleanup;
        t += dt_ms;
    }
    for (i = 0U; i < 20U; ++i) {
        const bench_frame f = make_det_frame(
            t, 0.295F, 0.72F - 0.022F * (float)i, 0.050F, 0.070F, 0.88F);
        if (bench_clip_add_frame(neg_stream, &f) != 0) goto cleanup;
        t += dt_ms;
    }
    for (s = 0U; s < 2U; ++s) {
        static const float pts_a[11][2] = {
            {0.400F, 0.600F}, {0.408F, 0.583F}, {0.416F, 0.566F},
            {0.425F, 0.549F}, {0.433F, 0.532F}, {0.442F, 0.515F},
            {0.450F, 0.500F}, {0.483F, 0.502F}, {0.516F, 0.505F},
            {0.550F, 0.508F}, {0.585F, 0.510F}
        };
        /* 304ms blank transition instead of synthetic instant position teleport */
        for (i = 0U; i < 8U; ++i) {
            const bench_frame empty = make_empty_frame(t);
            if (bench_clip_add_frame(neg_stream, &empty) != 0) goto cleanup;
            t += dt_ms;
        }
        for (i = 0U; i < 11U; ++i) {
            const bench_frame f = make_det_frame(
                t, pts_a[i][0], pts_a[i][1], 0.050F, 0.070F, 0.90F);
            if (bench_clip_add_frame(neg_stream, &f) != 0) goto cleanup;
            t += dt_ms;
        }
        for (i = 0U; i < 8U; ++i) {
            const bench_frame empty = make_empty_frame(t);
            if (bench_clip_add_frame(neg_stream, &empty) != 0) goto cleanup;
            t += dt_ms;
        }
        for (i = 0U; i < 80U; ++i) {
            const bench_frame f = make_det_frame(
                t, 0.498F + rand_signed(&rng, 0.003F),
                0.500F + rand_signed(&rng, 0.003F), 0.050F, 0.070F, 0.88F);
            if (bench_clip_add_frame(neg_stream, &f) != 0) goto cleanup;
            t += dt_ms;
        }
    }

    if (evaluate_suite_stream(&suite_a, 0, 0U, &v1_a) != 0 ||
        evaluate_suite_stream(&suite_a, 1, 0U, &v4_a) != 0) {
        (void)fprintf(stderr, "stream self-test: evaluation failed\n");
        goto cleanup;
    }
    if (export_suite_to_csv(&suite_a, tmp_path) != 0 ||
        load_csv_file(&suite_b, tmp_path, 0) != 0) {
        (void)fprintf(stderr, "stream self-test: CSV roundtrip failed\n");
        goto cleanup;
    }
    if (evaluate_suite_stream(&suite_b, 0, 0U, &v1_b) != 0 ||
        evaluate_suite_stream(&suite_b, 1, 0U, &v4_b) != 0) {
        (void)fprintf(stderr, "stream self-test: reloaded evaluation failed\n");
        goto cleanup;
    }

    if (memcmp(v1_a.confusion, v1_b.confusion, sizeof(v1_a.confusion)) != 0 ||
        memcmp(v4_a.confusion, v4_b.confusion, sizeof(v4_a.confusion)) != 0 ||
        v1_a.opposite_return_fp != v1_b.opposite_return_fp ||
        v4_a.opposite_return_fp != v4_b.opposite_return_fp) {
        (void)fprintf(stderr, "stream self-test: CSV roundtrip mismatch\n");
        goto cleanup;
    }

    if (v4_a.pos_correct != 10U || v4_a.pos_missed != 0U ||
        v4_a.opposite_return_fp != 0U || v4_a.pos_wrong_dir != 0U ||
        v4_a.neg_false_triggers != 0U || v4_a.latency_count != 10U ||
        v1_a.pos_correct >= 10U || v1_a.neg_false_triggers == 0U) {
        (void)fprintf(stderr,
                      "stream self-test failed: v1=(%zu/10, ret_fp=%zu, neg_fp=%zu) "
                      "dirv4=(%zu/10, miss=%zu, ret_fp=%zu, wrong=%zu, neg_fp=%zu)\n",
                      v1_a.pos_correct, v1_a.opposite_return_fp, v1_a.neg_false_triggers,
                      v4_a.pos_correct, v4_a.pos_missed, v4_a.opposite_return_fp,
                      v4_a.pos_wrong_dir, v4_a.neg_false_triggers);
        goto cleanup;
    }

    print_markdown_report(stdout, &suite_a, &v1_a, &v4_a);
    status = 0;

cleanup:
    (void)remove(tmp_path);
    bench_metrics_free(&v1_a);
    bench_metrics_free(&v4_a);
    bench_metrics_free(&v1_b);
    bench_metrics_free(&v4_b);
    bench_suite_free(&suite_a);
    bench_suite_free(&suite_b);
    return status;
}

static int run_self_test(void)
{
    bench_suite suite_a;
    bench_suite suite_b;
    bench_metrics v1_a, v4_a, v1_b, v4_b;
    const char *tmp_dir = getenv("TMPDIR");
    char tmp_path[512];
    int status = 1;

    if (tmp_dir == NULL || tmp_dir[0] == '\0') {
        tmp_dir = "/tmp";
    }
    (void)snprintf(tmp_path, sizeof(tmp_path),
                   "%s/hand_direction_bench_selftest.csv", tmp_dir);

    bench_suite_init(&suite_a);
    bench_suite_init(&suite_b);
    (void)memset(&v1_a, 0, sizeof(v1_a));
    (void)memset(&v4_a, 0, sizeof(v4_a));
    (void)memset(&v1_b, 0, sizeof(v1_b));
    (void)memset(&v4_b, 0, sizeof(v4_b));

    if (build_synthetic_ablation_suite(&suite_a) != 0) {
        (void)fprintf(stderr, "self-test: cannot build synthetic suite\n");
        goto cleanup;
    }
    if (evaluate_suite(&suite_a, 0, &v1_a) != 0 ||
        evaluate_suite(&suite_a, 1, &v4_a) != 0) {
        (void)fprintf(stderr, "self-test: evaluation failed\n");
        goto cleanup;
    }

    if (export_suite_to_csv(&suite_a, tmp_path) != 0 ||
        load_csv_file(&suite_b, tmp_path, 0) != 0) {
        (void)fprintf(stderr, "self-test: CSV export/reload failed\n");
        goto cleanup;
    }
    if (evaluate_suite(&suite_b, 0, &v1_b) != 0 ||
        evaluate_suite(&suite_b, 1, &v4_b) != 0) {
        (void)fprintf(stderr, "self-test: reloaded evaluation failed\n");
        goto cleanup;
    }

    if (suite_a.clip_count != 130U || suite_b.clip_count != 130U) {
        (void)fprintf(stderr, "self-test: clip count mismatch (%zu vs %zu)\n",
                      suite_a.clip_count, suite_b.clip_count);
        goto cleanup;
    }
    if (memcmp(v1_a.confusion, v1_b.confusion, sizeof(v1_a.confusion)) != 0 ||
        memcmp(v4_a.confusion, v4_b.confusion, sizeof(v4_a.confusion)) != 0) {
        (void)fprintf(stderr, "self-test: CSV roundtrip changed confusion matrix\n");
        goto cleanup;
    }

    if (v4_a.pos_correct < 76U || v4_a.neg_false_triggers != 0U ||
        v4_a.pos_correct <= v1_a.pos_correct ||
        v4_a.neg_false_triggers >= v1_a.neg_false_triggers) {
        (void)fprintf(stderr,
                      "self-test assertion failed: v1 pos=%zu/%zu neg_fp=%zu/%zu, "
                      "dirv4 pos=%zu/%zu neg_fp=%zu/%zu\n",
                      v1_a.pos_correct, v1_a.pos_clips,
                      v1_a.neg_false_triggers, v1_a.neg_clips,
                      v4_a.pos_correct, v4_a.pos_clips,
                      v4_a.neg_false_triggers, v4_a.neg_clips);
        goto cleanup;
    }

    print_markdown_report(stdout, &suite_a, &v1_a, &v4_a);
    if (run_stream_self_test() != 0) {
        goto cleanup;
    }
    (void)printf("replay_bench self-test passed\n");
    status = 0;

cleanup:
    (void)remove(tmp_path);
    bench_metrics_free(&v1_a);
    bench_metrics_free(&v4_a);
    bench_metrics_free(&v1_b);
    bench_metrics_free(&v4_b);
    bench_suite_free(&suite_a);
    bench_suite_free(&suite_b);
    return status;
}

static int load_input_arg_into_suite(bench_suite *suite,
                                     const char *arg,
                                     int split_sessions,
                                     int *stream_mode)
{
    struct stat st;
    const char *eq;

    if (stat(arg, &st) == 0) {
        return load_path_into_suite(suite, arg, split_sessions);
    }

    eq = strrchr(arg, '=');
    if (eq != NULL && eq != arg && eq[1] != '\0') {
        char *endp = NULL;
        unsigned long long parsed_trials = strtoull(eq + 1, &endp, 10);
        if (endp != eq + 1 && *endp == '\0') {
            const size_t path_len = (size_t)(eq - arg);
            char *clean_path = (char *)malloc(path_len + 1U);
            const size_t prev_count = suite->clip_count;
            size_t j;
            int rc;

            if (clean_path == NULL) return -1;
            (void)memcpy(clean_path, arg, path_len);
            clean_path[path_len] = '\0';
            rc = load_path_into_suite(suite, clean_path, split_sessions);
            free(clean_path);
            if (rc != 0) return -1;

            for (j = prev_count; j < suite->clip_count; ++j) {
                if (suite->clips[j].expected != HAND_DIRECTION_NONE &&
                    parsed_trials > 0ULL) {
                    suite->clips[j].expected_trials = (size_t)parsed_trials;
                }
            }
            if (stream_mode != NULL) {
                *stream_mode = 1;
            }
            return 0;
        }
    }

    return load_path_into_suite(suite, arg, split_sessions);
}

static void print_usage(const char *prog)
{
    (void)fprintf(stderr,
                  "usage:\n"
                  "  %s [--built-in] [--export-suite out.csv] [--markdown out.md]\n"
                  "  %s [--split-sessions] [--roi x0,y0,x1,y1] [--markdown out.md] <csv_or_dir>...\n"
                  "  %s [--stream] [--trials N] [--roi x0,y0,x1,y1] [--markdown out.md] <csv[=N]>...\n"
                  "  %s --self-test\n",
                  prog, prog, prog, prog);
}

int main(int argc, char **argv)
{
    bench_suite suite;
    bench_metrics v1_m, v4_m;
    const char *markdown_path = NULL;
    const char *export_path = NULL;
    size_t default_trials = 0U;
    int stream_mode = 0;
    int split_sessions = 0;
    int force_builtin = 0;
    int input_paths = 0;
    int exit_code = 1;
    int i;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--self-test") == 0) {
            return run_self_test();
        }
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    bench_suite_init(&suite);
    (void)memset(&v1_m, 0, sizeof(v1_m));
    (void)memset(&v4_m, 0, sizeof(v4_m));

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--built-in") == 0) {
            force_builtin = 1;
        } else if (strcmp(argv[i], "--stream") == 0) {
            stream_mode = 1;
        } else if (strcmp(argv[i], "--trials") == 0 && i + 1 < argc) {
            char *endp = NULL;
            unsigned long long n = strtoull(argv[++i], &endp, 10);
            if (endp == argv[i] || *endp != '\0' || n == 0ULL) {
                (void)fprintf(stderr, "invalid --trials '%s'\n", argv[i]);
                goto cleanup;
            }
            default_trials = (size_t)n;
            stream_mode = 1;
        } else if (strcmp(argv[i], "--split-sessions") == 0) {
            split_sessions = 1;
        } else if (strcmp(argv[i], "--roi") == 0 && i + 1 < argc) {
            float x0, y0, x1, y1;
            ++i;
            if (parse_roi_quad(argv[i], &x0, &y0, &x1, &y1) != 0) {
                (void)fprintf(stderr, "invalid --roi '%s'\n", argv[i]);
                goto cleanup;
            }
            suite.roi_x_min = x0;
            suite.roi_y_min = y0;
            suite.roi_x_max = x1;
            suite.roi_y_max = y1;
            suite.has_roi_override = 1;
        } else if (strcmp(argv[i], "--markdown") == 0 && i + 1 < argc) {
            markdown_path = argv[++i];
        } else if (strcmp(argv[i], "--export-suite") == 0 && i + 1 < argc) {
            export_path = argv[++i];
        } else if (argv[i][0] == '-') {
            print_usage(argv[0]);
            goto cleanup;
        } else {
            if (load_input_arg_into_suite(&suite, argv[i], split_sessions,
                                          &stream_mode) != 0) {
                goto cleanup;
            }
            ++input_paths;
        }
    }

    if (input_paths == 0 || force_builtin) {
        if (build_synthetic_ablation_suite(&suite) != 0) {
            (void)fprintf(stderr, "cannot build synthetic suite\n");
            goto cleanup;
        }
    }

    if (export_path != NULL && export_suite_to_csv(&suite, export_path) != 0) {
        goto cleanup;
    }

    if (suite.clip_count == 0U) {
        (void)fprintf(stderr, "no clips loaded\n");
        goto cleanup;
    }

    if (stream_mode) {
        if (evaluate_suite_stream(&suite, 0, default_trials, &v1_m) != 0 ||
            evaluate_suite_stream(&suite, 1, default_trials, &v4_m) != 0) {
            (void)fprintf(stderr, "stream benchmark evaluation failed\n");
            goto cleanup;
        }
    } else {
        if (evaluate_suite(&suite, 0, &v1_m) != 0 ||
            evaluate_suite(&suite, 1, &v4_m) != 0) {
            (void)fprintf(stderr, "benchmark evaluation failed\n");
            goto cleanup;
        }
    }

    print_markdown_report(stdout, &suite, &v1_m, &v4_m);
    if (markdown_path != NULL) {
        FILE *md = fopen(markdown_path, "w");
        if (md == NULL) {
            (void)fprintf(stderr, "cannot write markdown '%s': %s\n",
                          markdown_path, strerror(errno));
            goto cleanup;
        }
        print_markdown_report(md, &suite, &v1_m, &v4_m);
        (void)fclose(md);
    }
    exit_code = 0;

cleanup:
    bench_metrics_free(&v1_m);
    bench_metrics_free(&v4_m);
    bench_suite_free(&suite);
    return exit_code;
}
