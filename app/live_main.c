#include "audio_announcer.h"
#include "display_drm.h"
#include "live_media.h"

#include "hand_direction/detector.h"
#include "hand_direction/gesture_engine.h"
#include "hand_direction/output.h"

#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define LIVE_READ_TIMEOUT_MS 500
#define LIVE_MAX_CONSECUTIVE_FAILURES 3U
#define LIVE_INFERENCE_INTERVAL 1U
#define LIVE_LOG_INTERVAL 30U
#define LIVE_INFERENCE_BUFFER_COUNT 2U
#define LIVE_DISPLAY_BUFFER_COUNT 2U
#define LIVE_DIRECTION_WINDOW_MS 700U
#define LIVE_GESTURE_ANIMATION_MS 1000U
#define LIVE_GESTURE_ROI_X_FRACTION 0.80F
#define LIVE_GESTURE_ROI_Y_FRACTION 0.60F
#define LIVE_RECORD_BUFFER_BYTES 65536U
#define LIVE_RECORD_FLUSH_INTERVAL 60U

typedef struct {
    hand_detection detections[HAND_DETECTION_MAX_COUNT];
    size_t detection_count;
    hand_gesture_snapshot gesture;
    hand_detector_timing timing;
    double infer_ms;
    uint64_t capture_ms;
    uint64_t completed_ms;
    uint64_t sequence;
    uint64_t gesture_event_count;
} live_inference_result;

typedef struct {
    hand_detector *detector;
    hand_audio_announcer *announcer;
    hand_gesture_engine gesture_engine;
    hand_gesture_engine_config engine_config;
    char model_path[512];
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    int thread_started;
    int thread_joined;
    int startup_complete;
    int stop_requested;
    int failed;
    int pending_slot;
    int active_slot;
    uint8_t *frames[LIVE_INFERENCE_BUFFER_COUNT];
    uint32_t widths[LIVE_INFERENCE_BUFFER_COUNT];
    uint32_t heights[LIVE_INFERENCE_BUFFER_COUNT];
    uint64_t capture_times_ms[LIVE_INFERENCE_BUFFER_COUNT];
    int has_result;
    live_inference_result latest;
    uint64_t sequence;
    uint64_t gesture_event_count;
    FILE *record_file;
    char *record_io_buf;
    char failure_message[256];
} live_inference_worker;

typedef struct {
    hand_display *display;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    int thread_started;
    int thread_joined;
    int stop_requested;
    int failed;
    int pending_slot;
    int active_slot;
    uint8_t *frames[LIVE_DISPLAY_BUFFER_COUNT];
    uint32_t widths[LIVE_DISPLAY_BUFFER_COUNT];
    uint32_t heights[LIVE_DISPLAY_BUFFER_COUNT];
    hand_detection detections[LIVE_DISPLAY_BUFFER_COUNT][HAND_DETECTION_MAX_COUNT];
    size_t detection_counts[LIVE_DISPLAY_BUFFER_COUNT];
    hand_display_overlay overlays[LIVE_DISPLAY_BUFFER_COUNT];
    uint64_t sequences[LIVE_DISPLAY_BUFFER_COUNT];
    uint64_t submitted_count;
    uint64_t rendered_count;
    uint64_t replaced_count;
    double render_ms_sum;
    double render_cpu_ms_sum;
    double drm_commit_ms_sum;
    char failure_message[256];
} live_display_worker;

typedef struct {
    uint64_t submitted_count;
    uint64_t rendered_count;
    uint64_t replaced_count;
    double render_ms_sum;
    double render_cpu_ms_sum;
    double drm_commit_ms_sum;
} live_display_stats;

static volatile sig_atomic_t g_stop_requested = 0;

static int trace_frames_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *value = getenv("HAND_DIRECTION_TRACE_FRAMES");
        cached = value != NULL && value[0] != '\0' && strcmp(value, "0") != 0;
    }
    return cached;
}

static void trace_frame(const char *stage,
                        uint64_t frame,
                        uint64_t sequence,
                        size_t detections)
{
    if (!trace_frames_enabled()) {
        return;
    }
    (void)fprintf(stderr,
                  "[trace] %s frame=%llu seq=%llu detections=%zu\n",
                  stage,
                  (unsigned long long)frame,
                  (unsigned long long)sequence,
                  detections);
    (void)fflush(stderr);
}

static void trace_mark(const char *stage)
{
    if (!trace_frames_enabled()) {
        return;
    }
    (void)fprintf(stderr, "[trace] %s\n", stage);
    (void)fflush(stderr);
}

static int show_raw_boxes_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *value = getenv("HAND_DIRECTION_SHOW_RAW_BOXES");
        cached = value != NULL && value[0] != '\0' && strcmp(value, "0") != 0;
    }
    return cached;
}

static int trace_gesture_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *value = getenv("HAND_DIRECTION_TRACE_GESTURE");
        cached = value != NULL && value[0] != '\0' && strcmp(value, "0") != 0;
    }
    return cached;
}

static void trace_gesture_decision(const live_inference_worker *worker,
                                   hand_engine_state prev_state,
                                   const hand_gesture_event *event)
{
    hand_gesture_diagnostics diag;
    const hand_direction_diagnostics *dir;
    int near_candidate;

    if (!trace_gesture_enabled() || worker == NULL || event == NULL) {
        return;
    }

    hand_gesture_engine_get_diagnostics(&worker->gesture_engine, &diag);
    dir = &diag.direction;
    near_candidate =
        diag.state == HAND_ENGINE_CAPTURING &&
        dir->active_point_count >= (size_t)worker->engine_config.direction.min_samples &&
        (dir->displacement >= 0.70F * worker->engine_config.direction.min_displacement ||
         dir->candidate_direction != HAND_DIRECTION_NONE);

    if (!event->valid && diag.state == prev_state && !near_candidate) {
        return;
    }

    (void)fprintf(stderr,
                  "[gesture_trace] state=%s->%s roi_in=%d roi=(%.2f..%.2f,%.2f..%.2f) "
                  "act_conf=%.2f raw=(%.3f,%.3f) smooth=(%.3f,%.3f) "
                  "pts=%zu/%zu span=%llu ms d=(%.3f,%.3f) disp=%.3f path=%.3f "
                  "lin=%.2f axis=%.2f seg=%.2f(%u/%u) cos=%.2f streak=%u "
                  "cand=%s emit=%s reason=%s\n",
                  hand_gesture_engine_state_name(prev_state),
                  hand_gesture_engine_state_name(diag.state),
                  diag.roi_inside,
                  (double)diag.roi_x_min,
                  (double)diag.roi_x_max,
                  (double)diag.roi_y_min,
                  (double)diag.roi_y_max,
                  (double)diag.activation_confidence,
                  (double)(diag.has_raw_center ? diag.raw_center.x : -1.0F),
                  (double)(diag.has_raw_center ? diag.raw_center.y : -1.0F),
                  (double)(diag.has_center ? diag.center.x : -1.0F),
                  (double)(diag.has_center ? diag.center.y : -1.0F),
                  dir->point_count,
                  dir->active_point_count,
                  (unsigned long long)dir->span_ms,
                  (double)dir->dx,
                  (double)dir->dy,
                  (double)dir->displacement,
                  (double)dir->path_length,
                  (double)dir->linearity,
                  (double)dir->axis_ratio,
                  (double)dir->segment_consistency,
                  dir->consistent_segments,
                  dir->moving_segments,
                  (double)dir->half_cosine,
                  dir->candidate_streak,
                  hand_direction_name(dir->candidate_direction),
                  hand_direction_name(event->valid ? event->direction : dir->emitted_direction),
                  dir->reject_reason != NULL ? dir->reject_reason : "none");
    (void)fflush(stderr);
}

static void handle_stop_signal(int signal_number)
{
    (void)signal_number;
    g_stop_requested = 1;
}

static uint64_t monotonic_now_ms(void)
{
    struct timespec value;

    if (clock_gettime(CLOCK_MONOTONIC, &value) == 0) {
        return (uint64_t)value.tv_sec * 1000ULL + (uint64_t)value.tv_nsec / 1000000ULL;
    }
    return (uint64_t)time(NULL) * 1000ULL;
}

static double elapsed_ms(uint64_t start_ms, uint64_t end_ms)
{
    if (start_ms == 0U || end_ms < start_ms) {
        return 0.0;
    }
    return (double)(end_ms - start_ms);
}

static void copy_text(char *destination, size_t destination_size, const char *source)
{
    if (destination == NULL || destination_size == 0U) {
        return;
    }
    if (source == NULL) {
        destination[0] = '\0';
        return;
    }
    (void)snprintf(destination, destination_size, "%s", source);
}

static int inference_worker_open_recording(live_inference_worker *worker,
                                           const char *path,
                                           const char *label)
{
    FILE *file;
    long existing_size = 0L;

    if (worker == NULL || path == NULL || path[0] == '\0') {
        return 0;
    }
    file = fopen(path, "a");
    if (file == NULL) {
        (void)fprintf(stderr, "warning: cannot open recording CSV '%s'\n", path);
        return -1;
    }
    worker->record_io_buf = (char *)malloc(LIVE_RECORD_BUFFER_BYTES);
    if (worker->record_io_buf != NULL) {
        (void)setvbuf(file, worker->record_io_buf, _IOFBF, LIVE_RECORD_BUFFER_BYTES);
    }
    if (fseek(file, 0L, SEEK_END) == 0) {
        existing_size = ftell(file);
    }
    if (existing_size <= 0L) {
        (void)fprintf(file,
                      "# hand_direction_record_v1 width=%d height=%d "
                      "roi=%.6f,%.6f,%.6f,%.6f\n",
                      HAND_LIVE_MEDIA_WIDTH,
                      HAND_LIVE_MEDIA_HEIGHT,
                      (double)worker->engine_config.roi_x_min,
                      (double)worker->engine_config.roi_y_min,
                      (double)worker->engine_config.roi_x_max,
                      (double)worker->engine_config.roi_y_max);
        (void)fprintf(file,
                      "# columns: seq,capture_ms,infer_ms,prep_ms,npu_ms,det_count"
                      "[,x0,y0,x1,y1,conf,class_id]...\n");
    }
    if (label != NULL && label[0] != '\0') {
        (void)fprintf(file, "# clip label=%s\n", label);
    }
    (void)fflush(file);
    worker->record_file = file;
    return 0;
}

static void record_detection_frame(live_inference_worker *worker,
                                   uint64_t sequence,
                                   uint64_t capture_ms,
                                   double infer_ms,
                                   const hand_detector_timing *timing,
                                   const hand_detection *detections,
                                   size_t detection_count)
{
    size_t index;

    if (worker == NULL || worker->record_file == NULL) {
        return;
    }
    (void)fprintf(worker->record_file,
                  "%llu,%llu,%.3f,%.3f,%.3f,%zu",
                  (unsigned long long)sequence,
                  (unsigned long long)capture_ms,
                  infer_ms,
                  timing != NULL ? timing->preprocess_ms : 0.0,
                  timing != NULL ? timing->npu_execute_ms : 0.0,
                  detection_count);
    for (index = 0U; index < detection_count; ++index) {
        (void)fprintf(worker->record_file,
                      ",%.5f,%.5f,%.5f,%.5f,%.4f,%d",
                      (double)detections[index].x0,
                      (double)detections[index].y0,
                      (double)detections[index].x1,
                      (double)detections[index].y1,
                      (double)detections[index].confidence,
                      detections[index].class_id);
    }
    (void)fputc('\n', worker->record_file);
    if ((sequence % LIVE_RECORD_FLUSH_INTERVAL) == 0U) {
        (void)fflush(worker->record_file);
    }
}

static void inference_worker_fail_locked(live_inference_worker *worker,
                                         const char *message)
{
    worker->failed = 1;
    worker->startup_complete = 1;
    copy_text(worker->failure_message, sizeof(worker->failure_message), message);
    (void)pthread_cond_broadcast(&worker->condition);
}

static void *inference_worker_main(void *argument)
{
    live_inference_worker *worker = (live_inference_worker *)argument;
    char error_message[256] = {0};
    int detector_status;

    worker->detector = hand_detector_create_cv610();
    if (worker->detector == NULL) {
        (void)pthread_mutex_lock(&worker->mutex);
        inference_worker_fail_locked(worker, "cannot create CV610 detector");
        (void)pthread_mutex_unlock(&worker->mutex);
        return NULL;
    }

    detector_status = hand_detector_open(worker->detector,
                                         worker->model_path,
                                         error_message,
                                         sizeof(error_message));
    (void)pthread_mutex_lock(&worker->mutex);
    if (detector_status != HAND_DETECTOR_OK) {
        char failure[256];

        (void)snprintf(failure,
                       sizeof(failure),
                       "model open failed (%d): %s",
                       detector_status,
                       error_message[0] != '\0' ? error_message : "unknown error");
        inference_worker_fail_locked(worker, failure);
        (void)pthread_mutex_unlock(&worker->mutex);
        hand_detector_destroy(worker->detector);
        worker->detector = NULL;
        return NULL;
    }
    worker->startup_complete = 1;
    (void)pthread_cond_broadcast(&worker->condition);
    (void)pthread_mutex_unlock(&worker->mutex);

    for (;;) {
        int slot;
        hand_frame frame;
        hand_detection detections[HAND_DETECTION_MAX_COUNT];
        size_t detection_count = 0U;
        hand_gesture_event event;
        hand_gesture_snapshot snapshot;
        hand_detector_timing timing;
        uint64_t capture_ms;
        uint64_t inference_start_ms;
        uint64_t completed_ms;
        double inference_ms;
        int status;

        (void)pthread_mutex_lock(&worker->mutex);
        while (!worker->stop_requested && worker->pending_slot < 0) {
            (void)pthread_cond_wait(&worker->condition, &worker->mutex);
        }
        if (worker->stop_requested) {
            (void)pthread_mutex_unlock(&worker->mutex);
            break;
        }
        slot = worker->pending_slot;
        worker->pending_slot = -1;
        worker->active_slot = slot;
        capture_ms = worker->capture_times_ms[slot];
        (void)pthread_mutex_unlock(&worker->mutex);

        frame.data = worker->frames[slot];
        frame.width = worker->widths[slot];
        frame.height = worker->heights[slot];
        frame.stride = frame.width;
        frame.format = HAND_PIXEL_FORMAT_NV21;
        frame.timestamp_ms = capture_ms;

        error_message[0] = '\0';
        inference_start_ms = monotonic_now_ms();
        status = hand_detector_infer(worker->detector,
                                     &frame,
                                     detections,
                                     HAND_DETECTION_MAX_COUNT,
                                     &detection_count,
                                     error_message,
                                     sizeof(error_message));
        completed_ms = monotonic_now_ms();
        inference_ms = elapsed_ms(inference_start_ms, completed_ms);
        (void)memset(&timing, 0, sizeof(timing));
        (void)hand_detector_get_last_timing(worker->detector, &timing);

        if (detection_count > 0U) {
            trace_frame("worker infer done",
                        worker->sequence + 1U,
                        capture_ms,
                        detection_count);
        }

        if (status != HAND_DETECTOR_OK) {
            char failure[256];

            (void)snprintf(failure,
                           sizeof(failure),
                           "inference failed (%d): %s",
                           status,
                           error_message[0] != '\0' ? error_message : "unknown error");
            (void)pthread_mutex_lock(&worker->mutex);
            worker->active_slot = -1;
            inference_worker_fail_locked(worker, failure);
            (void)pthread_mutex_unlock(&worker->mutex);
            break;
        }
        record_detection_frame(worker,
                               worker->sequence + 1U,
                               capture_ms,
                               inference_ms,
                               &timing,
                               detections,
                               detection_count);

        (void)memset(&event, 0, sizeof(event));
        if (detection_count > 0U) {
            trace_frame("worker gesture begin",
                        worker->sequence + 1U,
                        capture_ms,
                        detection_count);
        }
        {
            const hand_engine_state prev_state = worker->gesture_engine.state;
            if (hand_gesture_engine_update(&worker->gesture_engine,
                                           detections,
                                           detection_count,
                                           capture_ms,
                                           &event) != 0) {
                (void)pthread_mutex_lock(&worker->mutex);
                worker->active_slot = -1;
                inference_worker_fail_locked(worker, "gesture engine update failed");
                (void)pthread_mutex_unlock(&worker->mutex);
                break;
            }
            trace_gesture_decision(worker, prev_state, &event);
        }
        if (detection_count > 0U) {
            trace_frame("worker gesture done",
                        worker->sequence + 1U,
                        capture_ms,
                        detection_count);
        }
        (void)hand_gesture_engine_snapshot(&worker->gesture_engine,
                                           completed_ms,
                                           &snapshot);

        if (detection_count > 0U) {
            trace_frame("worker snapshot done",
                        worker->sequence + 1U,
                        capture_ms,
                        (size_t)snapshot.has_center);
        }

        (void)pthread_mutex_lock(&worker->mutex);
        if (worker->stop_requested) {
            worker->active_slot = -1;
            (void)pthread_mutex_unlock(&worker->mutex);
            break;
        }
        worker->latest.capture_ms = capture_ms;
        worker->latest.completed_ms = completed_ms;
        worker->latest.infer_ms = inference_ms;
        worker->latest.timing = timing;
        worker->latest.detection_count = detection_count;
        if (detection_count > 0U) {
            (void)memcpy(worker->latest.detections,
                         detections,
                         detection_count * sizeof(detections[0]));
        }
        worker->latest.gesture = snapshot;
        if (event.valid) {
            ++worker->gesture_event_count;
        }
        worker->latest.gesture_event_count = worker->gesture_event_count;
        worker->latest.sequence = ++worker->sequence;
        worker->has_result = 1;
        worker->active_slot = -1;
        (void)pthread_cond_broadcast(&worker->condition);
        (void)pthread_mutex_unlock(&worker->mutex);

        if (detection_count > 0U) {
            trace_frame("worker publish done",
                        worker->sequence,
                        capture_ms,
                        detection_count);
        }

        if (event.valid) {
            hand_audio_announcer_post_direction(worker->announcer, event.direction);
            hand_gesture_output_console(stdout, &event);
        }
    }

    hand_detector_destroy(worker->detector);
    worker->detector = NULL;
    return NULL;
}

static live_inference_worker *inference_worker_create(
    const char *model_path,
    hand_audio_announcer *announcer)
{
    live_inference_worker *worker;
    int index;

    worker = (live_inference_worker *)calloc(1U, sizeof(*worker));
    if (worker == NULL) {
        return NULL;
    }
    worker->pending_slot = -1;
    worker->active_slot = -1;
    copy_text(worker->failure_message, sizeof(worker->failure_message), "");
    copy_text(worker->model_path, sizeof(worker->model_path), model_path);
    worker->announcer = announcer;

    if (pthread_mutex_init(&worker->mutex, NULL) != 0) {
        free(worker);
        return NULL;
    }
    if (pthread_cond_init(&worker->condition, NULL) != 0) {
        (void)pthread_mutex_destroy(&worker->mutex);
        free(worker);
        return NULL;
    }

    for (index = 0; index < LIVE_INFERENCE_BUFFER_COUNT; ++index) {
        worker->frames[index] = (uint8_t *)malloc(HAND_LIVE_MEDIA_NV21_BYTES);
        if (worker->frames[index] == NULL) {
            int allocated;

            for (allocated = 0; allocated < index; ++allocated) {
                free(worker->frames[allocated]);
            }
            (void)pthread_cond_destroy(&worker->condition);
            (void)pthread_mutex_destroy(&worker->mutex);
            free(worker);
            return NULL;
        }
    }

    {
        int crop_x;
        int crop_y;
        int crop_width;
        int crop_height;

        hand_gesture_engine_config_init_defaults(&worker->engine_config);
        if (hand_display_visible_rect(HAND_LIVE_MEDIA_WIDTH,
                                      HAND_LIVE_MEDIA_HEIGHT,
                                      &crop_x,
                                      &crop_y,
                                      &crop_width,
                                      &crop_height) == 0) {
            const float visible_x_min =
                (float)crop_x / (float)HAND_LIVE_MEDIA_WIDTH;
            const float visible_y_min =
                (float)crop_y / (float)HAND_LIVE_MEDIA_HEIGHT;
            const float visible_x_max =
                (float)(crop_x + crop_width) / (float)HAND_LIVE_MEDIA_WIDTH;
            const float visible_y_max =
                (float)(crop_y + crop_height) / (float)HAND_LIVE_MEDIA_HEIGHT;
            const float inset_x = (1.0F - LIVE_GESTURE_ROI_X_FRACTION) * 0.5F;
            const float inset_y = (1.0F - LIVE_GESTURE_ROI_Y_FRACTION) * 0.5F;

            /* Edge gestures are measured in LCD-visible coordinates, not in
             * the wider camera frame that the user cannot see. */
            worker->engine_config.roi_x_min =
                visible_x_min + (visible_x_max - visible_x_min) * inset_x;
            worker->engine_config.roi_x_max =
                visible_x_max - (visible_x_max - visible_x_min) * inset_x;
            worker->engine_config.roi_y_min =
                visible_y_min + (visible_y_max - visible_y_min) * inset_y;
            worker->engine_config.roi_y_max =
                visible_y_max - (visible_y_max - visible_y_min) * inset_y;
        }
    }

    worker->engine_config.direction.window_ms = LIVE_DIRECTION_WINDOW_MS;
    worker->engine_config.direction.x_scale =
        (float)HAND_LIVE_MEDIA_WIDTH / (float)HAND_LIVE_MEDIA_HEIGHT;
    worker->engine_config.event_animation_ms = LIVE_GESTURE_ANIMATION_MS;
    hand_gesture_engine_init(&worker->gesture_engine, &worker->engine_config);
    return worker;
}

static int inference_worker_start(live_inference_worker *worker)
{
    int result;
    int failed;
    char failure[256];

    if (worker == NULL) {
        return -1;
    }
    result = pthread_create(&worker->thread, NULL, inference_worker_main, worker);
    if (result != 0) {
        (void)fprintf(stderr, "cannot create inference thread: error %d\n", result);
        return -1;
    }
    worker->thread_started = 1;

    (void)pthread_mutex_lock(&worker->mutex);
    while (!worker->startup_complete) {
        (void)pthread_cond_wait(&worker->condition, &worker->mutex);
    }
    failed = worker->failed;
    copy_text(failure, sizeof(failure), worker->failure_message);
    (void)pthread_mutex_unlock(&worker->mutex);

    if (failed) {
        (void)fprintf(stderr, "%s\n", failure);
        return -1;
    }
    return 0;
}

static void inference_worker_submit(live_inference_worker *worker,
                                    const uint8_t *frame,
                                    uint32_t width,
                                    uint32_t height,
                                    uint64_t capture_ms)
{
    int slot;

    if (worker == NULL || frame == NULL ||
        width == 0U || height == 0U || width != HAND_LIVE_MEDIA_WIDTH ||
        height != HAND_LIVE_MEDIA_HEIGHT) {
        return;
    }

    (void)pthread_mutex_lock(&worker->mutex);
    if (worker->stop_requested || worker->failed) {
        (void)pthread_mutex_unlock(&worker->mutex);
        return;
    }

    if (worker->pending_slot >= 0) {
        slot = worker->pending_slot;
    } else {
        slot = worker->active_slot == 0 ? 1 : 0;
    }
    (void)memcpy(worker->frames[slot], frame, HAND_LIVE_MEDIA_NV21_BYTES);
    worker->widths[slot] = width;
    worker->heights[slot] = height;
    worker->capture_times_ms[slot] = capture_ms;
    worker->pending_slot = slot;
    (void)pthread_cond_signal(&worker->condition);
    (void)pthread_mutex_unlock(&worker->mutex);
}

static int inference_worker_copy_latest(live_inference_worker *worker,
                                        live_inference_result *result)
{
    if (worker == NULL || result == NULL) {
        return 0;
    }

    (void)pthread_mutex_lock(&worker->mutex);
    if (!worker->has_result) {
        (void)pthread_mutex_unlock(&worker->mutex);
        return 0;
    }
    *result = worker->latest;
    (void)pthread_mutex_unlock(&worker->mutex);
    return 1;
}

static int inference_worker_get_failure(live_inference_worker *worker,
                                        char *error_message,
                                        size_t error_message_size)
{
    int failed;

    if (worker == NULL) {
        return 0;
    }
    (void)pthread_mutex_lock(&worker->mutex);
    failed = worker->failed;
    if (failed) {
        copy_text(error_message, error_message_size, worker->failure_message);
    }
    (void)pthread_mutex_unlock(&worker->mutex);
    return failed;
}

static void inference_worker_request_stop(live_inference_worker *worker)
{
    if (worker == NULL) {
        return;
    }
    (void)pthread_mutex_lock(&worker->mutex);
    worker->stop_requested = 1;
    (void)pthread_cond_broadcast(&worker->condition);
    (void)pthread_mutex_unlock(&worker->mutex);
}

static void inference_worker_destroy(live_inference_worker *worker)
{
    int index;

    if (worker == NULL) {
        return;
    }
    if (worker->thread_started && !worker->thread_joined) {
        inference_worker_request_stop(worker);
        (void)pthread_join(worker->thread, NULL);
        worker->thread_joined = 1;
    }
    if (worker->record_file != NULL) {
        (void)fflush(worker->record_file);
        (void)fclose(worker->record_file);
        worker->record_file = NULL;
    }
    free(worker->record_io_buf);
    worker->record_io_buf = NULL;
    for (index = 0; index < LIVE_INFERENCE_BUFFER_COUNT; ++index) {
        free(worker->frames[index]);
    }
    (void)pthread_cond_destroy(&worker->condition);
    (void)pthread_mutex_destroy(&worker->mutex);
    free(worker);
}

static void live_display_worker_fail_locked(live_display_worker *worker,
                                            const char *message)
{
    worker->failed = 1;
    copy_text(worker->failure_message, sizeof(worker->failure_message), message);
    (void)pthread_cond_broadcast(&worker->condition);
}

static void *live_display_worker_main(void *argument)
{
    live_display_worker *worker = (live_display_worker *)argument;

    for (;;) {
        char error_message[256] = {0};
        hand_display_timing timing;
        uint64_t render_start_ms;
        unsigned int slot;
        int status;

        (void)pthread_mutex_lock(&worker->mutex);
        while (!worker->stop_requested && worker->pending_slot < 0) {
            (void)pthread_cond_wait(&worker->condition, &worker->mutex);
        }
        if (worker->stop_requested) {
            (void)pthread_mutex_unlock(&worker->mutex);
            break;
        }
        slot = (unsigned int)worker->pending_slot;
        worker->pending_slot = -1;
        worker->active_slot = (int)slot;
        (void)pthread_mutex_unlock(&worker->mutex);

        render_start_ms = monotonic_now_ms();
        status = hand_display_render(worker->display,
                                     worker->frames[slot],
                                     worker->widths[slot],
                                     worker->heights[slot],
                                     worker->detection_counts[slot] > 0U
                                         ? worker->detections[slot]
                                         : NULL,
                                     worker->detection_counts[slot],
                                     &worker->overlays[slot],
                                     error_message,
                                     sizeof(error_message));
        hand_display_get_last_timing(worker->display, &timing);

        (void)pthread_mutex_lock(&worker->mutex);
        worker->active_slot = -1;
        if (status != 0) {
            live_display_worker_fail_locked(
                worker,
                error_message[0] != '\0' ? error_message : "display render failed");
            (void)pthread_mutex_unlock(&worker->mutex);
            break;
        }
        ++worker->rendered_count;
        worker->render_ms_sum += elapsed_ms(render_start_ms, monotonic_now_ms());
        worker->render_cpu_ms_sum += timing.render_cpu_ms;
        worker->drm_commit_ms_sum += timing.drm_commit_ms;
        (void)pthread_cond_broadcast(&worker->condition);
        (void)pthread_mutex_unlock(&worker->mutex);
    }

    return NULL;
}

static live_display_worker *live_display_worker_create(void)
{
    live_display_worker *worker;
    int index;

    worker = (live_display_worker *)calloc(1U, sizeof(*worker));
    if (worker == NULL) {
        return NULL;
    }
    worker->pending_slot = -1;
    worker->active_slot = -1;
    copy_text(worker->failure_message, sizeof(worker->failure_message), "");

    if (pthread_mutex_init(&worker->mutex, NULL) != 0) {
        free(worker);
        return NULL;
    }
    if (pthread_cond_init(&worker->condition, NULL) != 0) {
        (void)pthread_mutex_destroy(&worker->mutex);
        free(worker);
        return NULL;
    }

    for (index = 0; index < LIVE_DISPLAY_BUFFER_COUNT; ++index) {
        worker->frames[index] = (uint8_t *)malloc(HAND_LIVE_MEDIA_NV21_BYTES);
        if (worker->frames[index] == NULL) {
            int allocated;

            for (allocated = 0; allocated < index; ++allocated) {
                free(worker->frames[allocated]);
            }
            (void)pthread_cond_destroy(&worker->condition);
            (void)pthread_mutex_destroy(&worker->mutex);
            free(worker);
            return NULL;
        }
    }
    return worker;
}

static int live_display_worker_start(live_display_worker *worker,
                                     hand_display *display)
{
    int result;

    if (worker == NULL || display == NULL) {
        return -1;
    }
    worker->display = display;
    result = pthread_create(&worker->thread, NULL, live_display_worker_main, worker);
    if (result != 0) {
        (void)fprintf(stderr, "cannot create display thread: error %d\n", result);
        return -1;
    }
    worker->thread_started = 1;
    return 0;
}

static int live_display_worker_publish(live_display_worker *worker,
                                       const uint8_t *nv21,
                                       uint32_t width,
                                       uint32_t height,
                                       const hand_detection *detections,
                                       size_t detection_count,
                                       const hand_display_overlay *overlay,
                                       uint64_t sequence)
{
    unsigned int slot;

    if (worker == NULL || nv21 == NULL ||
        width != HAND_LIVE_MEDIA_WIDTH || height != HAND_LIVE_MEDIA_HEIGHT ||
        detection_count > HAND_DETECTION_MAX_COUNT ||
        (detection_count > 0U && detections == NULL)) {
        return -1;
    }

    (void)pthread_mutex_lock(&worker->mutex);
    if (worker->stop_requested || worker->failed) {
        (void)pthread_mutex_unlock(&worker->mutex);
        return -1;
    }

    if (worker->pending_slot >= 0) {
        slot = (unsigned int)worker->pending_slot;
        ++worker->replaced_count;
    } else {
        slot = worker->active_slot == 0 ? 1U : 0U;
    }

    (void)memcpy(worker->frames[slot], nv21, HAND_LIVE_MEDIA_NV21_BYTES);
    worker->widths[slot] = width;
    worker->heights[slot] = height;
    worker->detection_counts[slot] = detection_count;
    if (detection_count > 0U) {
        (void)memcpy(worker->detections[slot],
                     detections,
                     detection_count * sizeof(detections[0]));
    }
    if (overlay != NULL) {
        worker->overlays[slot] = *overlay;
    } else {
        (void)memset(&worker->overlays[slot], 0, sizeof(worker->overlays[slot]));
    }
    worker->sequences[slot] = sequence;
    worker->pending_slot = (int)slot;
    ++worker->submitted_count;
    (void)pthread_cond_signal(&worker->condition);
    (void)pthread_mutex_unlock(&worker->mutex);
    return 0;
}

static void live_display_worker_get_stats(live_display_worker *worker,
                                          live_display_stats *stats)
{
    if (stats == NULL) {
        return;
    }
    (void)memset(stats, 0, sizeof(*stats));
    if (worker == NULL) {
        return;
    }

    (void)pthread_mutex_lock(&worker->mutex);
    stats->submitted_count = worker->submitted_count;
    stats->rendered_count = worker->rendered_count;
    stats->replaced_count = worker->replaced_count;
    stats->render_ms_sum = worker->render_ms_sum;
    stats->render_cpu_ms_sum = worker->render_cpu_ms_sum;
    stats->drm_commit_ms_sum = worker->drm_commit_ms_sum;
    (void)pthread_mutex_unlock(&worker->mutex);
}

static int live_display_worker_get_failure(live_display_worker *worker,
                                           char *error_message,
                                           size_t error_message_size)
{
    int failed;

    if (worker == NULL) {
        return 0;
    }
    (void)pthread_mutex_lock(&worker->mutex);
    failed = worker->failed;
    if (failed) {
        copy_text(error_message, error_message_size, worker->failure_message);
    }
    (void)pthread_mutex_unlock(&worker->mutex);
    return failed;
}

static void live_display_worker_destroy(live_display_worker *worker)
{
    int index;

    if (worker == NULL) {
        return;
    }
    if (worker->thread_started && !worker->thread_joined) {
        (void)pthread_mutex_lock(&worker->mutex);
        worker->stop_requested = 1;
        (void)pthread_cond_broadcast(&worker->condition);
        (void)pthread_mutex_unlock(&worker->mutex);
        (void)pthread_join(worker->thread, NULL);
        worker->thread_joined = 1;
    }
    for (index = 0; index < LIVE_DISPLAY_BUFFER_COUNT; ++index) {
        free(worker->frames[index]);
    }
    (void)pthread_cond_destroy(&worker->condition);
    (void)pthread_mutex_destroy(&worker->mutex);
    free(worker);
}

static void log_status(uint64_t frame_sequence,
                       uint32_t frames_in_interval,
                       uint64_t interval_start_ms,
                       double display_ms,
                       double disp_cpu_ms,
                       double disp_spi_ms,
                       double display_fps,
                       uint64_t display_skipped,
                       int has_result,
                       const live_inference_result *result)
{
    const uint64_t now_ms = monotonic_now_ms();
    const double interval_ms = elapsed_ms(interval_start_ms, now_ms);
    const double camera_fps =
        interval_ms > 0.0 ? (double)frames_in_interval * 1000.0 / interval_ms : 0.0;

    if (has_result && result != NULL) {
        (void)printf("frame=%llu camera_fps=%.2f display_fps=%.2f display_ms=%.3f "
                     "disp_cpu_ms=%.2f disp_spi_ms=%.2f "
                     "display_skipped=%llu infer_ms=%.3f "
                     "prep_ms=%.3f flush_ms=%.3f npu_ms=%.3f post_ms=%.3f "
                     "detections=%zu inference=%llu gesture_events=%llu\n",
                     (unsigned long long)frame_sequence,
                     camera_fps,
                     display_fps,
                     display_ms,
                     disp_cpu_ms,
                     disp_spi_ms,
                     (unsigned long long)display_skipped,
                     result->infer_ms,
                     result->timing.preprocess_ms,
                     result->timing.flush_ms,
                     result->timing.npu_execute_ms,
                     result->timing.postprocess_ms,
                     result->detection_count,
                     (unsigned long long)result->sequence,
                     (unsigned long long)result->gesture_event_count);
    } else {
        (void)printf("frame=%llu camera_fps=%.2f display_fps=%.2f display_ms=%.3f "
                     "disp_cpu_ms=%.2f disp_spi_ms=%.2f "
                     "display_skipped=%llu "
                     "infer_ms=pending npu_ms=pending detections=0\n",
                     (unsigned long long)frame_sequence,
                     camera_fps,
                     display_fps,
                     display_ms,
                     disp_cpu_ms,
                     disp_spi_ms,
                     (unsigned long long)display_skipped);
    }
    (void)fflush(stdout);
}

static void usage(const char *program)
{
    (void)fprintf(stderr, "usage: %s <model.om>\n", program);
}

int main(int argc, char **argv)
{
    struct sigaction stop_action;
    hand_live_media *media = NULL;
    hand_display *display = NULL;
    live_display_worker *display_worker = NULL;
    live_inference_worker *worker = NULL;
    hand_audio_announcer *announcer = NULL;
    const char *gesture_sound_path;
    live_inference_result latest;
    uint8_t *camera_frame = NULL;
    char error_message[256] = {0};
    uint64_t frame_sequence = 0U;
    uint32_t consecutive_failures = 0U;
    uint32_t frames_in_interval = 0U;
    uint64_t interval_start_ms;
    uint64_t previous_display_rendered = 0U;
    uint64_t previous_display_skipped = 0U;
    double previous_display_render_ms = 0.0;
    double previous_display_cpu_ms = 0.0;
    double previous_display_spi_ms = 0.0;
    int has_latest = 0;
    int exit_code = 1;
    int worker_error_reported = 0;
    int display_error_reported = 0;

    if (argc != 2) {
        usage(argv[0]);
        return 2;
    }

    trace_mark("main entered");
    (void)setvbuf(stdout, NULL, _IOLBF, 0U);
    (void)memset(&stop_action, 0, sizeof(stop_action));
    stop_action.sa_handler = handle_stop_signal;
    (void)sigemptyset(&stop_action.sa_mask);
    (void)sigaction(SIGINT, &stop_action, NULL);
    (void)sigaction(SIGTERM, &stop_action, NULL);

    camera_frame = (uint8_t *)malloc(HAND_LIVE_MEDIA_NV21_BYTES);
    if (camera_frame == NULL) {
        (void)fprintf(stderr, "cannot allocate camera frame buffer\n");
        goto cleanup;
    }
    trace_mark("camera frame allocated");

    (void)printf("opening camera pipeline (OS04D10 -> VI/ISP -> VPSS %dx%d NV21)...\n",
                 HAND_LIVE_MEDIA_WIDTH,
                 HAND_LIVE_MEDIA_HEIGHT);
    trace_mark("media open begin");
    if (hand_live_media_open(&media, error_message, sizeof(error_message)) != 0) {
        (void)fprintf(stderr, "camera pipeline open failed: %s\n", error_message);
        goto cleanup;
    }
    trace_mark("media open done");

    if (getenv("HAND_DIRECTION_DISABLE_DISPLAY") != NULL) {
        (void)printf("LCD display disabled by HAND_DIRECTION_DISABLE_DISPLAY\n");
    } else {
        if (hand_display_open(&display, error_message, sizeof(error_message)) != 0) {
            (void)printf("LCD display disabled: %s\n", error_message);
            display = NULL;
        } else {
            (void)printf("LCD output ready: JD9853 240x320 RGB565\n");
        }
    }
    trace_mark("display stage done");

    gesture_sound_path = getenv("HAND_GESTURE_SOUND");
    if (gesture_sound_path == NULL || gesture_sound_path[0] == '\0') {
        gesture_sound_path = "audio/gesture_effect.wav";
    }
    announcer = hand_audio_announcer_create(gesture_sound_path);
    if (announcer == NULL) {
        (void)fprintf(stderr,
                      "gesture sound unavailable (%s); continuing without audio\n",
                      gesture_sound_path);
    } else {
        (void)printf("gesture sound ready: %s\n", gesture_sound_path);
    }

    worker = inference_worker_create(argv[1], announcer);
    if (worker == NULL) {
        (void)fprintf(stderr, "cannot allocate inference worker\n");
        goto cleanup;
    }
    {
        const char *record_path = getenv("HAND_DIRECTION_RECORD_PATH");
        const char *record_label = getenv("HAND_DIRECTION_RECORD_LABEL");

        if (record_path != NULL && record_path[0] != '\0' &&
            inference_worker_open_recording(worker, record_path, record_label) == 0) {
            (void)printf("gesture recording ready: %s\n", record_path);
        }
    }
    (void)printf("loading model: %s\n", argv[1]);
    trace_mark("worker start begin");
    if (inference_worker_start(worker) != 0) {
        goto cleanup;
    }
    trace_mark("worker start done");

    if (display != NULL) {
        display_worker = live_display_worker_create();
        if (display_worker == NULL ||
            live_display_worker_start(display_worker, display) != 0) {
            (void)fprintf(stderr, "LCD display worker unavailable; disabling display\n");
            live_display_worker_destroy(display_worker);
            display_worker = NULL;
            hand_display_close(display);
            display = NULL;
        } else {
            (void)printf("LCD display worker ready (latest-frame mode)\n");
        }
    }
    (void)printf("gesture engine ready: class=%d confidence>=%.2f min_samples=%u "
                 "min_span=%u ms min_disp=%.2f axis>=%.2f cos>=%.2f "
                 "direction_window=%llu ms confirm=%u "
                 "x_scale=%.3f max_spd=%.1f cooldown=%llu ms animation=%llu ms "
                 "roi=(%.2f,%.2f)-(%.2f,%.2f), "
                 "inference_interval=%u frames\n",
                 worker->engine_config.activation_class_id,
                 (double)worker->engine_config.activation_confidence,
                 worker->engine_config.direction.min_samples,
                 worker->engine_config.direction.min_span_ms,
                 (double)worker->engine_config.direction.min_displacement,
                 (double)worker->engine_config.direction.axis_ratio,
                 (double)worker->engine_config.direction.min_half_cosine,
                 (unsigned long long)worker->engine_config.direction.window_ms,
                 worker->engine_config.direction.confirm_count,
                 (double)worker->engine_config.direction.x_scale,
                 (double)worker->engine_config.tracker.max_speed_norm_per_second,
                 (unsigned long long)worker->engine_config.cooldown_ms,
                 (unsigned long long)worker->engine_config.event_animation_ms,
                 (double)worker->engine_config.roi_x_min,
                 (double)worker->engine_config.roi_y_min,
                 (double)worker->engine_config.roi_x_max,
                 (double)worker->engine_config.roi_y_max,
                 LIVE_INFERENCE_INTERVAL);
    trace_mark("main loop begin");

    interval_start_ms = monotonic_now_ms();
    while (!g_stop_requested) {
        uint32_t width = 0U;
        uint32_t height = 0U;
        uint64_t capture_ms;
        hand_display_overlay overlay;
        int overlay_valid = 0;
        const hand_detection *detections = NULL;
        size_t detection_count = 0U;

        error_message[0] = '\0';
        if (hand_live_media_read_nv21(media,
                                      camera_frame,
                                      HAND_LIVE_MEDIA_NV21_BYTES,
                                      LIVE_READ_TIMEOUT_MS,
                                      &width,
                                      &height,
                                      error_message,
                                      sizeof(error_message)) != 0) {
            if (g_stop_requested) {
                break;
            }
            ++consecutive_failures;
            (void)fprintf(stderr,
                          "camera read failed (%u/%u): %s\n",
                          consecutive_failures,
                          LIVE_MAX_CONSECUTIVE_FAILURES,
                          error_message);
            if (consecutive_failures >= LIVE_MAX_CONSECUTIVE_FAILURES) {
                goto cleanup;
            }
            continue;
        }
        consecutive_failures = 0U;
        capture_ms = monotonic_now_ms();
        ++frame_sequence;

        if ((frame_sequence - 1U) % LIVE_INFERENCE_INTERVAL == 0U) {
            inference_worker_submit(worker, camera_frame, width, height, capture_ms);
        }

        (void)memset(&latest, 0, sizeof(latest));
        if (inference_worker_copy_latest(worker, &latest)) {
            has_latest = 1;
        }
        (void)memset(&overlay, 0, sizeof(overlay));
        overlay.show_raw_detections = show_raw_boxes_enabled();
        overlay.roi_valid = 1;
        overlay.roi_x_min = worker->engine_config.roi_x_min;
        overlay.roi_x_max = worker->engine_config.roi_x_max;
        overlay.roi_y_min = worker->engine_config.roi_y_min;
        overlay.roi_y_max = worker->engine_config.roi_y_max;
        overlay.cooldown_total_ms = worker->engine_config.cooldown_ms;
        overlay.state = (int)HAND_ENGINE_WAITING_FOR_HAND;
        if (has_latest) {
            detections = latest.detections;
            detection_count = latest.detection_count;
            overlay.state = (int)latest.gesture.state;
            overlay.center_valid = latest.gesture.has_center;
            overlay.center = latest.gesture.center;
            overlay.tracked_box_valid = latest.gesture.has_tracked_box;
            overlay.tracked_box = latest.gesture.tracked_box;
            overlay.roi_inside = latest.gesture.roi_inside;
            overlay.roi_x_min = latest.gesture.roi_x_min;
            overlay.roi_x_max = latest.gesture.roi_x_max;
            overlay.roi_y_min = latest.gesture.roi_y_min;
            overlay.roi_y_max = latest.gesture.roi_y_max;
            overlay.cooldown_remaining_ms = latest.gesture.cooldown_remaining_ms;
            overlay.cooldown_total_ms = latest.gesture.cooldown_total_ms;
            overlay.event_valid = latest.gesture.event_visible;
            overlay.event = latest.gesture.event;
            overlay.event_animation_ms = worker->engine_config.event_animation_ms;
        }
        overlay_valid = 1;

        if (detection_count > 0U) {
            trace_frame("main before display publish",
                        frame_sequence,
                        latest.sequence,
                        detection_count);
        }

        if (display_worker != NULL) {
            (void)live_display_worker_publish(
                display_worker,
                camera_frame,
                width,
                height,
                detection_count > 0U ? detections : NULL,
                detection_count,
                overlay_valid ? &overlay : NULL,
                latest.sequence);
        }

        if (detection_count > 0U) {
            trace_frame("main after display publish",
                        frame_sequence,
                        latest.sequence,
                        detection_count);
        }

        ++frames_in_interval;
        if (frames_in_interval >= LIVE_LOG_INTERVAL) {
            live_display_stats display_stats;
            uint64_t rendered_delta;
            uint64_t skipped_delta;
            uint64_t current_rendered;
            uint64_t current_skipped;
            double current_render_ms;
            double current_cpu_ms;
            double current_spi_ms;
            double render_ms_delta;
            double cpu_ms_delta;
            double spi_ms_delta;
            double interval_ms;
            double display_ms = 0.0;
            double disp_cpu_ms = 0.0;
            double disp_spi_ms = 0.0;
            double display_fps = 0.0;

            live_display_worker_get_stats(display_worker, &display_stats);
            current_rendered = display_stats.rendered_count;
            current_skipped = display_stats.replaced_count;
            current_render_ms = display_stats.render_ms_sum;
            current_cpu_ms = display_stats.render_cpu_ms_sum;
            current_spi_ms = display_stats.drm_commit_ms_sum;
            rendered_delta = current_rendered >= previous_display_rendered
                                 ? current_rendered - previous_display_rendered
                                 : 0U;
            skipped_delta = current_skipped >= previous_display_skipped
                                ? current_skipped - previous_display_skipped
                                : 0U;
            render_ms_delta = current_render_ms >= previous_display_render_ms
                                  ? current_render_ms - previous_display_render_ms
                                  : 0.0;
            cpu_ms_delta = current_cpu_ms >= previous_display_cpu_ms
                               ? current_cpu_ms - previous_display_cpu_ms
                               : 0.0;
            spi_ms_delta = current_spi_ms >= previous_display_spi_ms
                               ? current_spi_ms - previous_display_spi_ms
                               : 0.0;
            interval_ms = elapsed_ms(interval_start_ms, monotonic_now_ms());
            if (rendered_delta > 0U) {
                display_ms = render_ms_delta / (double)rendered_delta;
                disp_cpu_ms = cpu_ms_delta / (double)rendered_delta;
                disp_spi_ms = spi_ms_delta / (double)rendered_delta;
                if (interval_ms > 0.0) {
                    display_fps = (double)rendered_delta * 1000.0 / interval_ms;
                }
            }
            log_status(frame_sequence,
                       frames_in_interval,
                       interval_start_ms,
                       display_ms,
                       disp_cpu_ms,
                       disp_spi_ms,
                       display_fps,
                       skipped_delta,
                       has_latest,
                       &latest);
            previous_display_rendered = current_rendered;
            previous_display_skipped = current_skipped;
            previous_display_render_ms = current_render_ms;
            previous_display_cpu_ms = current_cpu_ms;
            previous_display_spi_ms = current_spi_ms;
            frames_in_interval = 0U;
            interval_start_ms = monotonic_now_ms();
        }

        if (!worker_error_reported &&
            inference_worker_get_failure(worker, error_message, sizeof(error_message))) {
            (void)fprintf(stderr, "inference worker stopped: %s\n", error_message);
            worker_error_reported = 1;
            goto cleanup;
        }

        if (!display_error_reported && display_worker != NULL &&
            live_display_worker_get_failure(display_worker,
                                            error_message,
                                            sizeof(error_message))) {
            (void)fprintf(stderr,
                          "LCD render failed, disabling display: %s\n",
                          error_message);
            display_error_reported = 1;
            live_display_worker_destroy(display_worker);
            display_worker = NULL;
            hand_display_close(display);
            display = NULL;
            previous_display_rendered = 0U;
            previous_display_skipped = 0U;
            previous_display_render_ms = 0.0;
            previous_display_cpu_ms = 0.0;
            previous_display_spi_ms = 0.0;
        }
    }

    exit_code = 0;

cleanup:
    if (g_stop_requested || worker != NULL) {
        (void)printf("stop requested; releasing camera and NPU resources\n");
    }
    live_display_worker_destroy(display_worker);
    inference_worker_destroy(worker);
    hand_audio_announcer_destroy(announcer);
    hand_display_close(display);
    hand_live_media_close(media);
    free(camera_frame);
    return exit_code;
}
