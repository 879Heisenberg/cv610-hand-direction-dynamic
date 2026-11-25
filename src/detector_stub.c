#include "hand_direction/detector.h"

#include "detector_internal.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct {
    int opened;
} detector_stub_context;

static void stub_message(char *buffer, size_t size, const char *text)
{
    if (buffer != NULL && size > 0U) {
        (void)snprintf(buffer, size, "%s", text != NULL ? text : "");
    }
}

static int stub_open(void *context,
                     const char *model_path,
                     char *error_message,
                     size_t error_message_size)
{
    detector_stub_context *stub = (detector_stub_context *)context;

    (void)model_path;
    if (stub == NULL) {
        stub_message(error_message, error_message_size, "stub context is null");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }

    stub->opened = 1;
    stub_message(error_message, error_message_size, "");
    return HAND_DETECTOR_OK;
}

static int stub_infer(void *context,
                      const hand_frame *frame,
                      hand_detection *detections,
                      size_t detection_capacity,
                      size_t *detection_count,
                      char *error_message,
                      size_t error_message_size)
{
    detector_stub_context *stub = (detector_stub_context *)context;
    uint64_t phase_ms;
    float center_x = 0.20F;
    const float box_width = 0.14F;
    const float box_height = 0.20F;

    if (stub == NULL || !stub->opened || frame == NULL || detections == NULL ||
        detection_count == NULL || detection_capacity == 0U) {
        stub_message(error_message, error_message_size, "stub detector is not ready");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }

    *detection_count = 0U;
    phase_ms = frame->timestamp_ms % 2400U;
    if (phase_ms >= 200U && phase_ms <= 1600U) {
        center_x += 0.60F * ((float)(phase_ms - 200U) / 1400.0F);
    } else if (phase_ms > 1600U) {
        center_x = 0.80F;
    }

    detections[0].x0 = center_x - box_width * 0.5F;
    detections[0].y0 = 0.50F - box_height * 0.5F;
    detections[0].x1 = center_x + box_width * 0.5F;
    detections[0].y1 = 0.50F + box_height * 0.5F;
    detections[0].confidence = 0.92F;
    detections[0].class_id = 0;
    *detection_count = 1U;

    stub_message(error_message, error_message_size, "");
    return HAND_DETECTOR_OK;
}

static void stub_destroy(void *context)
{
    free(context);
}

static const hand_detector_ops g_stub_ops = {
    .name = "stub",
    .open = stub_open,
    .infer = stub_infer,
    .destroy = stub_destroy
};

hand_detector *hand_detector_create_stub(void)
{
    detector_stub_context *context =
        (detector_stub_context *)calloc(1U, sizeof(*context));

    if (context == NULL) {
        return NULL;
    }

    return hand_detector_wrap(&g_stub_ops, context);
}
