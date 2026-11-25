#include "hand_direction/detector.h"

#include "detector_internal.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct {
    int reserved;
} detector_cv610_context;

static void cv610_message(char *buffer, size_t size, const char *text)
{
    if (buffer != NULL && size > 0U) {
        (void)snprintf(buffer, size, "%s", text != NULL ? text : "");
    }
}

static int cv610_open(void *context,
                      const char *model_path,
                      char *error_message,
                      size_t error_message_size)
{
    detector_cv610_context *cv610 = (detector_cv610_context *)context;

    (void)model_path;
    if (cv610 == NULL) {
        cv610_message(error_message, error_message_size, "CV610 context is null");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }

    cv610_message(error_message,
                  error_message_size,
                  "CV610 backend placeholder: connect svp_acl and detector .om here");
    return HAND_DETECTOR_UNSUPPORTED;
}

static int cv610_infer(void *context,
                       const hand_frame *frame,
                       hand_detection *detections,
                       size_t detection_capacity,
                       size_t *detection_count,
                       char *error_message,
                       size_t error_message_size)
{
    (void)context;
    (void)frame;
    (void)detections;
    (void)detection_capacity;
    if (detection_count != NULL) {
        *detection_count = 0U;
    }
    cv610_message(error_message, error_message_size, "CV610 backend is not implemented");
    return HAND_DETECTOR_UNSUPPORTED;
}

static void cv610_destroy(void *context)
{
    free(context);
}

static const hand_detector_ops g_cv610_ops = {
    .name = "cv610-placeholder",
    .open = cv610_open,
    .infer = cv610_infer,
    .destroy = cv610_destroy
};

hand_detector *hand_detector_create_cv610(void)
{
    detector_cv610_context *context =
        (detector_cv610_context *)calloc(1U, sizeof(*context));

    if (context == NULL) {
        return NULL;
    }

    return hand_detector_wrap(&g_cv610_ops, context);
}
