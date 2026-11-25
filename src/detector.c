#include "hand_direction/detector.h"

#include "detector_internal.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct hand_detector {
    const hand_detector_ops *ops;
    void *context;
};

static void detector_message(char *buffer, size_t size, const char *text)
{
    if (buffer != NULL && size > 0U) {
        (void)snprintf(buffer, size, "%s", text != NULL ? text : "");
    }
}

hand_detector *hand_detector_wrap(const hand_detector_ops *ops, void *context)
{
    hand_detector *detector;

    if (ops == NULL || ops->open == NULL || ops->infer == NULL) {
        return NULL;
    }

    detector = (hand_detector *)calloc(1U, sizeof(*detector));
    if (detector == NULL) {
        return NULL;
    }

    detector->ops = ops;
    detector->context = context;
    return detector;
}

const char *hand_detector_name(const hand_detector *detector)
{
    if (detector == NULL || detector->ops == NULL || detector->ops->name == NULL) {
        return "unknown";
    }
    return detector->ops->name;
}

int hand_detector_open(hand_detector *detector,
                       const char *model_path,
                       char *error_message,
                       size_t error_message_size)
{
    if (detector == NULL || detector->ops == NULL || detector->ops->open == NULL) {
        detector_message(error_message, error_message_size, "detector is not initialized");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }

    return detector->ops->open(detector->context,
                               model_path,
                               error_message,
                               error_message_size);
}

int hand_detector_infer(hand_detector *detector,
                        const hand_frame *frame,
                        hand_detection *detections,
                        size_t detection_capacity,
                        size_t *detection_count,
                        char *error_message,
                        size_t error_message_size)
{
    if (detector == NULL || detector->ops == NULL || detector->ops->infer == NULL) {
        detector_message(error_message, error_message_size, "detector is not initialized");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }

    if (frame == NULL || detections == NULL || detection_count == NULL ||
        detection_capacity == 0U) {
        detector_message(error_message, error_message_size, "invalid inference buffers");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }

    return detector->ops->infer(detector->context,
                                frame,
                                detections,
                                detection_capacity,
                                detection_count,
                                error_message,
                                error_message_size);
}

int hand_detector_get_last_timing(const hand_detector *detector,
                                  hand_detector_timing *timing)
{
    if (timing == NULL) {
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }
    memset(timing, 0, sizeof(*timing));
    if (detector == NULL || detector->ops == NULL ||
        detector->ops->get_timing == NULL) {
        return HAND_DETECTOR_UNSUPPORTED;
    }
    return detector->ops->get_timing(detector->context, timing);
}

void hand_detector_destroy(hand_detector *detector)
{
    if (detector == NULL) {
        return;
    }

    if (detector->ops != NULL && detector->ops->destroy != NULL) {
        detector->ops->destroy(detector->context);
    }
    free(detector);
}
