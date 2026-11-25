#ifndef HAND_DIRECTION_DETECTOR_INTERNAL_H
#define HAND_DIRECTION_DETECTOR_INTERNAL_H

#include "hand_direction/detector.h"

typedef struct {
    const char *name;
    int (*open)(void *context,
                const char *model_path,
                char *error_message,
                size_t error_message_size);
    int (*infer)(void *context,
                 const hand_frame *frame,
                 hand_detection *detections,
                 size_t detection_capacity,
                 size_t *detection_count,
                 char *error_message,
                 size_t error_message_size);
    int (*get_timing)(void *context, hand_detector_timing *timing);
    void (*destroy)(void *context);
} hand_detector_ops;

hand_detector *hand_detector_wrap(const hand_detector_ops *ops, void *context);

#endif
