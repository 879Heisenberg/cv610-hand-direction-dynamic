#ifndef HAND_DIRECTION_DETECTOR_H
#define HAND_DIRECTION_DETECTOR_H

#include "hand_direction/types.h"

#include <stddef.h>

typedef struct hand_detector hand_detector;

typedef enum {
    HAND_DETECTOR_OK = 0,
    HAND_DETECTOR_INVALID_ARGUMENT,
    HAND_DETECTOR_IO_ERROR,
    HAND_DETECTOR_UNSUPPORTED,
    HAND_DETECTOR_RUNTIME_ERROR
} hand_detector_status;

typedef struct {
    double set_context_ms;
    double preprocess_ms;
    double flush_ms;
    double npu_execute_ms;
    double invalidate_ms;
    double postprocess_ms;
    double total_ms;
} hand_detector_timing;

hand_detector *hand_detector_create_stub(void);
hand_detector *hand_detector_create_cv610(void);

const char *hand_detector_name(const hand_detector *detector);

int hand_detector_open(hand_detector *detector,
                       const char *model_path,
                       char *error_message,
                       size_t error_message_size);

int hand_detector_infer(hand_detector *detector,
                        const hand_frame *frame,
                        hand_detection *detections,
                        size_t detection_capacity,
                        size_t *detection_count,
                        char *error_message,
                        size_t error_message_size);

int hand_detector_get_last_timing(const hand_detector *detector,
                                  hand_detector_timing *timing);

void hand_detector_destroy(hand_detector *detector);

#endif
