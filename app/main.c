#include "hand_direction/detector.h"
#include "hand_direction/gesture_engine.h"
#include "hand_direction/output.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define DEMO_FRAME_COUNT 200U
#define DEMO_FRAME_PERIOD_MS 100U

int main(void)
{
    hand_detector *detector = hand_detector_create_stub();
    hand_gesture_engine engine;
    char error_message[256] = {0};
    unsigned int event_count = 0U;
    unsigned int frame_index;

    if (detector == NULL) {
        (void)fprintf(stderr, "failed to create detector\n");
        return 1;
    }

    if (hand_detector_open(detector,
                           NULL,
                           error_message,
                           sizeof(error_message)) != HAND_DETECTOR_OK) {
        (void)fprintf(stderr, "detector open failed: %s\n", error_message);
        hand_detector_destroy(detector);
        return 1;
    }

    hand_gesture_engine_init(&engine, NULL);
    (void)printf("backend=%s model=<empty> mode=simulation\n", hand_detector_name(detector));

    for (frame_index = 0U; frame_index < DEMO_FRAME_COUNT; ++frame_index) {
        hand_frame frame;
        hand_detection detections[HAND_DETECTION_MAX_COUNT];
        size_t detection_count = 0U;
        hand_gesture_event event;

        frame.data = NULL;
        frame.width = 640U;
        frame.height = 360U;
        frame.stride = 640U;
        frame.format = HAND_PIXEL_FORMAT_NV21;
        frame.timestamp_ms = (uint64_t)frame_index * DEMO_FRAME_PERIOD_MS;

        if (hand_detector_infer(detector,
                                &frame,
                                detections,
                                HAND_DETECTION_MAX_COUNT,
                                &detection_count,
                                error_message,
                                sizeof(error_message)) != HAND_DETECTOR_OK) {
            (void)fprintf(stderr,
                          "inference failed at %llu ms: %s\n",
                          (unsigned long long)frame.timestamp_ms,
                          error_message);
            break;
        }

        if (hand_gesture_engine_update(&engine,
                                       detections,
                                       detection_count,
                                       frame.timestamp_ms,
                                       &event) != 0) {
            (void)fprintf(stderr, "gesture engine failed\n");
            break;
        }

        if (event.valid) {
            hand_gesture_output_console(stdout, &event);
            ++event_count;
            break;
        }
    }

    (void)printf("events=%u state=%d\n", event_count, (int)engine.state);
    hand_detector_destroy(detector);
    return event_count > 0U ? 0 : 2;
}
