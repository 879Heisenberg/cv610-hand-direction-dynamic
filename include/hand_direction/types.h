#ifndef HAND_DIRECTION_TYPES_H
#define HAND_DIRECTION_TYPES_H

#include <stddef.h>
#include <stdint.h>

#define HAND_DETECTION_MAX_COUNT 8U

typedef enum {
    HAND_PIXEL_FORMAT_UNKNOWN = 0,
    HAND_PIXEL_FORMAT_NV21,
    HAND_PIXEL_FORMAT_RGB888,
    HAND_PIXEL_FORMAT_BGR888,
    HAND_PIXEL_FORMAT_GRAY8
} hand_pixel_format;

typedef struct {
    const uint8_t *data;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    hand_pixel_format format;
    uint64_t timestamp_ms;
} hand_frame;

typedef struct {
    float x0;
    float y0;
    float x1;
    float y1;
    float confidence;
    int class_id;
} hand_detection;

typedef struct {
    float x;
    float y;
    uint64_t timestamp_ms;
} hand_point;

typedef enum {
    HAND_DIRECTION_NONE = 0,
    HAND_DIRECTION_LEFT,
    HAND_DIRECTION_RIGHT,
    HAND_DIRECTION_UP,
    HAND_DIRECTION_DOWN
} hand_direction_code;

typedef struct {
    int valid;
    hand_direction_code direction;
    float confidence;
    float x;
    float y;
    uint64_t timestamp_ms;
} hand_gesture_event;

#endif
