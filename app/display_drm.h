#ifndef HAND_DIRECTION_DISPLAY_DRM_H
#define HAND_DIRECTION_DISPLAY_DRM_H

#include "hand_direction/types.h"

#include <stddef.h>
#include <stdint.h>

typedef struct hand_display hand_display;

typedef struct {
    int state;
    int center_valid;
    hand_point center;
    int tracked_box_valid;
    hand_detection tracked_box;
    int show_raw_detections;
    int roi_valid;
    int roi_inside;
    float roi_x_min;
    float roi_x_max;
    float roi_y_min;
    float roi_y_max;
    uint64_t cooldown_remaining_ms;
    uint64_t cooldown_total_ms;
    int event_valid;
    hand_gesture_event event;
    uint64_t event_animation_ms;
} hand_display_overlay;

typedef struct {
    double render_cpu_ms;
    double drm_commit_ms;
} hand_display_timing;


/* Return the source-image rectangle that is visible on the portrait LCD.
 * Display rendering and gesture ROI calculation must use this same crop. */
int hand_display_visible_rect(uint32_t source_width,
                              uint32_t source_height,
                              int *crop_x,
                              int *crop_y,
                              int *crop_width,
                              int *crop_height);

int hand_display_open(hand_display **display,
                      char *error,
                      size_t error_size);

int hand_display_render(hand_display *display,
                        const uint8_t *nv21,
                        uint32_t width,
                        uint32_t height,
                        const hand_detection *detections,
                        size_t detection_count,
                        const hand_display_overlay *overlay,
                        char *error,
                        size_t error_size);

void hand_display_get_last_timing(const hand_display *display,
                                  hand_display_timing *timing);

void hand_display_close(hand_display *display);

#endif
