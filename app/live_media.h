#ifndef HAND_DIRECTION_LIVE_MEDIA_H
#define HAND_DIRECTION_LIVE_MEDIA_H

#include <stddef.h>
#include <stdint.h>

typedef struct hand_live_media hand_live_media;

enum {
    HAND_LIVE_MEDIA_WIDTH = 640,
    HAND_LIVE_MEDIA_HEIGHT = 360,
    HAND_LIVE_MEDIA_NV21_BYTES = HAND_LIVE_MEDIA_WIDTH * HAND_LIVE_MEDIA_HEIGHT * 3 / 2
};

int hand_live_media_open(hand_live_media **media,
                         char *error_message,
                         size_t error_message_size);

int hand_live_media_read_nv21(hand_live_media *media,
                              uint8_t *buffer,
                              size_t buffer_size,
                              int timeout_ms,
                              uint32_t *width,
                              uint32_t *height,
                              char *error_message,
                              size_t error_message_size);

void hand_live_media_close(hand_live_media *media);

#endif
