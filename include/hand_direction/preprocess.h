#ifndef HAND_DIRECTION_PREPROCESS_H
#define HAND_DIRECTION_PREPROCESS_H

#include "hand_direction/types.h"

#include <stddef.h>
#include <stdint.h>

#define HAND_MODEL_INPUT_WIDTH 416U
#define HAND_MODEL_INPUT_HEIGHT 416U
#define HAND_MODEL_INPUT_CHANNELS 3U
#define HAND_MODEL_INPUT_PADDING 114U

typedef struct {
    uint32_t source_width;
    uint32_t source_height;
    uint32_t resized_width;
    uint32_t resized_height;
    uint32_t pad_left;
    uint32_t pad_top;
    float scale;
} hand_letterbox_transform;

int hand_letterbox_init(uint32_t source_width,
                        uint32_t source_height,
                        hand_letterbox_transform *transform);

int hand_preprocess_fill_padding(float *destination,
                                 size_t destination_bytes,
                                 size_t destination_row_stride,
                                 char *error_message,
                                 size_t error_message_size);

int hand_preprocess_frame_ex(const hand_frame *frame,
                             float *destination,
                             size_t destination_bytes,
                             size_t destination_row_stride,
                             int skip_padding_fill,
                             hand_letterbox_transform *transform,
                             char *error_message,
                             size_t error_message_size);

int hand_preprocess_frame(const hand_frame *frame,
                          float *destination,
                          size_t destination_bytes,
                          size_t destination_row_stride,
                          hand_letterbox_transform *transform,
                          char *error_message,
                          size_t error_message_size);

#endif
