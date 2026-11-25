#ifndef HAND_DIRECTION_TRAJECTORY_H
#define HAND_DIRECTION_TRAJECTORY_H

#include "hand_direction/types.h"

#include <stddef.h>
#include <stdint.h>

#define HAND_TRAJECTORY_CAPACITY 32U

typedef struct {
    hand_point points[HAND_TRAJECTORY_CAPACITY];
    size_t count;
} hand_trajectory;

void hand_trajectory_reset(hand_trajectory *trajectory);

int hand_trajectory_push(hand_trajectory *trajectory,
                         hand_point point,
                         uint64_t max_gap_ms,
                         uint64_t window_ms);

size_t hand_trajectory_count(const hand_trajectory *trajectory);
const hand_point *hand_trajectory_at(const hand_trajectory *trajectory, size_t index);

#endif
