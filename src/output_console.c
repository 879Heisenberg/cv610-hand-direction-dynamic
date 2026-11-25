#include "hand_direction/output.h"

#include "hand_direction/direction.h"

void hand_gesture_output_console(FILE *stream, const hand_gesture_event *event)
{
    if (stream == NULL || event == NULL || !event->valid) {
        return;
    }

    (void)fprintf(stream,
                  "gesture=%s confidence=%.2f center=(%.3f, %.3f) timestamp=%llu ms\n",
                  hand_direction_name(event->direction),
                  (double)event->confidence,
                  (double)event->x,
                  (double)event->y,
                  (unsigned long long)event->timestamp_ms);
}
