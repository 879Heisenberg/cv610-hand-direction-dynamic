#ifndef HAND_DIRECTION_AUDIO_ANNOUNCER_H
#define HAND_DIRECTION_AUDIO_ANNOUNCER_H

#include "hand_direction/types.h"

typedef struct hand_audio_announcer hand_audio_announcer;

/* Starts a lightweight audio trigger worker. Playback runs in a separate
 * process so Codec/AO failures can never block gesture detection. */
hand_audio_announcer *hand_audio_announcer_create(const char *wav_path);

/* Requests one direction-specific playback (or default WAV if unavailable).
 * Calls made while a sound is playing are collapsed into one pending request,
 * so repeated gesture events never stack audio. */
void hand_audio_announcer_post_direction(hand_audio_announcer *announcer,
                                         hand_direction_code direction);

void hand_audio_announcer_post(hand_audio_announcer *announcer);

void hand_audio_announcer_destroy(hand_audio_announcer *announcer);

#endif
