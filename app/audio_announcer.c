#include "audio_announcer.h"

#include "sample_comm.h"
#include "ss_mpi_audio.h"

#ifndef HAND_GESTURE_SOUND_PLAYER
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#endif
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define AUDIO_SAMPLE_RATE 16000U
#define AUDIO_CHANNELS 1U
#define AUDIO_BITS_PER_SAMPLE 16U
#define AUDIO_POINT_NUM_PER_FRAME 160U
#define AUDIO_FRAME_COUNT 30U
#define AUDIO_SEND_TIMEOUT_MS 1000U
#define AUDIO_TAIL_DRAIN_MS 350U
#define AUDIO_VOLUME_GAIN 6
#define AUDIO_MAX_PCM_BYTES (10U * AUDIO_SAMPLE_RATE * sizeof(int16_t))
#define AUDIO_PLAYER_TIMEOUT_MS 5000U

struct hand_audio_announcer {
#ifndef HAND_GESTURE_SOUND_PLAYER
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    int thread_started;
    int stop_requested;
    int pending;
    hand_direction_code pending_direction;
    int disabled;
    char *wav_path;
    char *dir_wav_paths[5];
    char *player_path;
    pid_t child_pid;
#else
    pthread_mutex_t mutex;
    int stop_requested;
    uint8_t *pcm;
    size_t pcm_size;
#endif
};

#ifdef HAND_GESTURE_SOUND_PLAYER
static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static int read_entire_file(const char *path, uint8_t **data, size_t *size)
{
    FILE *file;
    long length;
    uint8_t *buffer;
    size_t read_size;

    if (path == NULL || data == NULL || size == NULL) {
        return -1;
    }
    file = fopen(path, "rb");
    if (file == NULL) {
        (void)fprintf(stderr, "[audio] cannot open %s\n", path);
        return -1;
    }
    if (fseek(file, 0L, SEEK_END) != 0 ||
        (length = ftell(file)) <= 0L ||
        fseek(file, 0L, SEEK_SET) != 0) {
        (void)fprintf(stderr, "[audio] cannot determine WAV size: %s\n", path);
        (void)fclose(file);
        return -1;
    }
    if ((unsigned long)length > AUDIO_MAX_PCM_BYTES + 4096UL) {
        (void)fprintf(stderr, "[audio] WAV is too large: %s\n", path);
        (void)fclose(file);
        return -1;
    }
    buffer = (uint8_t *)malloc((size_t)length);
    if (buffer == NULL) {
        (void)fprintf(stderr, "[audio] out of memory reading %s\n", path);
        (void)fclose(file);
        return -1;
    }
    read_size = fread(buffer, 1U, (size_t)length, file);
    (void)fclose(file);
    if (read_size != (size_t)length) {
        (void)fprintf(stderr, "[audio] short read from %s\n", path);
        free(buffer);
        return -1;
    }
    *data = buffer;
    *size = read_size;
    return 0;
}

static int parse_pcm_wav(const char *path, uint8_t **pcm, size_t *pcm_size)
{
    uint8_t *file_data = NULL;
    size_t file_size = 0U;
    size_t position = 12U;
    int found_format = 0;
    int found_data = 0;

    if (pcm == NULL || pcm_size == NULL) {
        return -1;
    }
    *pcm = NULL;
    *pcm_size = 0U;
    if (read_entire_file(path, &file_data, &file_size) != 0) {
        return -1;
    }
    if (file_size < 12U ||
        memcmp(file_data, "RIFF", 4U) != 0 ||
        memcmp(file_data + 8U, "WAVE", 4U) != 0) {
        (void)fprintf(stderr, "[audio] invalid RIFF/WAVE file: %s\n", path);
        goto fail;
    }

    while (position + 8U <= file_size) {
        const uint8_t *chunk = file_data + position;
        uint32_t chunk_size = read_le32(chunk + 4U);
        size_t payload = position + 8U;
        size_t next;

        if ((size_t)chunk_size > file_size - payload) {
            (void)fprintf(stderr, "[audio] truncated WAV chunk in %s\n", path);
            goto fail;
        }
        if (memcmp(chunk, "fmt ", 4U) == 0) {
            uint16_t audio_format;
            uint16_t channels;
            uint32_t sample_rate;
            uint16_t bits_per_sample;

            if (chunk_size < 16U) {
                (void)fprintf(stderr, "[audio] invalid fmt chunk in %s\n", path);
                goto fail;
            }
            audio_format = read_le16(file_data + payload);
            channels = read_le16(file_data + payload + 2U);
            sample_rate = read_le32(file_data + payload + 4U);
            bits_per_sample = read_le16(file_data + payload + 14U);
            if (audio_format != 1U ||
                channels != AUDIO_CHANNELS ||
                sample_rate != AUDIO_SAMPLE_RATE ||
                bits_per_sample != AUDIO_BITS_PER_SAMPLE) {
                (void)fprintf(stderr,
                              "[audio] unsupported WAV: format=%u channels=%u rate=%lu bits=%u\n",
                              (unsigned)audio_format,
                              (unsigned)channels,
                              (unsigned long)sample_rate,
                              (unsigned)bits_per_sample);
                goto fail;
            }
            found_format = 1;
        } else if (memcmp(chunk, "data", 4U) == 0 && !found_data) {
            if (chunk_size == 0U ||
                (chunk_size & 1U) != 0U ||
                (size_t)chunk_size > AUDIO_MAX_PCM_BYTES) {
                (void)fprintf(stderr, "[audio] invalid PCM data size in %s\n", path);
                goto fail;
            }
            *pcm = (uint8_t *)malloc((size_t)chunk_size);
            if (*pcm == NULL) {
                (void)fprintf(stderr, "[audio] out of memory copying PCM data\n");
                goto fail;
            }
            (void)memcpy(*pcm, file_data + payload, (size_t)chunk_size);
            *pcm_size = (size_t)chunk_size;
            found_data = 1;
        }

        next = payload + (size_t)chunk_size + ((size_t)chunk_size & 1U);
        if (next <= position) {
            break;
        }
        position = next;
    }

    free(file_data);
    if (!found_format || !found_data) {
        (void)fprintf(stderr, "[audio] WAV has no usable PCM format/data: %s\n", path);
        free(*pcm);
        *pcm = NULL;
        *pcm_size = 0U;
        return -1;
    }
    return 0;

fail:
    free(file_data);
    free(*pcm);
    *pcm = NULL;
    *pcm_size = 0U;
    return -1;
}

static int stop_is_requested(hand_audio_announcer *announcer)
{
    int stop;

    (void)pthread_mutex_lock(&announcer->mutex);
    stop = announcer->stop_requested;
    (void)pthread_mutex_unlock(&announcer->mutex);
    return stop;
}

static void interruptible_sleep(hand_audio_announcer *announcer, unsigned milliseconds)
{
    unsigned remaining = milliseconds;

    while (remaining > 0U && !stop_is_requested(announcer)) {
        unsigned slice = remaining > 20U ? 20U : remaining;

        (void)usleep(slice * 1000U);
        remaining -= slice;
    }
}

static int play_pcm(hand_audio_announcer *announcer)
{
    ot_aio_attr attr;
    ot_audio_frame frame;
    td_s16 samples[AUDIO_POINT_NUM_PER_FRAME];
    size_t sample_count = announcer->pcm_size / sizeof(td_s16);
    size_t consumed = 0U;
    int audio_initialized = 0;
    int ao_started = 0;
    int result = -1;

    if (ss_mpi_audio_init() != TD_SUCCESS) {
        (void)fprintf(stderr, "[audio] ss_mpi_audio_init failed\n");
        return -1;
    }
    audio_initialized = 1;

    (void)memset(&attr, 0, sizeof(attr));
    attr.sample_rate = OT_AUDIO_SAMPLE_RATE_16000;
    attr.bit_width = OT_AUDIO_BIT_WIDTH_16;
    attr.work_mode = OT_AIO_MODE_I2S_MASTER;
    attr.snd_mode = OT_AUDIO_SOUND_MODE_MONO;
    attr.expand_flag = 0U;
    attr.frame_num = AUDIO_FRAME_COUNT;
    attr.point_num_per_frame = AUDIO_POINT_NUM_PER_FRAME;
    attr.chn_cnt = AUDIO_CHANNELS;
    attr.clk_share = 1U;
    attr.i2s_type = OT_AIO_I2STYPE_INNERCODEC;

    if (sample_comm_audio_start_ao(0,
                                   AUDIO_CHANNELS,
                                   &attr,
                                   OT_AUDIO_SAMPLE_RATE_BUTT,
                                   TD_FALSE) != TD_SUCCESS) {
        (void)fprintf(stderr, "[audio] AO start failed\n");
        goto cleanup;
    }
    ao_started = 1;

    if (sample_comm_audio_cfg_acodec(&attr) != TD_SUCCESS) {
        (void)fprintf(stderr, "[audio] inner codec configuration failed\n");
        goto cleanup;
    }

    {
        td_s32 volume = 0;

        if (ss_mpi_ao_get_volume(0, &volume) == TD_SUCCESS) {
            volume += AUDIO_VOLUME_GAIN;
            if (ss_mpi_ao_set_volume(0, volume) != TD_SUCCESS) {
                (void)fprintf(stderr, "[audio] cannot apply AO volume gain\n");
            }
        }
    }

    while (consumed < sample_count && !stop_is_requested(announcer)) {
        size_t count = sample_count - consumed;
        size_t index;

        if (count > AUDIO_POINT_NUM_PER_FRAME) {
            count = AUDIO_POINT_NUM_PER_FRAME;
        }
        for (index = 0U; index < count; ++index) {
            size_t sample_index = consumed + index;
            samples[index] = (td_s16)read_le16(announcer->pcm + sample_index * 2U);
        }
        for (; index < AUDIO_POINT_NUM_PER_FRAME; ++index) {
            samples[index] = 0;
        }

        (void)memset(&frame, 0, sizeof(frame));
        frame.bit_width = OT_AUDIO_BIT_WIDTH_16;
        frame.snd_mode = OT_AUDIO_SOUND_MODE_MONO;
        frame.virt_addr[0] = (td_u8 *)samples;
        frame.len = AUDIO_POINT_NUM_PER_FRAME * (td_u32)sizeof(td_s16);
        if (ss_mpi_ao_send_frame(0, 0, &frame, AUDIO_SEND_TIMEOUT_MS) != TD_SUCCESS) {
            (void)fprintf(stderr, "[audio] AO send failed\n");
            goto cleanup;
        }
        consumed += count;
    }

    if (!stop_is_requested(announcer)) {
        interruptible_sleep(announcer, AUDIO_TAIL_DRAIN_MS);
    }
    result = 0;

cleanup:
    if (ao_started) {
        (void)sample_comm_audio_stop_ao(0, AUDIO_CHANNELS, TD_FALSE);
    }
    if (audio_initialized) {
        (void)ss_mpi_audio_exit();
    }
    return result;
}

static void player_alarm_handler(int signal_number)
{
    (void)signal_number;
    _exit(124);
}

int main(int argc, char **argv)
{
    hand_audio_announcer player;
    int result;

    if (argc != 2) {
        (void)fprintf(stderr, "usage: %s <gesture.wav>\n", argv[0]);
        return 2;
    }
    (void)setvbuf(stderr, NULL, _IONBF, 0U);
    (void)signal(SIGALRM, player_alarm_handler);
    (void)alarm(8U);
    (void)memset(&player, 0, sizeof(player));
    if (pthread_mutex_init(&player.mutex, NULL) != 0) {
        return 3;
    }
    if (parse_pcm_wav(argv[1], &player.pcm, &player.pcm_size) != 0) {
        (void)pthread_mutex_destroy(&player.mutex);
        return 4;
    }
    result = play_pcm(&player);
    free(player.pcm);
    (void)pthread_mutex_destroy(&player.mutex);
    return result == 0 ? 0 : 5;
}

#else

static int stop_is_requested(hand_audio_announcer *announcer)
{
    int stop;

    (void)pthread_mutex_lock(&announcer->mutex);
    stop = announcer->stop_requested;
    (void)pthread_mutex_unlock(&announcer->mutex);
    return stop;
}

static int play_pcm(hand_audio_announcer *announcer, const char *wav_path)
{
    posix_spawn_file_actions_t actions;
    char *argv[3];
    pid_t child;
    unsigned elapsed_ms = 0U;
    int spawn_result;
    int actions_ready = 0;
    int result = -1;

    if (announcer == NULL || announcer->player_path == NULL || wav_path == NULL) {
        return -1;
    }
    argv[0] = announcer->player_path;
    argv[1] = (char *)wav_path;
    argv[2] = NULL;
    spawn_result = posix_spawn_file_actions_init(&actions);
    if (spawn_result == 0) {
        actions_ready = 1;
        spawn_result = posix_spawn_file_actions_addopen(
            &actions, STDOUT_FILENO, "/tmp/gesture_audio.log",
            O_WRONLY | O_CREAT | O_APPEND, 0600);
        if (spawn_result == 0) {
            spawn_result = posix_spawn_file_actions_adddup2(
                &actions, STDOUT_FILENO, STDERR_FILENO);
        }
    }
    if (spawn_result != 0) {
        (void)fprintf(stderr, "[audio] cannot prepare player launch: %s\n",
                      strerror(spawn_result));
        if (actions_ready) {
            (void)posix_spawn_file_actions_destroy(&actions);
        }
        return -1;
    }
    spawn_result = posix_spawn(&child, announcer->player_path, &actions, NULL,
                               argv, environ);
    (void)posix_spawn_file_actions_destroy(&actions);
    if (spawn_result != 0) {
        (void)fprintf(stderr, "[audio] cannot launch %s: %s\n",
                      announcer->player_path, strerror(spawn_result));
        return -1;
    }

    (void)pthread_mutex_lock(&announcer->mutex);
    announcer->child_pid = child;
    (void)pthread_mutex_unlock(&announcer->mutex);

    while (elapsed_ms < AUDIO_PLAYER_TIMEOUT_MS) {
        int status = 0;
        pid_t waited = waitpid(child, &status, WNOHANG);

        if (waited == child) {
            result = WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
            break;
        }
        if (waited < 0 && errno != EINTR) {
            break;
        }
        if (stop_is_requested(announcer)) {
            break;
        }
        (void)usleep(20U * 1000U);
        elapsed_ms += 20U;
    }

    if (result != 0) {
        int status = 0;

        (void)kill(child, SIGKILL);
        while (waitpid(child, &status, 0) < 0 && errno == EINTR) {
        }
        if (elapsed_ms >= AUDIO_PLAYER_TIMEOUT_MS) {
            (void)fprintf(stderr, "[audio] player timed out and was killed\n");
        }
    }
    (void)pthread_mutex_lock(&announcer->mutex);
    announcer->child_pid = -1;
    (void)pthread_mutex_unlock(&announcer->mutex);
    return result;
}

static const char *direction_wav_filename(hand_direction_code direction)
{
    switch (direction) {
    case HAND_DIRECTION_LEFT:
        return "gesture_left.wav";
    case HAND_DIRECTION_RIGHT:
        return "gesture_right.wav";
    case HAND_DIRECTION_UP:
        return "gesture_up.wav";
    case HAND_DIRECTION_DOWN:
        return "gesture_down.wav";
    case HAND_DIRECTION_NONE:
    default:
        return NULL;
    }
}

static void resolve_direction_wav_paths(const char *default_wav_path,
                                        char *dir_wav_paths[5])
{
    const char *slash;
    size_t dir_len;
    int index;

    if (default_wav_path == NULL || dir_wav_paths == NULL) {
        return;
    }
    slash = strrchr(default_wav_path, '/');
    dir_len = slash != NULL ? (size_t)(slash - default_wav_path) + 1U : 0U;

    for (index = 0; index < 5; ++index) {
        const char *filename = direction_wav_filename((hand_direction_code)index);
        char candidate[512];

        dir_wav_paths[index] = NULL;
        if (filename == NULL) {
            continue;
        }
        if (dir_len + strlen(filename) + 1U > sizeof(candidate)) {
            continue;
        }
        if (dir_len > 0U) {
            (void)snprintf(candidate, sizeof(candidate), "%.*s%s",
                           (int)dir_len, default_wav_path, filename);
        } else {
            (void)snprintf(candidate, sizeof(candidate), "%s", filename);
        }
        if (access(candidate, R_OK) == 0) {
            dir_wav_paths[index] = strdup(candidate);
        }
    }
}

static void free_announcer_paths(hand_audio_announcer *announcer)
{
    size_t index;

    if (announcer == NULL) {
        return;
    }
    free(announcer->wav_path);
    announcer->wav_path = NULL;
    free(announcer->player_path);
    announcer->player_path = NULL;
    for (index = 0U; index < sizeof(announcer->dir_wav_paths) / sizeof(announcer->dir_wav_paths[0]); ++index) {
        free(announcer->dir_wav_paths[index]);
        announcer->dir_wav_paths[index] = NULL;
    }
}

static void *audio_worker_main(void *argument)
{
    hand_audio_announcer *announcer = (hand_audio_announcer *)argument;

    for (;;) {
        int should_play;
        const char *selected_wav = NULL;
        int direction_index;

        (void)pthread_mutex_lock(&announcer->mutex);
        while (!announcer->stop_requested &&
               !announcer->pending &&
               !announcer->disabled) {
            (void)pthread_cond_wait(&announcer->condition, &announcer->mutex);
        }
        if (announcer->stop_requested || announcer->disabled) {
            (void)pthread_mutex_unlock(&announcer->mutex);
            break;
        }
        announcer->pending = 0;
        direction_index = (int)announcer->pending_direction;
        announcer->pending_direction = HAND_DIRECTION_NONE;
        selected_wav = announcer->wav_path;
        if (direction_index >= (int)HAND_DIRECTION_LEFT &&
            direction_index <= (int)HAND_DIRECTION_DOWN &&
            announcer->dir_wav_paths[direction_index] != NULL) {
            selected_wav = announcer->dir_wav_paths[direction_index];
        }
        should_play = 1;
        (void)pthread_mutex_unlock(&announcer->mutex);

        if (should_play && play_pcm(announcer, selected_wav) != 0) {
            (void)pthread_mutex_lock(&announcer->mutex);
            announcer->disabled = 1;
            announcer->pending = 0;
            (void)pthread_mutex_unlock(&announcer->mutex);
            (void)fprintf(stderr, "[audio] playback disabled after an audio error\n");
        }
    }
    return NULL;
}

hand_audio_announcer *hand_audio_announcer_create(const char *wav_path)
{
    hand_audio_announcer *announcer;
    const char *disabled;
    const char *player_path;

    disabled = getenv("HAND_GESTURE_SOUND");
    if (disabled != NULL &&
        (strcmp(disabled, "off") == 0 ||
         strcmp(disabled, "0") == 0 ||
         strcmp(disabled, "none") == 0)) {
        return NULL;
    }
    player_path = getenv("HAND_GESTURE_SOUND_PLAYER");
    if (player_path == NULL || player_path[0] == '\0') {
        player_path = "/root/hand_direction/bin/cv610_gesture_sound";
    }
    if (wav_path == NULL || access(wav_path, R_OK) != 0) {
        (void)fprintf(stderr, "[audio] WAV is not readable: %s\n",
                      wav_path != NULL ? wav_path : "(null)");
        return NULL;
    }
    if (access(player_path, X_OK) != 0) {
        (void)fprintf(stderr, "[audio] player is not executable: %s\n", player_path);
        return NULL;
    }
    announcer = (hand_audio_announcer *)calloc(1U, sizeof(*announcer));
    if (announcer == NULL) {
        return NULL;
    }
    announcer->wav_path = strdup(wav_path);
    announcer->player_path = strdup(player_path);
    announcer->child_pid = -1;
    if (announcer->wav_path == NULL || announcer->player_path == NULL) {
        free_announcer_paths(announcer);
        free(announcer);
        return NULL;
    }
    resolve_direction_wav_paths(wav_path, announcer->dir_wav_paths);
    if (pthread_mutex_init(&announcer->mutex, NULL) != 0) {
        free_announcer_paths(announcer);
        free(announcer);
        return NULL;
    }
    if (pthread_cond_init(&announcer->condition, NULL) != 0) {
        (void)pthread_mutex_destroy(&announcer->mutex);
        free_announcer_paths(announcer);
        free(announcer);
        return NULL;
    }
    if (pthread_create(&announcer->thread, NULL, audio_worker_main, announcer) != 0) {
        (void)fprintf(stderr, "[audio] cannot create playback thread\n");
        (void)pthread_cond_destroy(&announcer->condition);
        (void)pthread_mutex_destroy(&announcer->mutex);
        free_announcer_paths(announcer);
        free(announcer);
        return NULL;
    }
    announcer->thread_started = 1;
    return announcer;
}

void hand_audio_announcer_post_direction(hand_audio_announcer *announcer,
                                         hand_direction_code direction)
{
    if (announcer == NULL) {
        return;
    }
    (void)pthread_mutex_lock(&announcer->mutex);
    if (!announcer->stop_requested && !announcer->disabled) {
        announcer->pending_direction = direction;
        announcer->pending = 1;
        (void)pthread_cond_signal(&announcer->condition);
    }
    (void)pthread_mutex_unlock(&announcer->mutex);
}

void hand_audio_announcer_post(hand_audio_announcer *announcer)
{
    hand_audio_announcer_post_direction(announcer, HAND_DIRECTION_NONE);
}

void hand_audio_announcer_destroy(hand_audio_announcer *announcer)
{
    if (announcer == NULL) {
        return;
    }
    if (announcer->thread_started) {
        (void)pthread_mutex_lock(&announcer->mutex);
        announcer->stop_requested = 1;
        (void)pthread_cond_broadcast(&announcer->condition);
        (void)pthread_mutex_unlock(&announcer->mutex);
        (void)pthread_join(announcer->thread, NULL);
    }
    (void)pthread_cond_destroy(&announcer->condition);
    (void)pthread_mutex_destroy(&announcer->mutex);
    free_announcer_paths(announcer);
    free(announcer);
}

#endif
