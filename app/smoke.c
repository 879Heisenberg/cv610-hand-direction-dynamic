#include "hand_direction/detector.h"
#include "hand_direction/types.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

static void usage(const char *program)
{
    fprintf(stderr,
            "usage: %s <model.om> <frame.raw> <width> <height> <stride> "
            "<nv21|rgb|bgr|gray>\n",
            program);
}

static int parse_u32(const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if (text == NULL || value == NULL || text[0] == '\0') {
        return -1;
    }
    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) {
        return -1;
    }
    *value = (uint32_t)parsed;
    return 0;
}

static int parse_format(const char *text, hand_pixel_format *format)
{
    if (strcmp(text, "nv21") == 0) {
        *format = HAND_PIXEL_FORMAT_NV21;
    } else if (strcmp(text, "rgb") == 0) {
        *format = HAND_PIXEL_FORMAT_RGB888;
    } else if (strcmp(text, "bgr") == 0) {
        *format = HAND_PIXEL_FORMAT_BGR888;
    } else if (strcmp(text, "gray") == 0) {
        *format = HAND_PIXEL_FORMAT_GRAY8;
    } else {
        return -1;
    }
    return 0;
}

static int frame_bytes_for(uint32_t width,
                           uint32_t height,
                           uint32_t stride,
                           hand_pixel_format format,
                           size_t *frame_bytes)
{
    size_t rows;

    if (width == 0U || height == 0U || stride == 0U || frame_bytes == NULL) {
        return -1;
    }
    switch (format) {
    case HAND_PIXEL_FORMAT_NV21:
        if ((width & 1U) != 0U || (height & 1U) != 0U || stride < width) {
            return -1;
        }
        rows = (size_t)height + (size_t)height / 2U;
        break;
    case HAND_PIXEL_FORMAT_RGB888:
    case HAND_PIXEL_FORMAT_BGR888:
        if ((size_t)stride < (size_t)width * 3U) {
            return -1;
        }
        rows = height;
        break;
    case HAND_PIXEL_FORMAT_GRAY8:
        if (stride < width) {
            return -1;
        }
        rows = height;
        break;
    default:
        return -1;
    }
    if (rows > SIZE_MAX / (size_t)stride) {
        return -1;
    }
    *frame_bytes = rows * (size_t)stride;
    return 0;
}

static int read_file(const char *path, uint8_t **data, size_t *size)
{
    struct stat info;
    FILE *file;
    uint8_t *buffer;
    size_t bytes_read;

    if (stat(path, &info) != 0 || !S_ISREG(info.st_mode) ||
        info.st_size <= 0 || (uint64_t)info.st_size > SIZE_MAX) {
        fprintf(stderr, "invalid frame file: %s\n", path);
        return -1;
    }
    *size = (size_t)info.st_size;
    buffer = (uint8_t *)malloc(*size);
    if (buffer == NULL) {
        fprintf(stderr, "cannot allocate %zu bytes for frame\n", *size);
        return -1;
    }
    file = fopen(path, "rb");
    if (file == NULL) {
        fprintf(stderr, "cannot open frame file: %s\n", path);
        free(buffer);
        return -1;
    }
    bytes_read = fread(buffer, 1U, *size, file);
    if (fclose(file) != 0 || bytes_read != *size) {
        fprintf(stderr, "cannot read the complete frame file: %s\n", path);
        free(buffer);
        return -1;
    }
    *data = buffer;
    return 0;
}

static uint64_t now_ns(void)
{
    struct timespec value;

    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0U;
    }
    return (uint64_t)value.tv_sec * 1000000000ULL + (uint64_t)value.tv_nsec;
}

int main(int argc, char **argv)
{
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    hand_pixel_format format;
    size_t expected_bytes;
    size_t frame_bytes;
    uint8_t *frame_data = NULL;
    hand_frame frame;
    hand_detection detections[HAND_DETECTION_MAX_COUNT];
    size_t detection_count = 0U;
    hand_detector *detector = NULL;
    char error[256] = {0};
    uint64_t start;
    uint64_t elapsed;
    int status;
    size_t index;
    int exit_code = 1;

    if (argc != 7) {
        usage(argv[0]);
        return 2;
    }
    if (parse_u32(argv[3], &width) != 0 || parse_u32(argv[4], &height) != 0 ||
        parse_u32(argv[5], &stride) != 0 || parse_format(argv[6], &format) != 0 ||
        frame_bytes_for(width, height, stride, format, &expected_bytes) != 0) {
        fprintf(stderr, "invalid width, height, stride or pixel format\n");
        return 2;
    }
    if (read_file(argv[2], &frame_data, &frame_bytes) != 0) {
        return 1;
    }
    if (frame_bytes != expected_bytes) {
        fprintf(stderr,
                "frame size mismatch: file=%zu expected=%zu for %ux%u stride=%u\n",
                frame_bytes,
                expected_bytes,
                (unsigned int)width,
                (unsigned int)height,
                (unsigned int)stride);
        goto cleanup;
    }

    detector = hand_detector_create_cv610();
    if (detector == NULL) {
        fprintf(stderr, "cannot create CV610 detector\n");
        goto cleanup;
    }
    status = hand_detector_open(detector, argv[1], error, sizeof(error));
    if (status != HAND_DETECTOR_OK) {
        fprintf(stderr, "open failed (%d): %s\n", status, error);
        goto cleanup;
    }

    frame.data = frame_data;
    frame.width = width;
    frame.height = height;
    frame.stride = stride;
    frame.format = format;
    frame.timestamp_ms = 0U;
    start = now_ns();
    status = hand_detector_infer(detector,
                                 &frame,
                                 detections,
                                 HAND_DETECTION_MAX_COUNT,
                                 &detection_count,
                                 error,
                                 sizeof(error));
    elapsed = start != 0U ? now_ns() - start : 0U;
    if (status != HAND_DETECTOR_OK) {
        fprintf(stderr, "inference failed (%d): %s\n", status, error);
        goto cleanup;
    }

    printf("backend=%s detections=%zu elapsed_ms=%.3f\n",
           hand_detector_name(detector),
           detection_count,
           (double)elapsed / 1000000.0);
    for (index = 0U; index < detection_count; ++index) {
        const hand_detection *detection = &detections[index];

        printf("  [%zu] class=%d score=%.6f normalized=(%.6f,%.6f)-(%.6f,%.6f) "
               "pixels=(%.2f,%.2f)-(%.2f,%.2f)\n",
               index,
               detection->class_id,
               (double)detection->confidence,
               (double)detection->x0,
               (double)detection->y0,
               (double)detection->x1,
               (double)detection->y1,
               (double)detection->x0 * (double)width,
               (double)detection->y0 * (double)height,
               (double)detection->x1 * (double)width,
               (double)detection->y1 * (double)height);
    }
    exit_code = 0;

cleanup:
    hand_detector_destroy(detector);
    free(frame_data);
    return exit_code;
}
