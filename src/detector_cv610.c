#include "hand_direction/detector.h"
#include "hand_direction/postprocess.h"
#include "hand_direction/preprocess.h"

#include "detector_internal.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "svp_acl.h"
#include "svp_acl_ext.h"
#include "svp_acl_mdl.h"

#define HAND_CV610_MODEL_LIMIT (16U * 1024U * 1024U)
#define HAND_CV610_INPUT_RANK 4U
#define HAND_CV610_INPUT_COUNT 3U
#define HAND_CV610_IMAGE_INPUT_INDEX 0U
#define HAND_CV610_TASK_INPUT_INDEX 1U
#define HAND_CV610_WORK_INPUT_INDEX 2U
#define HAND_CV610_OUTPUT_RANK 3U
#define HAND_CV610_OUTPUT_CLASS_ID 0

typedef struct {
    bool initialized;
    bool device_ready;
    bool model_loaded;
    svp_acl_rt_context acl_context;
    uint32_t model_id;
    void *model_data;
    size_t model_bytes;
    svp_acl_mdl_desc *description;
    svp_acl_mdl_dataset *input_dataset;
    svp_acl_mdl_dataset *output_dataset;
    void *input_data[HAND_CV610_INPUT_COUNT];
    size_t input_bytes[HAND_CV610_INPUT_COUNT];
    size_t input_stride[HAND_CV610_INPUT_COUNT];
    svp_acl_data_buffer *input_buffer[HAND_CV610_INPUT_COUNT];
    void *output_data;
    size_t output_bytes;
    size_t output_stride;
    svp_acl_data_buffer *output_buffer;
    size_t candidate_count;
    hand_postprocess_config postprocess_config;
    hand_postprocess_workspace postprocess_workspace;
    size_t score_diagnostic_frames;
    uint32_t padded_source_width;
    uint32_t padded_source_height;
    bool input_padding_ready;
    hand_detector_timing last_timing;
} detector_cv610_context;

static uint64_t cv610_now_ns(void)
{
    struct timespec value;

    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0U;
    }
    return (uint64_t)value.tv_sec * 1000000000ULL + (uint64_t)value.tv_nsec;
}

static double cv610_elapsed_ms(uint64_t start, uint64_t end)
{
    if (start == 0U || end < start) {
        return 0.0;
    }
    return (double)(end - start) / 1000000.0;
}

static void cv610_finish_timing(detector_cv610_context *cv610, uint64_t total_start)
{
    cv610->last_timing.total_ms =
        cv610_elapsed_ms(total_start, cv610_now_ns());
}

static void cv610_message(char *buffer, size_t size, const char *format, ...)
{
    va_list arguments;

    if (buffer == NULL || size == 0U) {
        return;
    }

    va_start(arguments, format);
    (void)vsnprintf(buffer, size, format, arguments);
    va_end(arguments);
}

static int cv610_acl_check(svp_acl_error error,
                           const char *operation,
                           char *error_message,
                           size_t error_message_size)
{
    if (error == SVP_ACL_SUCCESS) {
        return 0;
    }

    cv610_message(error_message,
                  error_message_size,
                  "ACL %s failed: %d",
                  operation,
                  (int)error);
    return -1;
}

static void cv610_release(detector_cv610_context *cv610)
{
    size_t index;

    if (cv610 == NULL) {
        return;
    }

    if (cv610->acl_context != NULL) {
        (void)svp_acl_rt_set_current_context(cv610->acl_context);
    }
    if (cv610->output_buffer != NULL) {
        (void)svp_acl_destroy_data_buffer(cv610->output_buffer);
    }
    for (index = 0U; index < HAND_CV610_INPUT_COUNT; ++index) {
        if (cv610->input_buffer[index] != NULL) {
            (void)svp_acl_destroy_data_buffer(cv610->input_buffer[index]);
        }
    }
    if (cv610->output_data != NULL) {
        (void)svp_acl_rt_free(cv610->output_data);
    }
    for (index = 0U; index < HAND_CV610_INPUT_COUNT; ++index) {
        if (cv610->input_data[index] != NULL) {
            (void)svp_acl_rt_free(cv610->input_data[index]);
        }
    }
    if (cv610->output_dataset != NULL) {
        (void)svp_acl_mdl_destroy_dataset(cv610->output_dataset);
    }
    if (cv610->input_dataset != NULL) {
        (void)svp_acl_mdl_destroy_dataset(cv610->input_dataset);
    }
    if (cv610->description != NULL) {
        (void)svp_acl_mdl_destroy_desc(cv610->description);
    }
    if (cv610->model_loaded) {
        (void)svp_acl_mdl_unload(cv610->model_id);
    }
    if (cv610->model_data != NULL) {
        (void)svp_acl_rt_free(cv610->model_data);
    }
    hand_postprocess_workspace_destroy(&cv610->postprocess_workspace);
    if (cv610->device_ready) {
        (void)svp_acl_rt_reset_device(0);
    }
    if (cv610->initialized) {
        (void)svp_acl_finalize();
    }

    memset(cv610, 0, sizeof(*cv610));
}

static int cv610_read_model(detector_cv610_context *cv610,
                            const char *model_path,
                            char *error_message,
                            size_t error_message_size)
{
    struct stat file_info;
    FILE *file;
    size_t bytes_read;
    int close_status;

    if (model_path == NULL || model_path[0] == '\0' ||
        stat(model_path, &file_info) != 0 || !S_ISREG(file_info.st_mode) ||
        file_info.st_size <= 0 || (uint64_t)file_info.st_size > HAND_CV610_MODEL_LIMIT) {
        cv610_message(error_message,
                      error_message_size,
                      "invalid or oversized OM file: %s",
                      model_path != NULL ? model_path : "<null>");
        return HAND_DETECTOR_IO_ERROR;
    }

    cv610->model_bytes = (size_t)file_info.st_size;
    if (cv610_acl_check(svp_acl_rt_malloc_cached(&cv610->model_data,
                                                  cv610->model_bytes,
                                                  SVP_ACL_MEM_MALLOC_NORMAL_ONLY),
                        "allocate model",
                        error_message,
                        error_message_size) != 0) {
        return HAND_DETECTOR_RUNTIME_ERROR;
    }

    file = fopen(model_path, "rb");
    if (file == NULL) {
        cv610_message(error_message, error_message_size, "cannot open OM file: %s", model_path);
        return HAND_DETECTOR_IO_ERROR;
    }
    bytes_read = fread(cv610->model_data, 1U, cv610->model_bytes, file);
    close_status = fclose(file);
    if (bytes_read != cv610->model_bytes || close_status != 0) {
        cv610_message(error_message, error_message_size, "failed to read complete OM file");
        return HAND_DETECTOR_IO_ERROR;
    }
    if (cv610_acl_check(svp_acl_rt_mem_flush(cv610->model_data, cv610->model_bytes),
                        "flush model",
                        error_message,
                        error_message_size) != 0) {
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    return HAND_DETECTOR_OK;
}

static int cv610_validate_input(detector_cv610_context *cv610,
                                char *error_message,
                                size_t error_message_size)
{
    svp_acl_mdl_io_dims dimensions;
    const char *image_name;
    const char *task_name;
    const char *work_name;
    size_t logical_stride;
    size_t minimum_bytes;
    size_t index;

    if (svp_acl_mdl_get_num_inputs(cv610->description) != HAND_CV610_INPUT_COUNT) {
        cv610_message(error_message,
                      error_message_size,
                      "OM must have images, task_buf and work_buf inputs");
        return -1;
    }

    for (index = 0U; index < HAND_CV610_INPUT_COUNT; ++index) {
        cv610->input_bytes[index] =
            svp_acl_mdl_get_input_size_by_index(cv610->description, index);
        cv610->input_stride[index] =
            svp_acl_mdl_get_input_default_stride(cv610->description, index);
        if (cv610->input_bytes[index] == 0U || cv610->input_stride[index] == 0U) {
            cv610_message(error_message,
                          error_message_size,
                          "input[%lu] has an invalid size or stride",
                          (unsigned long)index);
            return -1;
        }
    }

    image_name = svp_acl_mdl_get_input_name_by_index(cv610->description,
                                                     HAND_CV610_IMAGE_INPUT_INDEX);
    task_name = svp_acl_mdl_get_input_name_by_index(cv610->description,
                                                    HAND_CV610_TASK_INPUT_INDEX);
    work_name = svp_acl_mdl_get_input_name_by_index(cv610->description,
                                                    HAND_CV610_WORK_INPUT_INDEX);
    logical_stride = (size_t)HAND_MODEL_INPUT_WIDTH * sizeof(float);
    minimum_bytes = (size_t)HAND_MODEL_INPUT_CHANNELS *
                    (size_t)HAND_MODEL_INPUT_HEIGHT *
                    cv610->input_stride[HAND_CV610_IMAGE_INPUT_INDEX];

    if (task_name == NULL || strcmp(task_name, "task_buf") != 0 ||
        work_name == NULL || strcmp(work_name, "work_buf") != 0) {
        cv610_message(error_message,
                      error_message_size,
                      "input order must be images, task_buf, work_buf");
        return -1;
    }
    if (svp_acl_mdl_get_input_dims(cv610->description,
                                   HAND_CV610_IMAGE_INPUT_INDEX,
                                   &dimensions) != SVP_ACL_SUCCESS ||
        image_name == NULL || strcmp(image_name, "images") != 0 ||
        dimensions.dim_count != HAND_CV610_INPUT_RANK ||
        dimensions.dims[0] != 1 || dimensions.dims[1] != (int64_t)HAND_MODEL_INPUT_CHANNELS ||
        dimensions.dims[2] != (int64_t)HAND_MODEL_INPUT_HEIGHT ||
        dimensions.dims[3] != (int64_t)HAND_MODEL_INPUT_WIDTH ||
        svp_acl_mdl_get_input_data_type(cv610->description,
                                        HAND_CV610_IMAGE_INPUT_INDEX) != SVP_ACL_FLOAT ||
        svp_acl_mdl_get_input_format(cv610->description,
                                     HAND_CV610_IMAGE_INPUT_INDEX) != SVP_ACL_FORMAT_NCHW ||
        cv610->input_stride[HAND_CV610_IMAGE_INPUT_INDEX] < logical_stride ||
        cv610->input_stride[HAND_CV610_IMAGE_INPUT_INDEX] % sizeof(float) != 0U ||
        cv610->input_bytes[HAND_CV610_IMAGE_INPUT_INDEX] < minimum_bytes) {
        cv610_message(error_message,
                      error_message_size,
                      "input contract must be images FP32 NCHW [1,3,416,416]");
        return -1;
    }
    return 0;
}

static int cv610_validate_output(detector_cv610_context *cv610,
                                 char *error_message,
                                 size_t error_message_size)
{
    svp_acl_mdl_io_dims dimensions;
    const char *name;
    size_t minimum_bytes;

    if (svp_acl_mdl_get_num_outputs(cv610->description) != 1U) {
        cv610_message(error_message, error_message_size, "OM must have exactly one output");
        return -1;
    }

    name = svp_acl_mdl_get_output_name_by_index(cv610->description, 0U);
    cv610->output_bytes = svp_acl_mdl_get_output_size_by_index(cv610->description, 0U);
    cv610->output_stride = svp_acl_mdl_get_output_default_stride(cv610->description, 0U);
    minimum_bytes = (size_t)HAND_MODEL_OUTPUT_CHANNELS * cv610->output_stride;

    if (svp_acl_mdl_get_output_dims(cv610->description, 0U, &dimensions) != SVP_ACL_SUCCESS ||
        name == NULL || strcmp(name, "output0") != 0 ||
        dimensions.dim_count != HAND_CV610_OUTPUT_RANK ||
        dimensions.dims[0] != 1 ||
        dimensions.dims[1] != (int64_t)HAND_MODEL_OUTPUT_CHANNELS ||
        dimensions.dims[2] != (int64_t)HAND_MODEL_OUTPUT_CANDIDATES ||
        svp_acl_mdl_get_output_data_type(cv610->description, 0U) != SVP_ACL_FLOAT ||
        svp_acl_mdl_get_output_format(cv610->description, 0U) != SVP_ACL_FORMAT_ND ||
        cv610->output_stride <
            (size_t)HAND_MODEL_OUTPUT_CANDIDATES * sizeof(float) ||
        cv610->output_stride % sizeof(float) != 0U ||
        cv610->output_bytes < minimum_bytes) {
        cv610_message(error_message,
                      error_message_size,
                      "output contract must be output0 FP32 ND [1,5,3549]");
        return -1;
    }

    cv610->candidate_count = HAND_MODEL_OUTPUT_CANDIDATES;
    return 0;
}

static int cv610_allocate_input_tensor(detector_cv610_context *cv610,
                                       size_t index,
                                       char *error_message,
                                       size_t error_message_size)
{
    void **data;
    svp_acl_data_buffer **buffer;
    size_t size;
    size_t stride;

    if (index >= HAND_CV610_INPUT_COUNT) {
        cv610_message(error_message, error_message_size, "invalid input tensor index");
        return -1;
    }

    data = &cv610->input_data[index];
    buffer = &cv610->input_buffer[index];
    size = cv610->input_bytes[index];
    stride = cv610->input_stride[index];

    if (svp_acl_rt_malloc_cached(data,
                                 size,
                                 SVP_ACL_MEM_MALLOC_NORMAL_ONLY) != SVP_ACL_SUCCESS ||
        *data == NULL) {
        cv610_message(error_message,
                      error_message_size,
                      "failed to allocate input[%lu] tensor",
                      (unsigned long)index);
        return -1;
    }
    memset(*data, 0, size);
    if (index == HAND_CV610_IMAGE_INPUT_INDEX) {
        if (hand_preprocess_fill_padding((float *)*data,
                                         size,
                                         stride,
                                         error_message,
                                         error_message_size) != 0) {
            return -1;
        }
        cv610->input_padding_ready = true;
    }
    if (cv610_acl_check(svp_acl_rt_mem_flush(*data, size),
                        "flush input tensor",
                        error_message,
                        error_message_size) != 0) {
        return -1;
    }

    *buffer = svp_acl_create_data_buffer(*data, size, stride);
    if (*buffer == NULL ||
        cv610_acl_check(svp_acl_mdl_add_dataset_buffer(cv610->input_dataset, *buffer),
                        "add input buffer",
                        error_message,
                        error_message_size) != 0) {
        if (*buffer == NULL) {
            cv610_message(error_message,
                          error_message_size,
                          "failed to wrap input[%lu] tensor",
                          (unsigned long)index);
        }
        return -1;
    }
    return 0;
}

static int cv610_allocate_output_tensor(detector_cv610_context *cv610,
                                        char *error_message,
                                        size_t error_message_size)
{
    if (svp_acl_rt_malloc_cached(&cv610->output_data,
                                 cv610->output_bytes,
                                 SVP_ACL_MEM_MALLOC_NORMAL_ONLY) != SVP_ACL_SUCCESS ||
        cv610->output_data == NULL) {
        cv610_message(error_message, error_message_size, "failed to allocate output tensor");
        return -1;
    }
    memset(cv610->output_data, 0, cv610->output_bytes);
    if (cv610_acl_check(svp_acl_rt_mem_flush(cv610->output_data, cv610->output_bytes),
                        "initialize output tensor",
                        error_message,
                        error_message_size) != 0) {
        return -1;
    }

    cv610->output_buffer = svp_acl_create_data_buffer(cv610->output_data,
                                                      cv610->output_bytes,
                                                      cv610->output_stride);
    if (cv610->output_buffer == NULL) {
        cv610_message(error_message, error_message_size, "failed to wrap output tensor");
        return -1;
    }
    if (cv610_acl_check(svp_acl_mdl_add_dataset_buffer(cv610->output_dataset,
                                                       cv610->output_buffer),
                        "add output buffer",
                        error_message,
                        error_message_size) != 0) {
        return -1;
    }
    return 0;
}

static int cv610_prepare_model(detector_cv610_context *cv610,
                               const char *model_path,
                               char *error_message,
                               size_t error_message_size)
{
    uint32_t aicpu_tasks = 0U;
    size_t index;

    if (cv610_read_model(cv610, model_path, error_message, error_message_size) !=
        HAND_DETECTOR_OK) {
        return -1;
    }
    if (cv610_acl_check(svp_acl_mdl_load_from_mem(cv610->model_data,
                                                   cv610->model_bytes,
                                                   &cv610->model_id),
                        "load model",
                        error_message,
                        error_message_size) != 0) {
        return -1;
    }
    cv610->model_loaded = true;

    cv610->description = svp_acl_mdl_create_desc();
    if (cv610->description == NULL ||
        cv610_acl_check(svp_acl_mdl_get_desc(cv610->description, cv610->model_id),
                        "get model description",
                        error_message,
                        error_message_size) != 0) {
        return -1;
    }
    if (cv610_validate_input(cv610, error_message, error_message_size) != 0 ||
        cv610_validate_output(cv610, error_message, error_message_size) != 0) {
        return -1;
    }
    if (cv610_acl_check(svp_acl_ext_get_mdl_aicpu_task_num(cv610->model_id,
                                                            &aicpu_tasks),
                        "query AICPU task count",
                        error_message,
                        error_message_size) != 0) {
        return -1;
    }
    if (aicpu_tasks != 0U) {
        cv610_message(error_message,
                      error_message_size,
                      "OM requires %u unsupported AICPU tasks",
                      (unsigned int)aicpu_tasks);
        return -1;
    }

    cv610->input_dataset = svp_acl_mdl_create_dataset();
    cv610->output_dataset = svp_acl_mdl_create_dataset();
    if (cv610->input_dataset == NULL || cv610->output_dataset == NULL) {
        cv610_message(error_message, error_message_size, "failed to create ACL datasets");
        return -1;
    }
    for (index = 0U; index < HAND_CV610_INPUT_COUNT; ++index) {
        if (cv610_allocate_input_tensor(cv610, index, error_message, error_message_size) != 0) {
            return -1;
        }
    }
    if (cv610_allocate_output_tensor(cv610, error_message, error_message_size) != 0) {
        return -1;
    }
    if (hand_postprocess_workspace_init(&cv610->postprocess_workspace,
                                        cv610->candidate_count) != 0) {
        cv610_message(error_message, error_message_size, "failed to allocate NMS workspace");
        return -1;
    }

    hand_postprocess_config_init_defaults(&cv610->postprocess_config);
    return 0;
}

static int cv610_open(void *context,
                      const char *model_path,
                      char *error_message,
                      size_t error_message_size)
{
    detector_cv610_context *cv610 = (detector_cv610_context *)context;
    int status;

    if (cv610 == NULL) {
        cv610_message(error_message, error_message_size, "CV610 context is null");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }
    if (cv610->initialized) {
        cv610_message(error_message, error_message_size, "CV610 detector is already open");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }

    if (cv610_acl_check(svp_acl_init(NULL), "initialize", error_message, error_message_size) != 0) {
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    cv610->initialized = true;
    if (cv610_acl_check(svp_acl_rt_set_device(0), "set device", error_message, error_message_size) != 0) {
        cv610_release(cv610);
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    cv610->device_ready = true;
    if (cv610_acl_check(svp_acl_rt_get_current_context(&cv610->acl_context),
                        "get context",
                        error_message,
                        error_message_size) != 0) {
        cv610_release(cv610);
        return HAND_DETECTOR_RUNTIME_ERROR;
    }

    status = cv610_prepare_model(cv610, model_path, error_message, error_message_size) == 0
                 ? HAND_DETECTOR_OK
                 : HAND_DETECTOR_UNSUPPORTED;
    if (status != HAND_DETECTOR_OK) {
        cv610_release(cv610);
    }
    return status;
}

static int cv610_infer(void *context,
                       const hand_frame *frame,
                       hand_detection *detections,
                       size_t detection_capacity,
                       size_t *detection_count,
                       char *error_message,
                       size_t error_message_size)
{
    detector_cv610_context *cv610 = (detector_cv610_context *)context;
    hand_letterbox_transform transform;
    uint64_t total_start;
    uint64_t stage_start;

    if (detection_count != NULL) {
        *detection_count = 0U;
    }
    if (cv610 == NULL || !cv610->model_loaded || frame == NULL ||
        detections == NULL || detection_count == NULL || detection_capacity == 0U) {
        cv610_message(error_message, error_message_size, "CV610 detector is not ready");
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }
    memset(&cv610->last_timing, 0, sizeof(cv610->last_timing));
    total_start = cv610_now_ns();
    stage_start = total_start;
    if (cv610_acl_check(svp_acl_rt_set_current_context(cv610->acl_context),
                        "set inference context",
                        error_message,
                        error_message_size) != 0) {
        cv610->last_timing.set_context_ms =
            cv610_elapsed_ms(stage_start, cv610_now_ns());
        cv610_finish_timing(cv610, total_start);
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    cv610->last_timing.set_context_ms =
        cv610_elapsed_ms(stage_start, cv610_now_ns());

    stage_start = cv610_now_ns();
    {
        const int skip_padding = cv610->input_padding_ready &&
                                 (cv610->padded_source_width == 0U ||
                                  (cv610->padded_source_width == frame->width &&
                                   cv610->padded_source_height == frame->height));

        if (hand_preprocess_frame_ex(frame,
                                     (float *)cv610->input_data[HAND_CV610_IMAGE_INPUT_INDEX],
                                     cv610->input_bytes[HAND_CV610_IMAGE_INPUT_INDEX],
                                     cv610->input_stride[HAND_CV610_IMAGE_INPUT_INDEX],
                                     skip_padding,
                                     &transform,
                                     error_message,
                                     error_message_size) != 0) {
            cv610->last_timing.preprocess_ms =
                cv610_elapsed_ms(stage_start, cv610_now_ns());
            cv610_finish_timing(cv610, total_start);
            return HAND_DETECTOR_INVALID_ARGUMENT;
        }
        cv610->padded_source_width = frame->width;
        cv610->padded_source_height = frame->height;
        cv610->input_padding_ready = true;
    }
    cv610->last_timing.preprocess_ms =
        cv610_elapsed_ms(stage_start, cv610_now_ns());

    stage_start = cv610_now_ns();
    if (cv610_acl_check(svp_acl_rt_mem_flush(cv610->input_data[HAND_CV610_IMAGE_INPUT_INDEX],
                                             cv610->input_bytes[HAND_CV610_IMAGE_INPUT_INDEX]),
                        "flush input",
                        error_message,
                        error_message_size) != 0) {
        cv610->last_timing.flush_ms =
            cv610_elapsed_ms(stage_start, cv610_now_ns());
        cv610_finish_timing(cv610, total_start);
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    cv610->last_timing.flush_ms =
        cv610_elapsed_ms(stage_start, cv610_now_ns());

    stage_start = cv610_now_ns();
    if (cv610_acl_check(svp_acl_mdl_execute(cv610->model_id,
                                            cv610->input_dataset,
                                            cv610->output_dataset),
                        "execute model",
                        error_message,
                        error_message_size) != 0) {
        cv610->last_timing.npu_execute_ms =
            cv610_elapsed_ms(stage_start, cv610_now_ns());
        cv610_finish_timing(cv610, total_start);
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    cv610->last_timing.npu_execute_ms =
        cv610_elapsed_ms(stage_start, cv610_now_ns());

    stage_start = cv610_now_ns();
    if (cv610_acl_check(svp_acl_rt_mem_invalidate(cv610->output_data, cv610->output_bytes),
                        "invalidate output",
                        error_message,
                        error_message_size) != 0) {
        cv610->last_timing.invalidate_ms =
            cv610_elapsed_ms(stage_start, cv610_now_ns());
        cv610_finish_timing(cv610, total_start);
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    cv610->last_timing.invalidate_ms =
        cv610_elapsed_ms(stage_start, cv610_now_ns());

    if (cv610->score_diagnostic_frames < 3U) {
        hand_postprocess_score_diagnostic diagnostic;

        if (hand_postprocess_inspect_scores((const float *)cv610->output_data,
                                            cv610->output_bytes,
                                            cv610->output_stride,
                                            cv610->candidate_count,
                                            &transform,
                                            &diagnostic,
                                            error_message,
                                            error_message_size) == 0) {
            size_t top_index;

            fprintf(stderr,
                    "[scores] valid=%zu min=%.7g max=%.7g mean=%.7g\n",
                    diagnostic.valid_score_count,
                    diagnostic.min_score,
                    diagnostic.max_score,
                    diagnostic.mean_score);
            for (top_index = 0U; top_index < diagnostic.top_count; ++top_index) {
                const hand_postprocess_candidate *candidate =
                    &diagnostic.top[top_index];

                fprintf(stderr,
                        "[score-top %zu] score=%.7g box=(%.2f,%.2f)-(%.2f,%.2f)\n",
                        top_index,
                        candidate->score,
                        candidate->x0,
                        candidate->y0,
                        candidate->x1,
                        candidate->y1);
            }
        } else {
            fprintf(stderr, "[scores] inspect failed: %s\n", error_message);
        }
        ++cv610->score_diagnostic_frames;
    }

    stage_start = cv610_now_ns();
    if (hand_postprocess_decode((const float *)cv610->output_data,
                                cv610->output_bytes,
                                cv610->output_stride,
                                cv610->candidate_count,
                                &transform,
                                HAND_CV610_OUTPUT_CLASS_ID,
                                &cv610->postprocess_config,
                                &cv610->postprocess_workspace,
                                detections,
                                detection_capacity,
                                detection_count,
                                error_message,
                                error_message_size) != 0) {
        cv610->last_timing.postprocess_ms =
            cv610_elapsed_ms(stage_start, cv610_now_ns());
        cv610_finish_timing(cv610, total_start);
        return HAND_DETECTOR_RUNTIME_ERROR;
    }
    cv610->last_timing.postprocess_ms =
        cv610_elapsed_ms(stage_start, cv610_now_ns());
    cv610_finish_timing(cv610, total_start);
    return HAND_DETECTOR_OK;
}

static int cv610_get_timing(void *context, hand_detector_timing *timing)
{
    detector_cv610_context *cv610 = (detector_cv610_context *)context;

    if (cv610 == NULL || timing == NULL) {
        return HAND_DETECTOR_INVALID_ARGUMENT;
    }
    *timing = cv610->last_timing;
    return HAND_DETECTOR_OK;
}

static void cv610_destroy(void *context)
{
    detector_cv610_context *cv610 = (detector_cv610_context *)context;

    if (cv610 == NULL) {
        return;
    }
    cv610_release(cv610);
    free(cv610);
}

static const hand_detector_ops g_cv610_ops = {
    .name = "cv610-acl",
    .open = cv610_open,
    .infer = cv610_infer,
    .get_timing = cv610_get_timing,
    .destroy = cv610_destroy
};

hand_detector *hand_detector_create_cv610(void)
{
    detector_cv610_context *context =
        (detector_cv610_context *)calloc(1U, sizeof(*context));

    if (context == NULL) {
        return NULL;
    }
    return hand_detector_wrap(&g_cv610_ops, context);
}
