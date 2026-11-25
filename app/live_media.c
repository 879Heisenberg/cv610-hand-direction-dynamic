#include "live_media.h"

#include "sample_comm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LIVE_VI_DEV            0
#define LIVE_VI_PIPE           0
#define LIVE_VI_CHN            0
#define LIVE_VPSS_GRP          0
#define LIVE_VPSS_CHN          1
#define LIVE_VPSS_BIND_CHN     0
#define LIVE_FRAME_RATE        30
#define LIVE_FIRST_FRAME_MS    3000

struct hand_live_media {
    sample_vi_cfg vi_cfg;
    ot_size sensor_size;
    int sys_initialized;
    int vi_started;
    int vi_bound;
    int vpss_started;
};

static void set_error(char *message, size_t size, const char *text)
{
    if (message != NULL && size > 0U) {
        (void)snprintf(message, size, "%s", text);
    }
}

static void set_sdk_error(char *message,
                          size_t size,
                          const char *operation,
                          td_s32 code)
{
    if (message != NULL && size > 0U) {
        (void)snprintf(message, size, "%s failed (0x%x)", operation, (unsigned int)code);
    }
}

static void log_stage(const char *text)
{
    (void)fprintf(stderr, "[live-media] %s\n", text);
    (void)fflush(stderr);
}

static int copy_frame_as_compact_nv21(const ot_video_frame_info *frame,
                                      uint8_t *buffer,
                                      size_t buffer_size,
                                      char *error_message,
                                      size_t error_message_size)
{
    const td_u32 width = HAND_LIVE_MEDIA_WIDTH;
    const td_u32 height = HAND_LIVE_MEDIA_HEIGHT;
    const ot_video_frame *video;
    td_u32 stride_y;
    td_u32 stride_uv;
    td_u32 y_size;
    td_u32 uv_size;
    td_u8 *y_addr;
    td_u8 *uv_addr;
    td_u32 row;

    if (frame == NULL || buffer == NULL || buffer_size < HAND_LIVE_MEDIA_NV21_BYTES) {
        set_error(error_message, error_message_size, "invalid NV21 output buffer");
        return -1;
    }

    video = &frame->video_frame;
    if (video->pixel_format != OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420 ||
        video->width < width || video->height < height) {
        set_error(error_message,
                  error_message_size,
                  "VPSS frame is not a sufficiently large NV21 frame");
        return -1;
    }

    stride_y = video->stride[0];
    stride_uv = video->stride[1];
    if (stride_y < width || stride_uv < width ||
        video->phys_addr[0] == 0U || video->phys_addr[1] == 0U) {
        set_error(error_message, error_message_size, "invalid VPSS frame layout");
        return -1;
    }

    y_size = stride_y * height;
    uv_size = stride_uv * (height / 2U);
    y_addr = (td_u8 *)ss_mpi_sys_mmap(video->phys_addr[0], y_size);
    uv_addr = (td_u8 *)ss_mpi_sys_mmap(video->phys_addr[1], uv_size);
    if (y_addr == TD_NULL || uv_addr == TD_NULL) {
        if (y_addr != TD_NULL) {
            (void)ss_mpi_sys_munmap(y_addr, y_size);
        }
        if (uv_addr != TD_NULL) {
            (void)ss_mpi_sys_munmap(uv_addr, uv_size);
        }
        set_error(error_message, error_message_size, "cannot mmap VPSS frame");
        return -1;
    }

    for (row = 0U; row < height; ++row) {
        (void)memcpy(buffer + (size_t)row * width,
                     y_addr + (size_t)row * stride_y,
                     width);
    }
    buffer += (size_t)width * height;
    for (row = 0U; row < height / 2U; ++row) {
        (void)memcpy(buffer + (size_t)row * width,
                     uv_addr + (size_t)row * stride_uv,
                     width);
    }

    (void)ss_mpi_sys_munmap(uv_addr, uv_size);
    (void)ss_mpi_sys_munmap(y_addr, y_size);
    return 0;
}

int hand_live_media_read_nv21(hand_live_media *media,
                              uint8_t *buffer,
                              size_t buffer_size,
                              int timeout_ms,
                              uint32_t *width,
                              uint32_t *height,
                              char *error_message,
                              size_t error_message_size)
{
    ot_video_frame_info frame;
    ot_video_frame_info newer;
    td_s32 ret;
    int copied;

    if (media == NULL || !media->vpss_started || buffer == NULL || width == NULL ||
        height == NULL || buffer_size < HAND_LIVE_MEDIA_NV21_BYTES) {
        set_error(error_message, error_message_size, "invalid live-media read arguments");
        return -1;
    }

    (void)memset(&frame, 0, sizeof(frame));
    ret = ss_mpi_vpss_get_chn_frame(LIVE_VPSS_GRP,
                                    LIVE_VPSS_CHN,
                                    &frame,
                                    timeout_ms < 0 ? LIVE_FIRST_FRAME_MS : timeout_ms);
    if (ret != TD_SUCCESS) {
        set_sdk_error(error_message, error_message_size, "ss_mpi_vpss_get_chn_frame", ret);
        return -1;
    }

    /* Keep only the newest queued frame. During a ~374 ms inference the channel can
     * accumulate old frames; feeding them later would make the result lag behind. */
    for (;;) {
        (void)memset(&newer, 0, sizeof(newer));
        ret = ss_mpi_vpss_get_chn_frame(LIVE_VPSS_GRP, LIVE_VPSS_CHN, &newer, 0);
        if (ret != TD_SUCCESS) {
            break;
        }
        (void)ss_mpi_vpss_release_chn_frame(LIVE_VPSS_GRP, LIVE_VPSS_CHN, &frame);
        frame = newer;
    }

    copied = copy_frame_as_compact_nv21(&frame,
                                        buffer,
                                        buffer_size,
                                        error_message,
                                        error_message_size);
    (void)ss_mpi_vpss_release_chn_frame(LIVE_VPSS_GRP, LIVE_VPSS_CHN, &frame);
    if (copied != 0) {
        return -1;
    }

    *width = HAND_LIVE_MEDIA_WIDTH;
    *height = HAND_LIVE_MEDIA_HEIGHT;
    return 0;
}

int hand_live_media_open(hand_live_media **out_media,
                         char *error_message,
                         size_t error_message_size)
{
    hand_live_media *media;
    sample_vb_param vb_param;
    ot_vb_cfg vb_cfg;
    sample_vpss_cfg vpss_cfg;
    sample_vpss_chn_attr vpss_chn_attr;
    td_u32 chn;
    td_s32 ret;

    if (out_media == NULL) {
        set_error(error_message, error_message_size, "invalid live-media output pointer");
        return -1;
    }
    *out_media = NULL;
    media = (hand_live_media *)calloc(1U, sizeof(*media));
    if (media == NULL) {
        set_error(error_message, error_message_size, "cannot allocate live-media state");
        return -1;
    }

    log_stage("reading sensor geometry");
    sample_comm_vi_get_size_by_sns_type(SENSOR0_TYPE, &media->sensor_size);
    (void)fprintf(stderr,
                  "[live-media] sensor geometry: %ux%u\n",
                  (unsigned int)media->sensor_size.width,
                  (unsigned int)media->sensor_size.height);
    (void)fflush(stderr);

    (void)memset(&vb_param, 0, sizeof(vb_param));
    vb_param.vb_size[0] = media->sensor_size;
    vb_param.vb_size[1] = media->sensor_size;
    vb_param.vb_size[2].width = HAND_LIVE_MEDIA_WIDTH;
    vb_param.vb_size[2].height = HAND_LIVE_MEDIA_HEIGHT;
    vb_param.pixel_format[0] = OT_PIXEL_FORMAT_RGB_BAYER_10BPP;
    vb_param.pixel_format[1] = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    vb_param.pixel_format[2] = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    vb_param.compress_mode[0] = OT_COMPRESS_MODE_LINE;
    vb_param.compress_mode[1] = OT_COMPRESS_MODE_NONE;
    vb_param.compress_mode[2] = OT_COMPRESS_MODE_NONE;
    vb_param.video_format[0] = OT_VIDEO_FORMAT_LINEAR;
    vb_param.video_format[1] = OT_VIDEO_FORMAT_LINEAR;
    vb_param.video_format[2] = OT_VIDEO_FORMAT_LINEAR;
    vb_param.blk_num[0] = 4;
    vb_param.blk_num[1] = 4;
    vb_param.blk_num[2] = 4;
    sample_comm_sys_get_default_vb_cfg(&vb_param, &vb_cfg);

    log_stage("initializing MPP system and VB pools");
    ret = sample_comm_sys_init_with_vb_supplement(&vb_cfg, OT_VB_SUPPLEMENT_BNR_MOT_MASK);
    if (ret != TD_SUCCESS) {
        set_sdk_error(error_message,
                      error_message_size,
                      "sample_comm_sys_init_with_vb_supplement",
                      ret);
        goto fail;
    }
    media->sys_initialized = 1;

    log_stage("configuring VI/VPSS working mode");
    ret = sample_comm_vi_set_vi_vpss_mode(OT_VI_OFFLINE_VPSS_ONLINE,
                                          OT_VI_AIISP_MODE_DEFAULT);
    if (ret != TD_SUCCESS) {
        set_sdk_error(error_message, error_message_size, "sample_comm_vi_set_vi_vpss_mode", ret);
        goto fail;
    }

    sample_comm_vi_get_default_vi_cfg(SENSOR0_TYPE, &media->vi_cfg);
    sample_comm_vi_get_mipi_info_by_dev_id(SENSOR0_TYPE, LIVE_VI_DEV, &media->vi_cfg.mipi_info);
    media->vi_cfg.sns_info.bus_id = 0;
    media->vi_cfg.sns_info.sns_clk_src = 0;
    media->vi_cfg.sns_info.sns_rst_src = 0;
    media->vi_cfg.dev_info.vi_dev = LIVE_VI_DEV;
    media->vi_cfg.bind_pipe.pipe_id[0] = LIVE_VI_PIPE;
    media->vi_cfg.pipe_info[0].isp_info.isp_pub_attr.frame_rate = LIVE_FRAME_RATE;
    media->vi_cfg.pipe_info[0].isp_be_end_trigger = TD_TRUE;

    log_stage("starting VI, ISP and OS04D10 sensor");
    ret = sample_comm_vi_start_vi(&media->vi_cfg);
    if (ret != TD_SUCCESS) {
        set_sdk_error(error_message, error_message_size, "sample_comm_vi_start_vi", ret);
        goto fail;
    }
    media->vi_started = 1;

    log_stage("binding VI to VPSS");
    ret = sample_comm_vi_bind_vpss(LIVE_VI_PIPE,
                                   LIVE_VI_CHN,
                                   LIVE_VPSS_GRP,
                                   LIVE_VPSS_BIND_CHN);
    if (ret != TD_SUCCESS) {
        set_sdk_error(error_message, error_message_size, "sample_comm_vi_bind_vpss", ret);
        goto fail;
    }
    media->vi_bound = 1;

    (void)memset(&vpss_cfg, 0, sizeof(vpss_cfg));
    sample_comm_vpss_get_default_grp_attr(&vpss_cfg.grp_attr);
    vpss_cfg.grp_attr.max_width = media->sensor_size.width;
    vpss_cfg.grp_attr.max_height = media->sensor_size.height;

    for (chn = 0U; chn < OT_VPSS_MAX_PHYS_CHN_NUM; ++chn) {
        sample_comm_vpss_get_default_chn_attr(&vpss_cfg.chn_attr[chn]);
        vpss_cfg.chn_attr[chn].width = media->sensor_size.width;
        vpss_cfg.chn_attr[chn].height = media->sensor_size.height;
        vpss_cfg.chn_attr[chn].depth = 0;
        vpss_cfg.chn_en[chn] = TD_FALSE;
    }

    vpss_cfg.chn_attr[LIVE_VPSS_CHN].compress_mode = OT_COMPRESS_MODE_NONE;
    vpss_cfg.chn_attr[LIVE_VPSS_CHN].width = HAND_LIVE_MEDIA_WIDTH;
    vpss_cfg.chn_attr[LIVE_VPSS_CHN].height = HAND_LIVE_MEDIA_HEIGHT;
    vpss_cfg.chn_attr[LIVE_VPSS_CHN].depth = 1;
    vpss_cfg.chn_en[LIVE_VPSS_CHN] = TD_TRUE;
    /* VI is bound to VPSS physical channel 0. Keep it enabled as the vendor
     * capture sample does, while chn1 produces the compact NV21 stream that
     * this application reads. */
    vpss_cfg.chn_en[LIVE_VPSS_BIND_CHN] = TD_TRUE;

    (void)memset(&vpss_chn_attr, 0, sizeof(vpss_chn_attr));
    (void)memcpy(vpss_chn_attr.chn_attr,
                 vpss_cfg.chn_attr,
                 sizeof(vpss_chn_attr.chn_attr));
    (void)memcpy(vpss_chn_attr.wrap_attr,
                 vpss_cfg.wrap_attr,
                 sizeof(vpss_chn_attr.wrap_attr));
    (void)memcpy(vpss_chn_attr.chn_enable,
                 vpss_cfg.chn_en,
                 sizeof(vpss_chn_attr.chn_enable));
    vpss_chn_attr.chn_array_size = OT_VPSS_MAX_PHYS_CHN_NUM;

    log_stage("starting VPSS 640x360 NV21 output");
    ret = sample_common_vpss_start(LIVE_VPSS_GRP,
                                   &vpss_cfg.grp_attr,
                                   &vpss_chn_attr);
    if (ret != TD_SUCCESS) {
        set_sdk_error(error_message, error_message_size, "sample_common_vpss_start", ret);
        goto fail;
    }
    media->vpss_started = 1;

    (void)printf("media pipeline: OS04D10 -> VI/ISP -> VPSS grp=%d chn=%d %dx%d NV21\n",
                 LIVE_VPSS_GRP,
                 LIVE_VPSS_CHN,
                 HAND_LIVE_MEDIA_WIDTH,
                 HAND_LIVE_MEDIA_HEIGHT);
    *out_media = media;
    return 0;

fail:
    hand_live_media_close(media);
    return -1;
}

void hand_live_media_close(hand_live_media *media)
{
    td_bool chn_enable[OT_VPSS_MAX_PHYS_CHN_NUM] = {
        TD_TRUE, TD_TRUE, TD_FALSE
    };

    if (media == NULL) {
        return;
    }
    if (media->vpss_started) {
        (void)sample_common_vpss_stop(LIVE_VPSS_GRP,
                                      chn_enable,
                                      OT_VPSS_MAX_PHYS_CHN_NUM);
        media->vpss_started = 0;
    }
    if (media->vi_bound) {
        (void)sample_comm_vi_un_bind_vpss(LIVE_VI_PIPE,
                                          LIVE_VI_CHN,
                                          LIVE_VPSS_GRP,
                                          LIVE_VPSS_BIND_CHN);
        media->vi_bound = 0;
    }
    if (media->vi_started) {
        sample_comm_vi_stop_vi(&media->vi_cfg);
        media->vi_started = 0;
    }
    if (media->sys_initialized) {
        sample_comm_sys_exit();
        media->sys_initialized = 0;
    }
    free(media);
}
