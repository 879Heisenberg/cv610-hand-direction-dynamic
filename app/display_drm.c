#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include "display_drm.h"
#include "hand_direction/direction.h"
#include "hand_direction/gesture_engine.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <libdrm/drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#define HAND_DISPLAY_WIDTH 240U
#define HAND_DISPLAY_HEIGHT 320U
#define HAND_DISPLAY_MAX_CACHED_WIDTH 512U
#define HAND_DISPLAY_BUFFER_COUNT 2U
#define HAND_DISPLAY_BLACK 0x0000U
#define HAND_DISPLAY_WHITE 0xFFFFU
#define HAND_DISPLAY_CYAN 0x07FFU
#define HAND_DISPLAY_YELLOW 0xFFE0U
#define HAND_DISPLAY_GREEN 0x07E0U
#define HAND_DISPLAY_AMBER 0xFD20U
#define HAND_DISPLAY_CYAN_GLOW 0x03BFU
#define HAND_ROI_BRACKET_ARM 14
#define HAND_ROI_BRACKET_THICKNESS 2
#define HAND_GESTURE_ANIMATION_MS 1000U
#define HAND_GESTURE_ANIMATION_TRAVEL_MS 650U
#define HAND_GESTURE_ANIMATION_TAIL 0.22F

struct hand_display {
    int fd;
    uint32_t width;
    uint32_t height;
    uint32_t connector_id;
    uint32_t crtc_id;
    uint32_t crtc_mask;
    uint32_t plane_id;
    drmModeModeInfo mode;

    uint32_t fb_ids[HAND_DISPLAY_BUFFER_COUNT];
    uint32_t dumb_handles[HAND_DISPLAY_BUFFER_COUNT];
    uint32_t pitches[HAND_DISPLAY_BUFFER_COUNT];
    uint64_t dumb_sizes[HAND_DISPLAY_BUFFER_COUNT];
    void *mappings[HAND_DISPLAY_BUFFER_COUNT];
    uint32_t buffer_count;
    uint32_t front_buffer;

    uint32_t connector_crtc_prop;
    uint32_t crtc_mode_prop;
    uint32_t crtc_active_prop;
    uint32_t plane_fb_prop;
    uint32_t plane_crtc_prop;
    uint32_t plane_src_x_prop;
    uint32_t plane_src_y_prop;
    uint32_t plane_src_w_prop;
    uint32_t plane_src_h_prop;
    uint32_t plane_crtc_x_prop;
    uint32_t plane_crtc_y_prop;
    uint32_t plane_crtc_w_prop;
    uint32_t plane_crtc_h_prop;
    uint32_t plane_damage_prop;
    uint32_t mode_blob;
    uint32_t damage_blob;
    hand_display_timing last_timing;
};

static void set_error(char *message, size_t size, const char *format, ...)
{
    va_list arguments;

    if (message == NULL || size == 0U) {
        return;
    }
    va_start(arguments, format);
    (void)vsnprintf(message, size, format, arguments);
    va_end(arguments);
}

static uint32_t find_object_property(int fd,
                                     uint32_t object_id,
                                     uint32_t object_type,
                                     const char *name)
{
    drmModeObjectPropertiesPtr properties;
    uint32_t result = 0U;
    int index;

    properties = drmModeObjectGetProperties(fd, object_id, object_type);
    if (properties == NULL) {
        return 0U;
    }
    for (index = 0; index < properties->count_props; ++index) {
        drmModePropertyPtr property = drmModeGetProperty(fd, properties->props[index]);

        if (property == NULL) {
            continue;
        }
        if (strcmp(property->name, name) == 0) {
            result = property->prop_id;
        }
        drmModeFreeProperty(property);
        if (result != 0U) {
            break;
        }
    }
    drmModeFreeObjectProperties(properties);
    return result;
}

static int plane_is_primary(int fd, uint32_t plane_id)
{
    drmModeObjectPropertiesPtr properties;
    int result = 0;
    int index;

    properties = drmModeObjectGetProperties(fd, plane_id, DRM_MODE_OBJECT_PLANE);
    if (properties == NULL) {
        return 0;
    }
    for (index = 0; index < properties->count_props; ++index) {
        drmModePropertyPtr property = drmModeGetProperty(fd, properties->props[index]);
        int enum_index;

        if (property == NULL) {
            continue;
        }
        if (strcmp(property->name, "type") == 0) {
            for (enum_index = 0; enum_index < property->count_enums; ++enum_index) {
                if (strcmp(property->enums[enum_index].name, "Primary") == 0 &&
                    property->enums[enum_index].value ==
                        properties->prop_values[index]) {
                    result = 1;
                }
            }
        }
        drmModeFreeProperty(property);
        if (result != 0) {
            break;
        }
    }
    drmModeFreeObjectProperties(properties);
    return result;
}

static int set_backlight_full(char *error, size_t error_size)
{
    static const char *const brightness_path =
        "/sys/class/backlight/backlight/brightness";
    static const char *const maximum_path =
        "/sys/class/backlight/backlight/max_brightness";
    FILE *file;
    int maximum = 100;

    file = fopen(maximum_path, "r");
    if (file != NULL) {
        if (fscanf(file, "%d", &maximum) != 1 || maximum <= 0) {
            maximum = 100;
        }
        (void)fclose(file);
    }
    file = fopen(brightness_path, "w");
    if (file == NULL) {
        set_error(error, error_size, "cannot open backlight: %s", strerror(errno));
        return -1;
    }
    if (fprintf(file, "%d\n", maximum) < 0 || fclose(file) != 0) {
        set_error(error, error_size, "cannot set backlight");
        return -1;
    }
    return 0;
}

static void draw_pixel(hand_display *display,
                       uint32_t buffer_index,
                       int x,
                       int y,
                       uint16_t color)
{
    uint8_t *base;

    if (x < 0 || y < 0 || x >= (int)display->width || y >= (int)display->height) {
        return;
    }
    base = (uint8_t *)display->mappings[buffer_index] +
           (size_t)y * display->pitches[buffer_index];
    ((uint16_t *)base)[x] = color;
}

static void draw_filled_rect(hand_display *display,
                             uint32_t buffer_index,
                             int x,
                             int y,
                             int width,
                             int height,
                             uint16_t color)
{
    int64_t left = x;
    int64_t top = y;
    int64_t right = (int64_t)x + (int64_t)width;
    int64_t bottom = (int64_t)y + (int64_t)height;
    int64_t row;

    if (display == NULL || buffer_index >= display->buffer_count ||
        display->mappings[buffer_index] == NULL || width <= 0 || height <= 0) {
        return;
    }

    if (left < 0) left = 0;
    if (top < 0) top = 0;
    if (right > (int64_t)display->width) right = (int64_t)display->width;
    if (bottom > (int64_t)display->height) bottom = (int64_t)display->height;
    if (left >= right || top >= bottom) {
        return;
    }

    for (row = top; row < bottom; ++row) {
        uint8_t *base = (uint8_t *)display->mappings[buffer_index] +
                        (size_t)row * display->pitches[buffer_index];
        uint16_t *pixels = (uint16_t *)base;
        int64_t column;

        for (column = left; column < right; ++column) {
            pixels[column] = color;
        }
    }
}

static void draw_rect(hand_display *display,
                      uint32_t buffer_index,
                      int x0,
                      int y0,
                      int x1,
                      int y1,
                      int thickness,
                      uint16_t color)
{
    int index;

    if (x1 < x0 || y1 < y0) {
        return;
    }
    for (index = 0; index < thickness; ++index) {
        draw_filled_rect(display, buffer_index, x0, y0 + index, x1 - x0 + 1, 1, color);
        draw_filled_rect(display, buffer_index, x0, y1 - index, x1 - x0 + 1, 1, color);
        draw_filled_rect(display, buffer_index, x0 + index, y0, 1, y1 - y0 + 1, color);
        draw_filled_rect(display, buffer_index, x1 - index, y0, 1, y1 - y0 + 1, color);
    }
}

static void draw_round_dot(hand_display *display,
                            uint32_t buffer_index,
                            int center_x,
                            int center_y,
                            int radius,
                            uint16_t color)
{
    int y;

    for (y = -radius; y <= radius; ++y) {
        int x;
        for (x = -radius; x <= radius; ++x) {
            if (x * x + y * y <= radius * radius) {
                draw_pixel(display, buffer_index,
                           center_x + x, center_y + y, color);
            }
        }
    }
}

static void glyph_rows(char character, uint8_t rows[7])
{
    static const uint8_t space[7] = {0, 0, 0, 0, 0, 0, 0};
    static const uint8_t zero[7] = {14, 17, 19, 21, 25, 17, 14};
    static const uint8_t one[7] = {4, 12, 4, 4, 4, 4, 14};
    static const uint8_t two[7] = {14, 17, 1, 2, 4, 8, 31};
    static const uint8_t three[7] = {30, 1, 1, 14, 1, 1, 30};
    static const uint8_t four[7] = {2, 6, 10, 18, 31, 2, 2};
    static const uint8_t five[7] = {31, 16, 16, 30, 1, 1, 30};
    static const uint8_t six[7] = {14, 16, 16, 30, 17, 17, 14};
    static const uint8_t seven[7] = {31, 1, 2, 4, 8, 8, 8};
    static const uint8_t eight[7] = {14, 17, 17, 14, 17, 17, 14};
    static const uint8_t nine[7] = {14, 17, 17, 15, 1, 1, 14};
    static const uint8_t letter_a[7] = {14, 17, 17, 31, 17, 17, 17};
    static const uint8_t letter_b[7] = {30, 17, 17, 30, 17, 17, 30};
    static const uint8_t letter_c[7] = {14, 17, 16, 16, 16, 17, 14};
    static const uint8_t letter_d[7] = {30, 17, 17, 17, 17, 17, 30};
    static const uint8_t letter_e[7] = {31, 16, 16, 30, 16, 16, 31};
    static const uint8_t letter_f[7] = {31, 16, 16, 30, 16, 16, 16};
    static const uint8_t letter_g[7] = {14, 17, 16, 23, 17, 17, 15};
    static const uint8_t letter_h[7] = {17, 17, 17, 31, 17, 17, 17};
    static const uint8_t letter_i[7] = {31, 4, 4, 4, 4, 4, 31};
    static const uint8_t letter_j[7] = {7, 2, 2, 2, 18, 18, 12};
    static const uint8_t letter_k[7] = {17, 18, 20, 24, 20, 18, 17};
    static const uint8_t letter_l[7] = {16, 16, 16, 16, 16, 16, 31};
    static const uint8_t letter_m[7] = {17, 27, 21, 21, 17, 17, 17};
    static const uint8_t letter_n[7] = {17, 25, 25, 21, 19, 19, 17};
    static const uint8_t letter_o[7] = {14, 17, 17, 17, 17, 17, 14};
    static const uint8_t letter_p[7] = {30, 17, 17, 30, 16, 16, 16};
    static const uint8_t letter_q[7] = {14, 17, 17, 17, 21, 18, 13};
    static const uint8_t letter_r[7] = {30, 17, 17, 30, 20, 18, 17};
    static const uint8_t letter_s[7] = {15, 16, 16, 14, 1, 1, 30};
    static const uint8_t letter_t[7] = {31, 4, 4, 4, 4, 4, 4};
    static const uint8_t letter_u[7] = {17, 17, 17, 17, 17, 17, 14};
    static const uint8_t letter_v[7] = {17, 17, 17, 17, 17, 10, 4};
    static const uint8_t letter_w[7] = {17, 17, 17, 21, 21, 21, 10};
    static const uint8_t letter_x[7] = {17, 17, 10, 4, 10, 17, 17};
    static const uint8_t letter_y[7] = {17, 17, 10, 4, 4, 4, 4};
    static const uint8_t letter_z[7] = {31, 1, 2, 4, 8, 16, 31};
    static const uint8_t dot[7] = {0, 0, 0, 0, 0, 12, 12};
    static const uint8_t colon[7] = {0, 12, 12, 0, 12, 12, 0};
    static const uint8_t hyphen[7] = {0, 0, 0, 31, 0, 0, 0};
    const uint8_t *source = space;

    switch (character) {
    case '0': source = zero; break;
    case '1': source = one; break;
    case '2': source = two; break;
    case '3': source = three; break;
    case '4': source = four; break;
    case '5': source = five; break;
    case '6': source = six; break;
    case '7': source = seven; break;
    case '8': source = eight; break;
    case '9': source = nine; break;
    case 'A': source = letter_a; break;
    case 'B': source = letter_b; break;
    case 'C': source = letter_c; break;
    case 'D': source = letter_d; break;
    case 'E': source = letter_e; break;
    case 'F': source = letter_f; break;
    case 'G': source = letter_g; break;
    case 'H': source = letter_h; break;
    case 'I': source = letter_i; break;
    case 'J': source = letter_j; break;
    case 'K': source = letter_k; break;
    case 'L': source = letter_l; break;
    case 'M': source = letter_m; break;
    case 'N': source = letter_n; break;
    case 'O': source = letter_o; break;
    case 'P': source = letter_p; break;
    case 'Q': source = letter_q; break;
    case 'R': source = letter_r; break;
    case 'S': source = letter_s; break;
    case 'T': source = letter_t; break;
    case 'U': source = letter_u; break;
    case 'V': source = letter_v; break;
    case 'W': source = letter_w; break;
    case 'X': source = letter_x; break;
    case 'Y': source = letter_y; break;
    case 'Z': source = letter_z; break;
    case '.': source = dot; break;
    case ':': source = colon; break;
    case '-': source = hyphen; break;
    default: break;
    }
    (void)memcpy(rows, source, 7U);
}

static int text_width(const char *text, int scale)
{
    const int length = (int)strlen(text);

    return length > 0 ? length * 6 * scale - scale : 0;
}

static void draw_text(hand_display *display,
                      uint32_t buffer_index,
                      int x,
                      int y,
                      const char *text,
                      int scale,
                      uint16_t color)
{
    uint8_t rows[7];
    int character_index;

    for (character_index = 0; text[character_index] != '\0'; ++character_index) {
        int row;

        glyph_rows(text[character_index], rows);
        for (row = 0; row < 7; ++row) {
            int column;

            for (column = 0; column < 5; ++column) {
                if ((rows[row] & (1U << (4 - column))) != 0U) {
                    draw_filled_rect(display,
                                     buffer_index,
                                     x + character_index * 6 * scale + column * scale,
                                     y + row * scale,
                                     scale,
                                     scale,
                                     color);
                }
            }
        }
    }
}

static void draw_label(hand_display *display,
                       uint32_t buffer_index,
                       int x,
                       int y,
                       const char *text,
                       uint16_t foreground)
{
    const int scale = 2;
    const int width = text_width(text, scale);
    const int height = 7 * scale;

    if (x < 2) {
        x = 2;
    }
    if (y < 2) {
        y = 2;
    }
    if (x + width + 4 >= (int)display->width) {
        x = (int)display->width - width - 6;
    }
    if (y + height + 4 >= (int)display->height) {
        y = (int)display->height - height - 6;
    }
    draw_filled_rect(display, buffer_index, x - 2, y - 2, width + 4, height + 4,
                     HAND_DISPLAY_BLACK);
    draw_text(display, buffer_index, x, y, text, scale, foreground);
}

static uint16_t pack_rgb565_from_c298(int c298, int rv_bias, int guv_bias, int bu_bias)
{
    int red = (c298 + rv_bias) >> 8;
    int green = (c298 + guv_bias) >> 8;
    int blue = (c298 + bu_bias) >> 8;

    if (red < 0) red = 0;
    else if (red > 255) red = 255;
    if (green < 0) green = 0;
    else if (green > 255) green = 255;
    if (blue < 0) blue = 0;
    else if (blue > 255) blue = 255;
    return (uint16_t)(((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3));
}

static uint16_t yuv_to_rgb565(int y, int u, int v)
{
    const int c = y - 16;
    const int d = u - 128;
    const int e = v - 128;

    return pack_rgb565_from_c298(298 * c,
                                 409 * e + 128,
                                 -100 * d - 208 * e + 128,
                                 516 * d + 128);
}

static int render_camera_image(hand_display *display,
                               uint32_t buffer_index,
                               const uint8_t *nv21,
                               uint32_t source_width,
                               uint32_t source_height,
                               int crop_x,
                               int crop_y,
                               int crop_width,
                               int crop_height,
                               char *error,
                               size_t error_size)
{
    const uint8_t *luma = nv21;
    const uint8_t *chroma = nv21 + (size_t)source_width * source_height;
    uint32_t destination_x;
    uint32_t destination_y;
    uint32_t step_x;
    uint32_t step_y;

    if (nv21 == NULL || source_width < 2U || source_height < 2U ||
        crop_width <= 0 || crop_height <= 0 ||
        display->width < 2U || display->height < 2U ||
        display->mappings[buffer_index] == NULL) {
        set_error(error, error_size, "invalid camera image");
        return -1;
    }

    step_x = ((uint32_t)(crop_width - 1) << 16) / (display->width - 1U);
    step_y = ((uint32_t)(crop_height - 1) << 16) / (display->height - 1U);

    if (display->width <= HAND_DISPLAY_MAX_CACHED_WIDTH) {
        uint16_t x0_tbl[HAND_DISPLAY_MAX_CACHED_WIDTH];
        uint16_t x1_tbl[HAND_DISPLAY_MAX_CACHED_WIDTH];
        uint16_t wx_q8_tbl[HAND_DISPLAY_MAX_CACHED_WIDTH];
        uint16_t uv_x_tbl[HAND_DISPLAY_MAX_CACHED_WIDTH];

        for (destination_x = 0U; destination_x < display->width; ++destination_x) {
            const uint32_t source_x_fixed =
                ((uint32_t)crop_x << 16) + destination_x * step_x;
            const uint32_t source_x = source_x_fixed >> 16;

            x0_tbl[destination_x] = (uint16_t)source_x;
            x1_tbl[destination_x] =
                (uint16_t)(source_x + 1U < source_width ? source_x + 1U : source_x);
            wx_q8_tbl[destination_x] = (uint16_t)((source_x_fixed & 0xFFFFU) >> 8);
            uv_x_tbl[destination_x] = (uint16_t)(source_x & ~1U);
        }

        for (destination_y = 0U; destination_y < display->height; ++destination_y) {
            uint16_t *row = (uint16_t *)((uint8_t *)display->mappings[buffer_index] +
                                         (size_t)destination_y * display->pitches[buffer_index]);
            const uint32_t source_y_fixed =
                ((uint32_t)crop_y << 16) + destination_y * step_y;
            const uint32_t source_y = source_y_fixed >> 16;
            const uint32_t next_y = source_y + 1U < source_height ? source_y + 1U : source_y;
            const uint32_t wy_q8 = (source_y_fixed & 0xFFFFU) >> 8;
            const uint32_t wy0_q8 = 256U - wy_q8;
            const uint8_t *luma_row0 = luma + (size_t)source_y * source_width;
            const uint8_t *luma_row1 = luma + (size_t)next_y * source_width;
            const uint8_t *chroma_row = chroma + (size_t)(source_y / 2U) * source_width;
            uint32_t last_uv_x = UINT32_MAX;
            int rv_bias = 0;
            int guv_bias = 0;
            int bu_bias = 0;

            for (destination_x = 0U; destination_x < display->width; ++destination_x) {
                const uint32_t x0 = x0_tbl[destination_x];
                const uint32_t x1 = x1_tbl[destination_x];
                const uint32_t wx1 = wx_q8_tbl[destination_x];
                const uint32_t wx0 = 256U - wx1;
                const uint32_t top =
                    (uint32_t)luma_row0[x0] * wx0 + (uint32_t)luma_row0[x1] * wx1;
                const uint32_t bottom =
                    (uint32_t)luma_row1[x0] * wx0 + (uint32_t)luma_row1[x1] * wx1;
                const int y_value =
                    (int)((top * wy0_q8 + bottom * wy_q8 + 32768U) >> 16);
                const uint32_t uv_x = uv_x_tbl[destination_x];

                if (uv_x != last_uv_x) {
                    const int v_delta = (int)chroma_row[uv_x] - 128;
                    const int u_delta = (int)chroma_row[uv_x + 1U] - 128;

                    last_uv_x = uv_x;
                    rv_bias = 409 * v_delta + 128;
                    guv_bias = -100 * u_delta - 208 * v_delta + 128;
                    bu_bias = 516 * u_delta + 128;
                }
                row[destination_x] = pack_rgb565_from_c298(
                    298 * (y_value - 16), rv_bias, guv_bias, bu_bias);
            }
        }
        return 0;
    }

    for (destination_y = 0U; destination_y < display->height; ++destination_y) {
        uint16_t *row = (uint16_t *)((uint8_t *)display->mappings[buffer_index] +
                                    (size_t)destination_y * display->pitches[buffer_index]);
        const uint32_t source_y_fixed = ((uint32_t)crop_y << 16) +
                                        destination_y * step_y;
        const uint32_t source_y = source_y_fixed >> 16;
        const uint32_t fraction_y = source_y_fixed & 0xFFFFU;
        const uint32_t next_y = source_y + 1U < source_height ? source_y + 1U : source_y;

        for (destination_x = 0U; destination_x < display->width; ++destination_x) {
            const uint32_t source_x_fixed = ((uint32_t)crop_x << 16) +
                                            destination_x * step_x;
            const uint32_t source_x = source_x_fixed >> 16;
            const uint32_t fraction_x = source_x_fixed & 0xFFFFU;
            const uint32_t next_x = source_x + 1U < source_width ? source_x + 1U : source_x;
            const uint32_t y00 = luma[(size_t)source_y * source_width + source_x];
            const uint32_t y01 = luma[(size_t)source_y * source_width + next_x];
            const uint32_t y10 = luma[(size_t)next_y * source_width + source_x];
            const uint32_t y11 = luma[(size_t)next_y * source_width + next_x];
            const uint32_t top = (y00 * (65536U - fraction_x) + y01 * fraction_x + 32768U) >> 16;
            const uint32_t bottom = (y10 * (65536U - fraction_x) + y11 * fraction_x + 32768U) >> 16;
            const int y_value = (int)((top * (65536U - fraction_y) +
                                      bottom * fraction_y + 32768U) >> 16);
            const size_t uv_offset =
                ((size_t)(source_y / 2U) * source_width + (source_x & ~1U));
            const int v_value = chroma[uv_offset];
            const int u_value = chroma[uv_offset + 1U];

            row[destination_x] = yuv_to_rgb565(y_value, u_value, v_value);
        }
    }
    return 0;
}

static int normalized_to_x(float normalized,
                           uint32_t source_width,
                           int crop_x,
                           int crop_width)
{
    float scaled;
    float pixel;

    if (!isfinite(normalized) || source_width == 0U || crop_width <= 0) {
        return -1;
    }
    if (normalized < 0.0F) normalized = 0.0F;
    if (normalized > 1.0F) normalized = 1.0F;
    pixel = normalized * (float)source_width - (float)crop_x;
    if (!isfinite(pixel)) {
        return -1;
    }
    scaled = pixel * (float)HAND_DISPLAY_WIDTH / (float)crop_width + 0.5F;
    if (!isfinite(scaled)) {
        return -1;
    }
    if (scaled <= 0.0F) return 0;
    if (scaled >= (float)(HAND_DISPLAY_WIDTH - 1U)) {
        return (int)HAND_DISPLAY_WIDTH - 1;
    }
    return (int)scaled;
}

static int normalized_to_y(float normalized, uint32_t source_height, int crop_y, int crop_height)
{
    float scaled;
    float pixel;

    if (!isfinite(normalized) || source_height == 0U || crop_height <= 0) {
        return -1;
    }
    if (normalized < 0.0F) normalized = 0.0F;
    if (normalized > 1.0F) normalized = 1.0F;
    pixel = normalized * (float)source_height - (float)crop_y;
    if (!isfinite(pixel)) {
        return -1;
    }
    scaled = pixel * (float)HAND_DISPLAY_HEIGHT / (float)crop_height + 0.5F;
    if (!isfinite(scaled)) {
        return -1;
    }
    if (scaled <= 0.0F) return 0;
    if (scaled >= (float)(HAND_DISPLAY_HEIGHT - 1U)) {
        return (int)HAND_DISPLAY_HEIGHT - 1;
    }
    return (int)scaled;
}

static int draw_highlighted_hand_box(hand_display *display,
                                     uint32_t buffer_index,
                                     uint32_t source_width,
                                     uint32_t source_height,
                                     int crop_x,
                                     int crop_y,
                                     int crop_width,
                                     int crop_height,
                                     const hand_detection *box)
{
    int x0;
    int y0;
    int x1;
    int y1;
    char text[16];
    int score;

    if (box == NULL || !isfinite(box->x0) || !isfinite(box->y0) ||
        !isfinite(box->x1) || !isfinite(box->y1) ||
        !isfinite(box->confidence) || box->confidence <= 0.0F) {
        return 0;
    }
    x0 = normalized_to_x(box->x0, source_width, crop_x, crop_width);
    y0 = normalized_to_y(box->y0, source_height, crop_y, crop_height);
    x1 = normalized_to_x(box->x1, source_width, crop_x, crop_width);
    y1 = normalized_to_y(box->y1, source_height, crop_y, crop_height);
    if (x0 < 0 || y0 < 0 || x1 < 0 || y1 < 0 || x1 <= x0 || y1 <= y0) {
        return 0;
    }

    score = (int)(box->confidence * 100.0F + 0.5F);
    if (score < 0) score = 0;
    if (score > 999) score = 999;
    (void)snprintf(text, sizeof(text), "HAND %d.%02d", score / 100, score % 100);
    draw_rect(display, buffer_index, x0, y0, x1, y1, 2, HAND_DISPLAY_YELLOW);
    draw_label(display, buffer_index, x0, y0 >= 20 ? y0 - 19 : y0 + 4,
               text, HAND_DISPLAY_YELLOW);
    return 1;
}

static void draw_detections(hand_display *display,
                            uint32_t buffer_index,
                            uint32_t source_width,
                            uint32_t source_height,
                            int crop_x,
                            int crop_y,
                            int crop_width,
                            int crop_height,
                            const hand_detection *detections,
                            size_t detection_count,
                            const hand_display_overlay *overlay)
{
    size_t index;
    int best_index = -1;
    float best_confidence = -1.0F;

    if (overlay != NULL && !overlay->show_raw_detections) {
        if (!overlay->tracked_box_valid ||
            overlay->tracked_box.confidence < 0.50F ||
            !draw_highlighted_hand_box(display,
                                       buffer_index,
                                       source_width,
                                       source_height,
                                       crop_x,
                                       crop_y,
                                       crop_width,
                                       crop_height,
                                       &overlay->tracked_box)) {
            draw_label(display, buffer_index, 4, 4, "NO HAND", HAND_DISPLAY_WHITE);
        }
        return;
    }

    for (index = 0U; index < detection_count; ++index) {
        const hand_detection *detection = &detections[index];
        int x0;
        int y0;
        int x1;
        int y1;

        if (!isfinite(detection->x0) || !isfinite(detection->y0) ||
            !isfinite(detection->x1) || !isfinite(detection->y1) ||
            !isfinite(detection->confidence)) {
            continue;
        }
        x0 = normalized_to_x(detection->x0, source_width, crop_x, crop_width);
        y0 = normalized_to_y(detection->y0, source_height, crop_y, crop_height);
        x1 = normalized_to_x(detection->x1, source_width, crop_x, crop_width);
        y1 = normalized_to_y(detection->y1, source_height, crop_y, crop_height);
        if (x0 < 0 || y0 < 0 || x1 < 0 || y1 < 0) {
            continue;
        }

        draw_rect(display, buffer_index, x0, y0, x1, y1, 2, HAND_DISPLAY_CYAN);
        if (detection->confidence > best_confidence) {
            best_confidence = detection->confidence;
            best_index = (int)index;
        }
    }

    if (best_index >= 0 &&
        draw_highlighted_hand_box(display,
                                  buffer_index,
                                  source_width,
                                  source_height,
                                  crop_x,
                                  crop_y,
                                  crop_width,
                                  crop_height,
                                  &detections[best_index])) {
        return;
    }
    draw_label(display, buffer_index, 4, 4, "NO HAND", HAND_DISPLAY_WHITE);
}

static uint64_t display_now_ns(void)
{
    struct timespec value;

    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0U;
    }
    return (uint64_t)value.tv_sec * 1000000000ULL + (uint64_t)value.tv_nsec;
}

static uint64_t display_now_ms(void)
{
    struct timespec value;

    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0U;
    }
    return (uint64_t)value.tv_sec * 1000ULL +
           (uint64_t)value.tv_nsec / 1000000ULL;
}

static uint16_t blend_rgb565(uint16_t background, uint16_t foreground, unsigned int alpha)
{
    const unsigned int inverse = 255U - alpha;
    const unsigned int br = (background >> 11) & 31U;
    const unsigned int bg = (background >> 5) & 63U;
    const unsigned int bb = background & 31U;
    const unsigned int fr = (foreground >> 11) & 31U;
    const unsigned int fg = (foreground >> 5) & 63U;
    const unsigned int fb = foreground & 31U;
    const unsigned int red = (br * inverse + fr * alpha + 127U) / 255U;
    const unsigned int green = (bg * inverse + fg * alpha + 127U) / 255U;
    const unsigned int blue = (bb * inverse + fb * alpha + 127U) / 255U;

    return (uint16_t)((red << 11) | (green << 5) | blue);
}

static void blend_pixel(hand_display *display,
                        uint32_t buffer_index,
                        int x,
                        int y,
                        uint16_t color,
                        unsigned int alpha)
{
    uint8_t *base;
    uint16_t *pixel;

    if (display == NULL || buffer_index >= display->buffer_count ||
        display->mappings[buffer_index] == NULL || x < 0 || y < 0 ||
        x >= (int)display->width || y >= (int)display->height || alpha == 0U) {
        return;
    }
    if (alpha >= 255U) {
        draw_pixel(display, buffer_index, x, y, color);
        return;
    }
    base = (uint8_t *)display->mappings[buffer_index] +
           (size_t)y * display->pitches[buffer_index];
    pixel = (uint16_t *)base + x;
    *pixel = blend_rgb565(*pixel, color, alpha);
}

static void draw_filled_rect_alpha(hand_display *display,
                                   uint32_t buffer_index,
                                   int x,
                                   int y,
                                   int width,
                                   int height,
                                   uint16_t color,
                                   unsigned int alpha)
{
    int64_t left = x;
    int64_t top = y;
    int64_t right = (int64_t)x + (int64_t)width;
    int64_t bottom = (int64_t)y + (int64_t)height;
    int64_t row;

    if (alpha == 0U || width <= 0 || height <= 0 || display == NULL ||
        buffer_index >= display->buffer_count || display->mappings[buffer_index] == NULL) {
        return;
    }
    if (alpha >= 255U) {
        draw_filled_rect(display, buffer_index, x, y, width, height, color);
        return;
    }

    if (left < 0) left = 0;
    if (top < 0) top = 0;
    if (right > (int64_t)display->width) right = (int64_t)display->width;
    if (bottom > (int64_t)display->height) bottom = (int64_t)display->height;
    if (left >= right || top >= bottom) {
        return;
    }

    for (row = top; row < bottom; ++row) {
        uint8_t *base = (uint8_t *)display->mappings[buffer_index] +
                        (size_t)row * display->pitches[buffer_index];
        uint16_t *pixels = (uint16_t *)base;
        int64_t column;

        for (column = left; column < right; ++column) {
            pixels[column] = blend_rgb565(pixels[column], color, alpha);
        }
    }
}

static void draw_roi_hud(hand_display *display,
                         uint32_t buffer_index,
                         uint32_t source_width,
                         uint32_t source_height,
                         int crop_x,
                         int crop_y,
                         int crop_width,
                         int crop_height,
                         const hand_display_overlay *overlay)
{
    uint16_t hud_color = HAND_DISPLAY_WHITE;
    unsigned int bracket_alpha = 135U;
    const char *badge = "READY";
    int rx0;
    int ry0;
    int rx1;
    int ry1;
    int arm = HAND_ROI_BRACKET_ARM;
    const int thick = HAND_ROI_BRACKET_THICKNESS;
    int badge_x;

    if (overlay == NULL || !overlay->roi_valid) {
        return;
    }

    switch (overlay->state) {
    case HAND_ENGINE_CAPTURING:
        hud_color = HAND_DISPLAY_GREEN;
        bracket_alpha = 230U;
        badge = "ACTIVE";
        break;
    case HAND_ENGINE_COOLDOWN:
        hud_color = HAND_DISPLAY_AMBER;
        bracket_alpha = 220U;
        badge = "COOL";
        break;
    case HAND_ENGINE_WAITING_FOR_HAND:
    case HAND_ENGINE_WAITING_FOR_ACTIVATION:
    default:
        hud_color = overlay->roi_inside ? HAND_DISPLAY_CYAN : HAND_DISPLAY_WHITE;
        bracket_alpha = overlay->roi_inside ? 200U : 135U;
        badge = "READY";
        break;
    }

    rx0 = normalized_to_x(overlay->roi_x_min, source_width, crop_x, crop_width);
    ry0 = normalized_to_y(overlay->roi_y_min, source_height, crop_y, crop_height);
    rx1 = normalized_to_x(overlay->roi_x_max, source_width, crop_x, crop_width);
    ry1 = normalized_to_y(overlay->roi_y_max, source_height, crop_y, crop_height);

    if (rx0 >= 0 && ry0 >= 0 && rx1 > rx0 + 4 && ry1 > ry0 + 4) {
        const int roi_w = rx1 - rx0 + 1;
        const int roi_h = ry1 - ry0 + 1;

        if (roi_w < arm * 2) arm = roi_w / 2;
        if (roi_h < arm * 2) arm = roi_h / 2;
        if (arm > thick) {
            /* Top-left */
            draw_filled_rect_alpha(display, buffer_index, rx0, ry0, arm, thick,
                                   hud_color, bracket_alpha);
            draw_filled_rect_alpha(display, buffer_index, rx0, ry0 + thick, thick, arm - thick,
                                   hud_color, bracket_alpha);
            /* Top-right */
            draw_filled_rect_alpha(display, buffer_index, rx1 - arm + 1, ry0, arm, thick,
                                   hud_color, bracket_alpha);
            draw_filled_rect_alpha(display, buffer_index, rx1 - thick + 1, ry0 + thick, thick, arm - thick,
                                   hud_color, bracket_alpha);
            /* Bottom-left */
            draw_filled_rect_alpha(display, buffer_index, rx0, ry1 - thick + 1, arm, thick,
                                   hud_color, bracket_alpha);
            draw_filled_rect_alpha(display, buffer_index, rx0, ry1 - arm + 1, thick, arm - thick,
                                   hud_color, bracket_alpha);
            /* Bottom-right */
            draw_filled_rect_alpha(display, buffer_index, rx1 - arm + 1, ry1 - thick + 1, arm, thick,
                                   hud_color, bracket_alpha);
            draw_filled_rect_alpha(display, buffer_index, rx1 - thick + 1, ry1 - arm + 1, thick, arm - thick,
                                   hud_color, bracket_alpha);
        }

        if (overlay->state == HAND_ENGINE_COOLDOWN &&
            overlay->cooldown_total_ms > 0U &&
            overlay->cooldown_remaining_ms > 0U &&
            roi_w > 8) {
            const int bar_y = ry0 >= 6 ? ry0 - 6 : ry0 + 4;
            uint64_t remaining = overlay->cooldown_remaining_ms;
            int fill_w;

            if (remaining > overlay->cooldown_total_ms) {
                remaining = overlay->cooldown_total_ms;
            }
            fill_w = (int)((uint64_t)roi_w * remaining / overlay->cooldown_total_ms);
            draw_filled_rect_alpha(display, buffer_index, rx0, bar_y, roi_w, 3,
                                   HAND_DISPLAY_BLACK, 160U);
            if (fill_w > 0) {
                draw_filled_rect_alpha(display, buffer_index, rx0, bar_y, fill_w, 3,
                                       HAND_DISPLAY_AMBER, 235U);
            }
        }
    }

    badge_x = (int)display->width - text_width(badge, 2) - 4;
    if (badge_x < 96) {
        badge_x = 96;
    }
    draw_label(display, buffer_index, badge_x, 4, badge, hud_color);
}

static void draw_line_alpha(hand_display *display,
                            uint32_t buffer_index,
                            int x0,
                            int y0,
                            int x1,
                            int y1,
                            int thickness,
                            uint16_t color,
                            unsigned int alpha)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    unsigned int steps = 0U;
    unsigned int maximum_steps;
    const int radius = thickness > 1 ? thickness / 2 : 0;

    if (display == NULL || thickness <= 0) {
        return;
    }
    maximum_steps = display->width + display->height + 2U;
    for (;;) {
        int oy;
        if (++steps > maximum_steps) {
            return;
        }
        for (oy = -radius; oy <= radius; ++oy) {
            int ox;
            for (ox = -radius; ox <= radius; ++ox) {
                blend_pixel(display, buffer_index, x0 + ox, y0 + oy,
                            color, alpha);
            }
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        if (2 * error >= dy) {
            error += dy;
            x0 += sx;
        }
        if (2 * error <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

static int point_inside_crop(const hand_point *point,
                             uint32_t source_width,
                             uint32_t source_height,
                             int crop_x,
                             int crop_y,
                             int crop_width,
                             int crop_height)
{
    const float source_x = point->x * (float)source_width;
    const float source_y = point->y * (float)source_height;

    return isfinite(source_x) && isfinite(source_y) &&
           source_x >= (float)crop_x &&
           source_x < (float)(crop_x + crop_width) &&
           source_y >= (float)crop_y &&
           source_y < (float)(crop_y + crop_height);
}

static void gesture_animation_path(hand_direction_code direction,
                                   int *start_x,
                                   int *start_y,
                                   int *end_x,
                                   int *end_y)
{
    const int center_x = (int)HAND_DISPLAY_WIDTH / 2;
    const int center_y = (int)HAND_DISPLAY_HEIGHT / 2;

    *start_x = center_x;
    *start_y = center_y;
    *end_x = center_x;
    *end_y = center_y;

    switch (direction) {
    case HAND_DIRECTION_UP:
        *start_y = 270;
        *end_y = 50;
        break;
    case HAND_DIRECTION_DOWN:
        *start_y = 50;
        *end_y = 270;
        break;
    case HAND_DIRECTION_LEFT:
        *start_x = 205;
        *end_x = 35;
        break;
    case HAND_DIRECTION_RIGHT:
        *start_x = 35;
        *end_x = 205;
        break;
    case HAND_DIRECTION_NONE:
    default:
        break;
    }
}

static int lerp_int(int start, int end, float progress)
{
    return start + (int)((float)(end - start) * progress + 0.5F);
}

static void draw_gesture_arrow(hand_display *display,
                               uint32_t buffer_index,
                               hand_direction_code direction,
                               int tip_x,
                               int tip_y,
                               unsigned int alpha)
{
    int direction_x = 0;
    int direction_y = 0;
    int perpendicular_x;
    int perpendicular_y;
    int side;

    switch (direction) {
    case HAND_DIRECTION_UP:
        direction_y = -1;
        break;
    case HAND_DIRECTION_DOWN:
        direction_y = 1;
        break;
    case HAND_DIRECTION_LEFT:
        direction_x = -1;
        break;
    case HAND_DIRECTION_RIGHT:
        direction_x = 1;
        break;
    case HAND_DIRECTION_NONE:
    default:
        return;
    }

    perpendicular_x = -direction_y;
    perpendicular_y = direction_x;
    for (side = -1; side <= 1; side += 2) {
        const int wing_x = tip_x - direction_x * 9 + perpendicular_x * 5 * side;
        const int wing_y = tip_y - direction_y * 9 + perpendicular_y * 5 * side;
        const unsigned int glow_alpha = alpha / 4U;

        draw_line_alpha(display, buffer_index, tip_x, tip_y, wing_x, wing_y,
                        5, HAND_DISPLAY_CYAN_GLOW, glow_alpha);
        draw_line_alpha(display, buffer_index, tip_x, tip_y, wing_x, wing_y,
                        3, HAND_DISPLAY_WHITE, alpha);
    }
    draw_round_dot(display, buffer_index, tip_x, tip_y, 3, HAND_DISPLAY_WHITE);
}

static void draw_gesture_overlay(hand_display *display,
                                 uint32_t buffer_index,
                                 uint32_t source_width,
                                 uint32_t source_height,
                                 int crop_x,
                                 int crop_y,
                                 int crop_width,
                                 int crop_height,
                                 const hand_display_overlay *overlay)
{
    if (overlay == NULL) {
        return;
    }

    draw_roi_hud(display,
                 buffer_index,
                 source_width,
                 source_height,
                 crop_x,
                 crop_y,
                 crop_width,
                 crop_height,
                 overlay);

    /* A persistent event snapshot replays one fixed, screen-relative motion.
     * No measured hand points are drawn here, so a tracker discontinuity cannot
     * create a line spanning the LCD. */
    if (overlay->event_valid && overlay->event.direction != HAND_DIRECTION_NONE &&
        overlay->event.timestamp_ms != 0U) {
        const uint64_t now_ms = display_now_ms();
        uint64_t animation_ms = overlay->event_animation_ms;
        uint64_t travel_ms;
        uint64_t age_ms;

        if (animation_ms == 0U) {
            animation_ms = HAND_GESTURE_ANIMATION_MS;
        }
        travel_ms = animation_ms < HAND_GESTURE_ANIMATION_TRAVEL_MS
                        ? animation_ms
                        : HAND_GESTURE_ANIMATION_TRAVEL_MS;

        if (now_ms >= overlay->event.timestamp_ms) {
            age_ms = now_ms - overlay->event.timestamp_ms;
            if (age_ms < animation_ms) {
                int start_x;
                int start_y;
                int end_x;
                int end_y;
                float head_progress = 1.0F;
                float tail_progress = 0.0F;
                unsigned int core_alpha = 255U;
                int head_x;
                int head_y;
                int tail_x;
                int tail_y;
                uint16_t label_color;

                gesture_animation_path(overlay->event.direction,
                                       &start_x, &start_y, &end_x, &end_y);

                if (age_ms < travel_ms) {
                    const float linear = (float)age_ms / (float)travel_ms;
                    head_progress = linear * linear * (3.0F - 2.0F * linear);
                    tail_progress = head_progress > HAND_GESTURE_ANIMATION_TAIL
                                        ? head_progress - HAND_GESTURE_ANIMATION_TAIL
                                        : 0.0F;
                } else if (animation_ms > travel_ms) {
                    const uint64_t fade_age = age_ms - travel_ms;
                    const uint64_t fade_ms = animation_ms - travel_ms;
                    core_alpha = 255U - (unsigned int)(fade_age * 255U / fade_ms);
                }

                head_x = lerp_int(start_x, end_x, head_progress);
                head_y = lerp_int(start_y, end_y, head_progress);
                tail_x = lerp_int(start_x, end_x, tail_progress);
                tail_y = lerp_int(start_y, end_y, tail_progress);

                draw_line_alpha(display, buffer_index,
                                tail_x, tail_y, head_x, head_y,
                                7, HAND_DISPLAY_CYAN_GLOW, core_alpha / 5U);
                draw_line_alpha(display, buffer_index,
                                tail_x, tail_y, head_x, head_y,
                                3, HAND_DISPLAY_CYAN, core_alpha * 3U / 4U);
                draw_gesture_arrow(display, buffer_index,
                                   overlay->event.direction,
                                   head_x, head_y, core_alpha);

                label_color = blend_rgb565(HAND_DISPLAY_BLACK,
                                            HAND_DISPLAY_YELLOW,
                                            core_alpha);
                draw_label(display, buffer_index, 4, 30,
                           hand_direction_name(overlay->event.direction),
                           label_color);
            }
        }
    }

    if (overlay->center_valid &&
        isfinite(overlay->center.x) && isfinite(overlay->center.y) &&
        overlay->center.x >= 0.0F && overlay->center.x <= 1.0F &&
        overlay->center.y >= 0.0F && overlay->center.y <= 1.0F &&
        point_inside_crop(&overlay->center, source_width, source_height,
                          crop_x, crop_y, crop_width, crop_height)) {
        const int center_x = normalized_to_x(overlay->center.x, source_width,
                                             crop_x, crop_width);
        const int center_y = normalized_to_y(overlay->center.y, source_height,
                                             crop_y, crop_height);
        if (center_x >= 0 && center_y >= 0) {
            draw_round_dot(display, buffer_index, center_x, center_y, 3,
                           HAND_DISPLAY_BLACK);
            draw_round_dot(display, buffer_index, center_x, center_y, 1,
                           HAND_DISPLAY_WHITE);
        }
    }
}

static int select_connector_and_crtc(hand_display *display,
                                     char *error,
                                     size_t error_size)
{
    drmModeResPtr resources = drmModeGetResources(display->fd);
    drmModeConnectorPtr selected_connector = NULL;
    drmModeModeInfo selected_mode;
    int selected_index = -1;
    int connector_index;
    int mode_index;

    if (resources == NULL) {
        set_error(error, error_size, "drmModeGetResources: %s", strerror(errno));
        return -1;
    }
    (void)memset(&selected_mode, 0, sizeof(selected_mode));

    for (connector_index = 0; connector_index < resources->count_connectors;
         ++connector_index) {
        drmModeConnectorPtr connector =
            drmModeGetConnector(display->fd, resources->connectors[connector_index]);

        if (connector == NULL || connector->connection != DRM_MODE_CONNECTED ||
            connector->count_modes <= 0) {
            if (connector != NULL) drmModeFreeConnector(connector);
            continue;
        }
        selected_connector = connector;
        break;
    }
    if (selected_connector == NULL) {
        drmModeFreeResources(resources);
        set_error(error, error_size, "no connected DRM display");
        return -1;
    }

    for (mode_index = 0; mode_index < selected_connector->count_modes; ++mode_index) {
        if (selected_connector->modes[mode_index].hdisplay == HAND_DISPLAY_WIDTH &&
            selected_connector->modes[mode_index].vdisplay == HAND_DISPLAY_HEIGHT) {
            selected_index = mode_index;
            break;
        }
    }
    if (selected_index < 0) {
        for (mode_index = 0; mode_index < selected_connector->count_modes; ++mode_index) {
            if ((selected_connector->modes[mode_index].type & DRM_MODE_TYPE_PREFERRED) != 0U) {
                selected_index = mode_index;
                break;
            }
        }
    }
    if (selected_index < 0) selected_index = 0;
    selected_mode = selected_connector->modes[selected_index];

    display->connector_id = selected_connector->connector_id;
    display->mode = selected_mode;
    display->width = selected_mode.hdisplay;
    display->height = selected_mode.vdisplay;

    {
        drmModeEncoderPtr encoder = NULL;
        int encoder_index;
        int crtc_index;

        if (selected_connector->encoder_id != 0U) {
            encoder = drmModeGetEncoder(display->fd, selected_connector->encoder_id);
        }
        if (encoder == NULL) {
            for (encoder_index = 0; encoder_index < resources->count_encoders;
                 ++encoder_index) {
                drmModeEncoderPtr candidate =
                    drmModeGetEncoder(display->fd, resources->encoders[encoder_index]);

                if (candidate == NULL) continue;
                if (candidate->possible_crtcs != 0U) {
                    encoder = candidate;
                    break;
                }
                drmModeFreeEncoder(candidate);
            }
        }
        if (encoder == NULL) {
            drmModeFreeConnector(selected_connector);
            drmModeFreeResources(resources);
            set_error(error, error_size, "display has no usable encoder");
            return -1;
        }

        display->crtc_mask = encoder->possible_crtcs;
        display->crtc_id = encoder->crtc_id;
        if (display->crtc_id == 0U) {
            for (crtc_index = 0; crtc_index < resources->count_crtcs; ++crtc_index) {
                if ((display->crtc_mask & (1U << crtc_index)) != 0U) {
                    display->crtc_id = resources->crtcs[crtc_index];
                    break;
                }
            }
        }
        if (display->crtc_id == 0U) {
            drmModeFreeEncoder(encoder);
            drmModeFreeConnector(selected_connector);
            drmModeFreeResources(resources);
            set_error(error, error_size, "display has no usable CRTC");
            return -1;
        }
        for (crtc_index = 0; crtc_index < resources->count_crtcs; ++crtc_index) {
            if (resources->crtcs[crtc_index] == display->crtc_id) {
                display->crtc_mask = 1U << crtc_index;
                break;
            }
        }
        drmModeFreeEncoder(encoder);
    }

    drmModeFreeConnector(selected_connector);
    drmModeFreeResources(resources);
    return 0;
}

static int select_primary_plane(hand_display *display,
                                char *error,
                                size_t error_size)
{
    drmModePlaneResPtr planes = drmModeGetPlaneResources(display->fd);
    uint32_t fallback_plane = 0U;
    int index;

    if (planes == NULL) {
        set_error(error, error_size, "drmModeGetPlaneResources: %s", strerror(errno));
        return -1;
    }
    for (index = 0; index < planes->count_planes; ++index) {
        drmModePlanePtr plane = drmModeGetPlane(display->fd, planes->planes[index]);

        if (plane == NULL) continue;
        if ((plane->possible_crtcs & display->crtc_mask) != 0U) {
            if (fallback_plane == 0U) fallback_plane = plane->plane_id;
            if (plane_is_primary(display->fd, plane->plane_id)) {
                display->plane_id = plane->plane_id;
                drmModeFreePlane(plane);
                break;
            }
        }
        drmModeFreePlane(plane);
    }
    if (display->plane_id == 0U) display->plane_id = fallback_plane;
    drmModeFreePlaneResources(planes);
    if (display->plane_id == 0U) {
        set_error(error, error_size, "display has no usable plane");
        return -1;
    }
    return 0;
}

static int resolve_atomic_properties(hand_display *display,
                                     char *error,
                                     size_t error_size)
{
    display->connector_crtc_prop = find_object_property(
        display->fd, display->connector_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
    display->crtc_mode_prop = find_object_property(
        display->fd, display->crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
    display->crtc_active_prop = find_object_property(
        display->fd, display->crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
    display->plane_fb_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID");
    display->plane_crtc_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID");
    display->plane_src_x_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X");
    display->plane_src_y_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y");
    display->plane_src_w_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W");
    display->plane_src_h_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H");
    display->plane_crtc_x_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X");
    display->plane_crtc_y_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y");
    display->plane_crtc_w_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W");
    display->plane_crtc_h_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H");
    display->plane_damage_prop = find_object_property(
        display->fd, display->plane_id, DRM_MODE_OBJECT_PLANE, "FB_DAMAGE_CLIPS");

    if (display->connector_crtc_prop == 0U ||
        display->crtc_mode_prop == 0U ||
        display->crtc_active_prop == 0U ||
        display->plane_fb_prop == 0U ||
        display->plane_crtc_prop == 0U ||
        display->plane_src_x_prop == 0U ||
        display->plane_src_y_prop == 0U ||
        display->plane_src_w_prop == 0U ||
        display->plane_src_h_prop == 0U ||
        display->plane_crtc_x_prop == 0U ||
        display->plane_crtc_y_prop == 0U ||
        display->plane_crtc_w_prop == 0U ||
        display->plane_crtc_h_prop == 0U) {
        set_error(error, error_size,
                  "DRM atomic properties are incomplete "
                  "(connector CRTC_ID, CRTC MODE_ID/ACTIVE, plane IDs, "
                  "rectangle required)");
        return -1;
    }
    return 0;
}

static int create_frame_buffer(hand_display *display,
                               uint32_t buffer_index,
                               char *error,
                               size_t error_size)
{
    struct drm_mode_create_dumb create = {0};
    struct drm_mode_map_dumb map_request = {0};
    uint32_t handles[4] = {0U};
    uint32_t pitches[4] = {0U};
    uint32_t offsets[4] = {0U};
    void *mapping;
    int framebuffer_created = 0;

    create.width = display->width;
    create.height = display->height;
    create.bpp = 16U;
    if (drmIoctl(display->fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0) {
        set_error(error, error_size, "DRM_IOCTL_MODE_CREATE_DUMB: %s", strerror(errno));
        return -1;
    }

    display->dumb_handles[buffer_index] = create.handle;
    display->pitches[buffer_index] = create.pitch;
    display->dumb_sizes[buffer_index] = create.size;

    handles[0] = create.handle;
    pitches[0] = create.pitch;
    offsets[0] = 0U;
    if (drmModeAddFB2(display->fd,
                      display->width,
                      display->height,
                      DRM_FORMAT_RGB565,
                      handles,
                      pitches,
                      offsets,
                      &display->fb_ids[buffer_index],
                      0U) != 0) {
        set_error(error, error_size, "drmModeAddFB2 RGB565: %s", strerror(errno));
        goto fail;
    }
    framebuffer_created = 1;

    map_request.handle = create.handle;
    if (drmIoctl(display->fd, DRM_IOCTL_MODE_MAP_DUMB, &map_request) != 0) {
        set_error(error, error_size, "DRM_IOCTL_MODE_MAP_DUMB: %s", strerror(errno));
        goto fail;
    }
    mapping = mmap(NULL,
                   (size_t)create.size,
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED,
                   display->fd,
                   (off_t)map_request.offset);
    if (mapping == MAP_FAILED) {
        set_error(error, error_size, "mmap RGB565 framebuffer: %s", strerror(errno));
        goto fail;
    }
    display->mappings[buffer_index] = mapping;
    (void)memset(mapping, 0, (size_t)create.size);
    return 0;

fail:
    if (framebuffer_created && display->fb_ids[buffer_index] != 0U) {
        (void)drmModeRmFB(display->fd, display->fb_ids[buffer_index]);
        display->fb_ids[buffer_index] = 0U;
    }
    if (display->dumb_handles[buffer_index] != 0U) {
        struct drm_mode_destroy_dumb destroy = {0};

        destroy.handle = display->dumb_handles[buffer_index];
        (void)drmIoctl(display->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        display->dumb_handles[buffer_index] = 0U;
    }
    return -1;
}

static void destroy_frame_buffer(hand_display *display, uint32_t buffer_index)
{
    if (display == NULL || buffer_index >= HAND_DISPLAY_BUFFER_COUNT) {
        return;
    }
    if (display->mappings[buffer_index] != NULL) {
        (void)munmap(display->mappings[buffer_index],
                     (size_t)display->dumb_sizes[buffer_index]);
        display->mappings[buffer_index] = NULL;
    }
    if (display->fb_ids[buffer_index] != 0U) {
        (void)drmModeRmFB(display->fd, display->fb_ids[buffer_index]);
        display->fb_ids[buffer_index] = 0U;
    }
    if (display->dumb_handles[buffer_index] != 0U) {
        struct drm_mode_destroy_dumb destroy = {0};

        destroy.handle = display->dumb_handles[buffer_index];
        (void)drmIoctl(display->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        display->dumb_handles[buffer_index] = 0U;
    }
}

static int add_atomic_property(drmModeAtomicReqPtr request,
                               uint32_t object_id,
                               uint32_t property_id,
                               uint64_t value,
                               const char *property_name,
                               char *error,
                               size_t error_size)
{
    if (drmModeAtomicAddProperty(request, object_id, property_id, value) < 0) {
        set_error(error, error_size,
                  "cannot add atomic property %s: %s",
                  property_name,
                  strerror(errno));
        return -1;
    }
    return 0;
}

static int add_plane_properties(hand_display *display,
                                drmModeAtomicReqPtr request,
                                uint32_t framebuffer_id,
                                char *error,
                                size_t error_size)
{
    if (add_atomic_property(request, display->plane_id, display->plane_fb_prop,
                            framebuffer_id, "plane.FB_ID", error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_crtc_prop,
                            display->crtc_id, "plane.CRTC_ID", error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_src_x_prop,
                            0U, "plane.SRC_X", error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_src_y_prop,
                            0U, "plane.SRC_Y", error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_src_w_prop,
                            (uint64_t)display->width << 16, "plane.SRC_W",
                            error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_src_h_prop,
                            (uint64_t)display->height << 16, "plane.SRC_H",
                            error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_crtc_x_prop,
                            0U, "plane.CRTC_X", error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_crtc_y_prop,
                            0U, "plane.CRTC_Y", error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_crtc_w_prop,
                            display->width, "plane.CRTC_W", error, error_size) != 0 ||
        add_atomic_property(request, display->plane_id, display->plane_crtc_h_prop,
                            display->height, "plane.CRTC_H", error, error_size) != 0) {
        return -1;
    }
    if (display->plane_damage_prop != 0U &&
        add_atomic_property(request, display->plane_id, display->plane_damage_prop,
                            display->damage_blob, "plane.FB_DAMAGE_CLIPS",
                            error, error_size) != 0) {
        return -1;
    }
    return 0;
}

static int commit_modeset(hand_display *display,
                          uint32_t framebuffer_id,
                          char *error,
                          size_t error_size)
{
    drmModeAtomicReqPtr request = drmModeAtomicAlloc();
    int result;

    if (request == NULL) {
        set_error(error, error_size, "drmModeAtomicAlloc failed");
        return -1;
    }
    if (add_atomic_property(request, display->connector_id,
                            display->connector_crtc_prop, display->crtc_id,
                            "connector.CRTC_ID", error, error_size) != 0 ||
        add_atomic_property(request, display->crtc_id, display->crtc_mode_prop,
                            display->mode_blob, "crtc.MODE_ID",
                            error, error_size) != 0 ||
        add_atomic_property(request, display->crtc_id, display->crtc_active_prop,
                            1U, "crtc.ACTIVE", error, error_size) != 0 ||
        add_plane_properties(display, request, framebuffer_id, error, error_size) != 0) {
        drmModeAtomicFree(request);
        return -1;
    }

    result = drmModeAtomicCommit(display->fd,
                                 request,
                                 DRM_MODE_ATOMIC_ALLOW_MODESET,
                                 NULL);
    drmModeAtomicFree(request);
    if (result != 0) {
        set_error(error, error_size, "initial DRM atomic commit: %s", strerror(errno));
        return -1;
    }
    return 0;
}

static int commit_frame(hand_display *display,
                        uint32_t framebuffer_id,
                        char *error,
                        size_t error_size)
{
    drmModeAtomicReqPtr request = drmModeAtomicAlloc();
    int result;

    if (request == NULL) {
        set_error(error, error_size, "drmModeAtomicAlloc failed");
        return -1;
    }
    if (add_plane_properties(display, request, framebuffer_id, error, error_size) != 0) {
        drmModeAtomicFree(request);
        return -1;
    }
    result = drmModeAtomicCommit(display->fd, request, 0U, NULL);
    drmModeAtomicFree(request);
    if (result != 0) {
        set_error(error, error_size, "DRM frame commit: %s", strerror(errno));
        return -1;
    }
    return 0;
}

int hand_display_open(hand_display **display,
                      char *error,
                      size_t error_size)
{
    static const char *const default_device = "/dev/dri/card0";
    const char *device_path = getenv("HAND_DRM_DEVICE");
    hand_display *opened;
    struct drm_mode_rect damage;
    uint64_t has_dumb = 0U;
    uint32_t index;

    if (display == NULL) {
        set_error(error, error_size, "display output pointer is null");
        return -1;
    }
    *display = NULL;
    if (device_path == NULL || device_path[0] == '\0') {
        device_path = default_device;
    }

    opened = (hand_display *)calloc(1U, sizeof(*opened));
    if (opened == NULL) {
        set_error(error, error_size, "cannot allocate display state");
        return -1;
    }
    opened->fd = -1;
    opened->buffer_count = HAND_DISPLAY_BUFFER_COUNT;

    opened->fd = open(device_path, O_RDWR | O_CLOEXEC);
    if (opened->fd < 0) {
        set_error(error, error_size, "cannot open %s: %s", device_path, strerror(errno));
        hand_display_close(opened);
        return -1;
    }
    if (drmGetCap(opened->fd, DRM_CAP_DUMB_BUFFER, &has_dumb) != 0 || has_dumb == 0U) {
        set_error(error, error_size, "%s does not support DRM dumb buffers", device_path);
        hand_display_close(opened);
        return -1;
    }
    if (drmSetClientCap(opened->fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0 ||
        drmSetClientCap(opened->fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
        set_error(error, error_size, "DRM atomic/universal-plane API unavailable: %s",
                  strerror(errno));
        hand_display_close(opened);
        return -1;
    }
    if (select_connector_and_crtc(opened, error, error_size) != 0 ||
        select_primary_plane(opened, error, error_size) != 0 ||
        resolve_atomic_properties(opened, error, error_size) != 0) {
        hand_display_close(opened);
        return -1;
    }

    for (index = 0U; index < opened->buffer_count; ++index) {
        if (create_frame_buffer(opened, index, error, error_size) != 0) {
            hand_display_close(opened);
            return -1;
        }
    }

    if (drmModeCreatePropertyBlob(opened->fd,
                                  &opened->mode,
                                  sizeof(opened->mode),
                                  &opened->mode_blob) != 0) {
        set_error(error, error_size, "cannot create DRM mode blob: %s", strerror(errno));
        hand_display_close(opened);
        return -1;
    }
    if (opened->plane_damage_prop != 0U) {
        damage.x1 = 0;
        damage.y1 = 0;
        damage.x2 = (int32_t)opened->width;
        damage.y2 = (int32_t)opened->height;
        if (drmModeCreatePropertyBlob(opened->fd,
                                      &damage,
                                      sizeof(damage),
                                      &opened->damage_blob) != 0) {
            set_error(error, error_size, "cannot create DRM damage blob: %s", strerror(errno));
            hand_display_close(opened);
            return -1;
        }
    }
    if (commit_modeset(opened, opened->fb_ids[0], error, error_size) != 0) {
        hand_display_close(opened);
        return -1;
    }
    opened->front_buffer = 0U;

    if (set_backlight_full(error, error_size) != 0) {
        hand_display_close(opened);
        return -1;
    }

    *display = opened;
    return 0;
}

int hand_display_visible_rect(uint32_t source_width,
                              uint32_t source_height,
                              int *crop_x,
                              int *crop_y,
                              int *crop_width,
                              int *crop_height)
{
    uint64_t portrait_crop_width;
    int width;
    int height;
    int x;
    int y;

    if (crop_x == NULL || crop_y == NULL || crop_width == NULL ||
        crop_height == NULL || source_width < 2U || source_height < 2U) {
        return -1;
    }

    portrait_crop_width =
        ((uint64_t)source_height * HAND_DISPLAY_WIDTH) / HAND_DISPLAY_HEIGHT;
    if (portrait_crop_width <= source_width) {
        width = (int)portrait_crop_width;
        height = (int)source_height;
    } else {
        width = (int)source_width;
        height = (int)(((uint64_t)source_width * HAND_DISPLAY_HEIGHT) /
                       HAND_DISPLAY_WIDTH);
    }

    width &= ~1;
    if (width < 2) width = 2;
    if (height < 2) height = 2;
    x = ((int)source_width - width) / 2;
    y = ((int)source_height - height) / 2;
    x &= ~1;

    *crop_x = x;
    *crop_y = y;
    *crop_width = width;
    *crop_height = height;
    return 0;
}

int hand_display_render(hand_display *display,
                        const uint8_t *nv21,
                        uint32_t width,
                        uint32_t height,
                        const hand_detection *detections,
                        size_t detection_count,
                        const hand_display_overlay *overlay,
                        char *error,
                        size_t error_size)
{
    uint32_t buffer_index;
    int crop_x;
    int crop_y;
    int crop_width;
    int crop_height;
    uint64_t t_start_ns;
    uint64_t t_cpu_done_ns;
    uint64_t t_commit_done_ns;

    if (display == NULL || display->fd < 0 || display->buffer_count < 2U ||
        width < 2U || height < 2U || (detection_count > 0U && detections == NULL)) {
        set_error(error, error_size, "invalid display/render input");
        return -1;
    }

    if (hand_display_visible_rect(width,
                                  height,
                                  &crop_x,
                                  &crop_y,
                                  &crop_width,
                                  &crop_height) != 0) {
        set_error(error, error_size, "cannot calculate display crop");
        return -1;
    }

    t_start_ns = display_now_ns();
    buffer_index = (display->front_buffer + 1U) % display->buffer_count;
    if (render_camera_image(display,
                            buffer_index,
                            nv21,
                            width,
                            height,
                            crop_x,
                            crop_y,
                            crop_width,
                            crop_height,
                            error,
                            error_size) != 0) {
        return -1;
    }
    draw_detections(display,
                    buffer_index,
                    width,
                    height,
                    crop_x,
                    crop_y,
                    crop_width,
                    crop_height,
                    detections,
                    detection_count,
                    overlay);
    draw_gesture_overlay(display,
                         buffer_index,
                         width,
                         height,
                         crop_x,
                         crop_y,
                         crop_width,
                         crop_height,
                         overlay);
    t_cpu_done_ns = display_now_ns();
    if (t_cpu_done_ns >= t_start_ns && t_start_ns != 0U) {
        display->last_timing.render_cpu_ms =
            (double)(t_cpu_done_ns - t_start_ns) / 1000000.0;
    } else {
        display->last_timing.render_cpu_ms = 0.0;
    }

    if (commit_frame(display, display->fb_ids[buffer_index], error, error_size) != 0) {
        return -1;
    }
    t_commit_done_ns = display_now_ns();
    if (t_commit_done_ns >= t_cpu_done_ns && t_cpu_done_ns != 0U) {
        display->last_timing.drm_commit_ms =
            (double)(t_commit_done_ns - t_cpu_done_ns) / 1000000.0;
    } else {
        display->last_timing.drm_commit_ms = 0.0;
    }
    display->front_buffer = buffer_index;
    return 0;
}

void hand_display_get_last_timing(const hand_display *display,
                                  hand_display_timing *timing)
{
    if (timing == NULL) {
        return;
    }
    (void)memset(timing, 0, sizeof(*timing));
    if (display != NULL) {
        *timing = display->last_timing;
    }
}

void hand_display_close(hand_display *display)
{
    uint32_t index;

    if (display == NULL) {
        return;
    }
    if (display->fd >= 0) {
        if (display->damage_blob != 0U) {
            (void)drmModeDestroyPropertyBlob(display->fd, display->damage_blob);
            display->damage_blob = 0U;
        }
        if (display->mode_blob != 0U) {
            (void)drmModeDestroyPropertyBlob(display->fd, display->mode_blob);
            display->mode_blob = 0U;
        }
        for (index = 0U; index < HAND_DISPLAY_BUFFER_COUNT; ++index) {
            destroy_frame_buffer(display, index);
        }
        (void)close(display->fd);
        display->fd = -1;
    }
    free(display);
}




