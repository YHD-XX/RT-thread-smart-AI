#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "k_type.h"
#include "mpi_sys_api.h"

#include "motion_osd.h"

#define MOTION_OSD_HOLD_TICKS 15

/*
 * ARGB8888颜色。
 * 即使设备实际显示的红蓝顺序有差异，也不影响文字可见性。
 */
#define MOTION_OSD_BG_COLOR      0xC0B02020U
#define MOTION_OSD_TEXT_COLOR    0xFFFFFFFFU
#define MOTION_OSD_BORDER_COLOR  0xFFFFFF00U

typedef struct
{
    char ch;
    k_u8 rows[7];
} motion_glyph_t;

/*
 * 只保存“MOTION DETECTED!”需要的字符。
 * 每个字符为5×7点阵。
 */
static const motion_glyph_t g_motion_font[] =
{
    {
        'M',
        {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}
    },
    {
        'O',
        {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}
    },
    {
        'T',
        {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}
    },
    {
        'I',
        {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F}
    },
    {
        'N',
        {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}
    },
    {
        'D',
        {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}
    },
    {
        'E',
        {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}
    },
    {
        'C',
        {0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F}
    },
    {
        '!',
        {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04}
    }
};

static pthread_mutex_t g_motion_osd_lock =
    PTHREAD_MUTEX_INITIALIZER;

static k_u64 g_motion_osd_phys_addr = 0;
static k_u32 *g_motion_osd_pixels = NULL;
static k_u32 g_motion_osd_width = 0;
static k_u32 g_motion_osd_height = 0;
static k_u32 g_motion_osd_buffer_size = 0;
static int g_motion_osd_hold_ticks = 0;
static int g_motion_osd_bound = 0;

static const k_u8 *motion_osd_find_glyph(char ch)
{
    k_u32 i;

    for (i = 0;
         i < sizeof(g_motion_font) / sizeof(g_motion_font[0]);
         ++i)
    {
        if (g_motion_font[i].ch == ch)
            return g_motion_font[i].rows;
    }

    return NULL;
}

static void motion_osd_flush_locked(void)
{
    if (!g_motion_osd_bound ||
        !g_motion_osd_pixels ||
        g_motion_osd_phys_addr == 0)
    {
        return;
    }

    kd_mpi_sys_mmz_flush_cache(
        g_motion_osd_phys_addr,
        g_motion_osd_pixels,
        g_motion_osd_buffer_size);
}

static void motion_osd_clear_locked(void)
{
    if (!g_motion_osd_bound || !g_motion_osd_pixels)
        return;

    memset(
        g_motion_osd_pixels,
        0,
        g_motion_osd_buffer_size);
}

static void motion_osd_set_pixel_locked(k_s32 x,
                                        k_s32 y,
                                        k_u32 color)
{
    if (x < 0 || y < 0)
        return;

    if ((k_u32)x >= g_motion_osd_width ||
        (k_u32)y >= g_motion_osd_height)
    {
        return;
    }

    g_motion_osd_pixels[
        (k_u32)y * g_motion_osd_width + (k_u32)x] = color;
}

static void motion_osd_fill_rect_locked(k_s32 x,
                                        k_s32 y,
                                        k_s32 width,
                                        k_s32 height,
                                        k_u32 color)
{
    k_s32 row;
    k_s32 col;

    for (row = 0; row < height; ++row)
    {
        for (col = 0; col < width; ++col)
        {
            motion_osd_set_pixel_locked(
                x + col,
                y + row,
                color);
        }
    }
}

static void motion_osd_draw_char_locked(char ch,
                                        k_s32 start_x,
                                        k_s32 start_y,
                                        k_s32 scale,
                                        k_u32 color)
{
    const k_u8 *glyph;
    k_s32 row;
    k_s32 col;
    k_s32 sy;
    k_s32 sx;

    if (ch == ' ')
        return;

    glyph = motion_osd_find_glyph(ch);
    if (!glyph)
        return;

    for (row = 0; row < 7; ++row)
    {
        for (col = 0; col < 5; ++col)
        {
            if ((glyph[row] & (1U << (4 - col))) == 0)
                continue;

            for (sy = 0; sy < scale; ++sy)
            {
                for (sx = 0; sx < scale; ++sx)
                {
                    motion_osd_set_pixel_locked(
                        start_x + col * scale + sx,
                        start_y + row * scale + sy,
                        color);
                }
            }
        }
    }
}

static void motion_osd_draw_text_locked(const char *text,
                                        k_s32 scale)
{
    k_s32 text_length;
    k_s32 cell_width;
    k_s32 text_width;
    k_s32 text_height;
    k_s32 start_x;
    k_s32 start_y;
    k_s32 i;

    if (!text)
        return;

    text_length = (k_s32)strlen(text);
    cell_width = 6 * scale;
    text_width = text_length * cell_width;
    text_height = 7 * scale;

    start_x =
        ((k_s32)g_motion_osd_width - text_width) / 2;

    start_y =
        ((k_s32)g_motion_osd_height - text_height) / 2;

    if (start_x < 4)
        start_x = 4;

    if (start_y < 4)
        start_y = 4;

    for (i = 0; i < text_length; ++i)
    {
        motion_osd_draw_char_locked(
            text[i],
            start_x + i * cell_width,
            start_y,
            scale,
            MOTION_OSD_TEXT_COLOR);
    }
}

static void motion_osd_render_alert_locked(void)
{
    k_s32 x;
    k_s32 y;

    motion_osd_clear_locked();

    /*
     * 半透明背景。
     */
    motion_osd_fill_rect_locked(
        0,
        0,
        (k_s32)g_motion_osd_width,
        (k_s32)g_motion_osd_height,
        MOTION_OSD_BG_COLOR);

    /*
     * 黄色边框。
     */
    for (x = 0; x < (k_s32)g_motion_osd_width; ++x)
    {
        motion_osd_set_pixel_locked(
            x,
            0,
            MOTION_OSD_BORDER_COLOR);

        motion_osd_set_pixel_locked(
            x,
            (k_s32)g_motion_osd_height - 1,
            MOTION_OSD_BORDER_COLOR);
    }

    for (y = 0; y < (k_s32)g_motion_osd_height; ++y)
    {
        motion_osd_set_pixel_locked(
            0,
            y,
            MOTION_OSD_BORDER_COLOR);

        motion_osd_set_pixel_locked(
            (k_s32)g_motion_osd_width - 1,
            y,
            MOTION_OSD_BORDER_COLOR);
    }

    /*
     * 320×64区域中，3倍5×7字体可以容纳完整英文提示。
     */
    motion_osd_draw_text_locked(
        "MOTION DETECTED!",
        3);
}

int motion_osd_bind(k_u64 phys_addr,
                    void *virt_addr,
                    k_u32 width,
                    k_u32 height,
                    k_u32 buffer_size)
{
    k_u64 required_size;

    if (phys_addr == 0 ||
        virt_addr == NULL ||
        width == 0 ||
        height == 0)
    {
        printf("motion_osd: invalid bind arguments\n");
        return -1;
    }

    required_size =
        (k_u64)width * (k_u64)height * sizeof(k_u32);

    if (required_size > buffer_size)
    {
        printf(
            "motion_osd: buffer too small, required=%lu size=%u\n",
            (unsigned long)required_size,
            buffer_size);
        return -1;
    }

    pthread_mutex_lock(&g_motion_osd_lock);

    g_motion_osd_phys_addr = phys_addr;
    g_motion_osd_pixels = (k_u32 *)virt_addr;
    g_motion_osd_width = width;
    g_motion_osd_height = height;
    g_motion_osd_buffer_size = buffer_size;
    g_motion_osd_hold_ticks = 0;
    g_motion_osd_bound = 1;

    /*
     * 初始状态完全透明。
     */
    motion_osd_clear_locked();
    motion_osd_flush_locked();

    pthread_mutex_unlock(&g_motion_osd_lock);

    printf(
        "motion_osd: bound, size=%ux%u phys=0x%lx\n",
        width,
        height,
        (unsigned long)phys_addr);

    return 0;
}

void motion_osd_trigger(void)
{
    pthread_mutex_lock(&g_motion_osd_lock);

    if (g_motion_osd_bound)
    {
        motion_osd_render_alert_locked();
        g_motion_osd_hold_ticks = MOTION_OSD_HOLD_TICKS;
        motion_osd_flush_locked();

        printf(
            "motion_osd: show MOTION DETECTED, hold_ticks=%d\n",
            g_motion_osd_hold_ticks);
    }

    pthread_mutex_unlock(&g_motion_osd_lock);
}

void motion_osd_tick(void)
{
    pthread_mutex_lock(&g_motion_osd_lock);

    if (g_motion_osd_bound &&
        g_motion_osd_hold_ticks > 0)
    {
        g_motion_osd_hold_ticks--;

        if (g_motion_osd_hold_ticks == 0)
        {
            motion_osd_clear_locked();
            motion_osd_flush_locked();

            printf("motion_osd: hidden\n");
        }
    }

    pthread_mutex_unlock(&g_motion_osd_lock);
}

void motion_osd_unbind(void)
{
    pthread_mutex_lock(&g_motion_osd_lock);

    if (g_motion_osd_bound)
    {
        motion_osd_clear_locked();
        motion_osd_flush_locked();
    }

    g_motion_osd_phys_addr = 0;
    g_motion_osd_pixels = NULL;
    g_motion_osd_width = 0;
    g_motion_osd_height = 0;
    g_motion_osd_buffer_size = 0;
    g_motion_osd_hold_ticks = 0;
    g_motion_osd_bound = 0;

    pthread_mutex_unlock(&g_motion_osd_lock);

    printf("motion_osd: unbound\n");
}
