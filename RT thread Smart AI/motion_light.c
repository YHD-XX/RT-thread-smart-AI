#include <stdio.h>
#include <string.h>

#include "k_type.h"
#include "motion_ipc.h"
#include "motion_light.h"

/*
 * 整体光照突变判定参数。
 *
 * mean_diff:
 *   当前帧平均亮度与上一帧平均亮度差值。
 *
 * changed_ratio:
 *   全图中 abs(curr - prev) > pixel_diff_threshold 的像素比例。
 *
 * dominant_ratio:
 *   全图中同方向变亮或同方向变暗的最大比例。
 *
 * 只有“平均亮度明显变化 + 大面积像素同步变化”同时满足，
 * 才认为是整体光照突变。
 */
#define MOTION_LIGHT_MEAN_DIFF_THRESHOLD        18U
#define MOTION_LIGHT_PIXEL_DIFF_THRESHOLD       25U
#define MOTION_LIGHT_CHANGED_RATIO_PERMILLE     550U
#define MOTION_LIGHT_DOMINANT_RATIO_PERMILLE    450U

/*
 * 约 10fps 分析频率下，15 帧约 1.5 秒。
 * 这段时间先暂时屏蔽 LK，后续 Stage 7D 改为视觉模型识别为主。
 */
#define MOTION_LIGHT_HOLD_FRAMES                15U

static k_bool g_light_initialized = K_FALSE;
static k_bool g_light_has_prev = K_FALSE;

static k_u32 g_light_width = 0;
static k_u32 g_light_height = 0;
static k_u32 g_light_pixels = 0;

static k_u32 g_light_prev_mean = 0;
static k_u32 g_light_hold_frames = 0;

static k_u8 g_light_prev_gray[MOTION_IPC_GRAY_SIZE];


static int motion_light_abs_int(int value)
{
    return value < 0 ? -value : value;
}


int motion_light_init(k_u32 width,
                      k_u32 height)
{
    if (width != MOTION_IPC_WIDTH ||
        height != MOTION_IPC_HEIGHT)
    {
        printf("motion_light: unsupported size=%ux%u, "
               "expected=%ux%u\n",
               width,
               height,
               MOTION_IPC_WIDTH,
               MOTION_IPC_HEIGHT);

        return -1;
    }

    memset(g_light_prev_gray,
           0,
           sizeof(g_light_prev_gray));

    g_light_width = width;
    g_light_height = height;
    g_light_pixels = width * height;

    g_light_prev_mean = 0;
    g_light_hold_frames = 0;

    g_light_has_prev = K_FALSE;
    g_light_initialized = K_TRUE;

    printf("motion_light: initialized, size=%ux%u, "
           "mean_diff_th=%u pixel_diff_th=%u "
           "changed_ratio_th=%u dominant_ratio_th=%u "
           "hold=%u\n",
           width,
           height,
           MOTION_LIGHT_MEAN_DIFF_THRESHOLD,
           MOTION_LIGHT_PIXEL_DIFF_THRESHOLD,
           MOTION_LIGHT_CHANGED_RATIO_PERMILLE,
           MOTION_LIGHT_DOMINANT_RATIO_PERMILLE,
           MOTION_LIGHT_HOLD_FRAMES);

    return 0;
}


int motion_light_update(const k_u8 *gray,
                        k_u32 frame_number,
                        motion_light_result_t *result)
{
    unsigned long long sum = 0;

    k_u32 index;
    k_u32 curr_mean;
    k_u32 mean_diff = 0;

    k_u32 changed_count = 0;
    k_u32 brighter_count = 0;
    k_u32 darker_count = 0;
    k_u32 dominant_count = 0;

    k_u32 changed_ratio_permille = 0;
    k_u32 dominant_ratio_permille = 0;

    k_bool light_changed = K_FALSE;
    k_bool in_light_change = K_FALSE;

    int diff;

    if (!g_light_initialized ||
        !gray ||
        !result)
    {
        return -1;
    }

    memset(result,
           0,
           sizeof(*result));

    for (index = 0;
         index < g_light_pixels;
         ++index)
    {
        sum += gray[index];
    }

    curr_mean = (k_u32)(sum / g_light_pixels);

    if (!g_light_has_prev)
    {
        memcpy(g_light_prev_gray,
               gray,
               g_light_pixels);

        g_light_prev_mean = curr_mean;
        g_light_has_prev = K_TRUE;

        result->frame_number = frame_number;
        result->mean_y = curr_mean;
        result->prev_mean_y = curr_mean;
        result->mean_diff = 0;
        result->light_changed = K_FALSE;
        result->in_light_change = K_FALSE;
        result->hold_frames_left = 0;

        return 0;
    }

    mean_diff =
        (curr_mean > g_light_prev_mean)
            ? (curr_mean - g_light_prev_mean)
            : (g_light_prev_mean - curr_mean);

    for (index = 0;
         index < g_light_pixels;
         ++index)
    {
        diff = (int)gray[index] -
               (int)g_light_prev_gray[index];

        if (motion_light_abs_int(diff) >
            (int)MOTION_LIGHT_PIXEL_DIFF_THRESHOLD)
        {
            changed_count++;

            if (diff > 0)
                brighter_count++;
            else
                darker_count++;
        }
    }

    dominant_count =
        brighter_count > darker_count
            ? brighter_count
            : darker_count;

    changed_ratio_permille =
        changed_count * 1000U / g_light_pixels;

    dominant_ratio_permille =
        dominant_count * 1000U / g_light_pixels;

    if (mean_diff >= MOTION_LIGHT_MEAN_DIFF_THRESHOLD &&
        changed_ratio_permille >=
            MOTION_LIGHT_CHANGED_RATIO_PERMILLE &&
        dominant_ratio_permille >=
            MOTION_LIGHT_DOMINANT_RATIO_PERMILLE)
    {
        light_changed = K_TRUE;
        g_light_hold_frames = MOTION_LIGHT_HOLD_FRAMES;

        printf("motion_light: LIGHT CHANGE frame=%u, "
               "mean=%u prev_mean=%u mean_diff=%u, "
               "changed=%u/1000 dominant=%u/1000, "
               "hold=%u\n",
               frame_number,
               curr_mean,
               g_light_prev_mean,
               mean_diff,
               changed_ratio_permille,
               dominant_ratio_permille,
               g_light_hold_frames);
    }

    in_light_change =
        g_light_hold_frames > 0 ? K_TRUE : K_FALSE;

    result->frame_number = frame_number;
    result->mean_y = curr_mean;
    result->prev_mean_y = g_light_prev_mean;
    result->mean_diff = mean_diff;
    result->changed_ratio_permille =
        changed_ratio_permille;
    result->dominant_ratio_permille =
        dominant_ratio_permille;
    result->light_changed = light_changed;
    result->in_light_change = in_light_change;
    result->hold_frames_left = g_light_hold_frames;

    memcpy(g_light_prev_gray,
           gray,
           g_light_pixels);

    g_light_prev_mean = curr_mean;

    if (g_light_hold_frames > 0)
        g_light_hold_frames--;

    return 0;
}


void motion_light_deinit(void)
{
    if (!g_light_initialized)
        return;

    g_light_initialized = K_FALSE;
    g_light_has_prev = K_FALSE;
    g_light_hold_frames = 0;

    printf("motion_light: deinitialized\n");
}
