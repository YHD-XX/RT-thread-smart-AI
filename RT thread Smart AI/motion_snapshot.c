#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>

#include "jpeglib.h"

#include "k_type.h"
#include "motion_frame.h"
#include "motion_snapshot.h"

#define MOTION_SNAPSHOT_DIR           "/sharefs/snapshot_jpg"
#define MOTION_SNAPSHOT_PATH_SIZE     128
#define MOTION_SNAPSHOT_MAX_INDEX     9999U
#define MOTION_JPEG_QUALITY           85

static k_u8 g_snapshot_pending_yuv[MOTION_AI_YUV_SIZE];
static k_u8 g_snapshot_write_yuv[MOTION_AI_YUV_SIZE];

static pthread_t g_snapshot_tid;

static pthread_mutex_t g_snapshot_lock =
    PTHREAD_MUTEX_INITIALIZER;

static pthread_cond_t g_snapshot_cond =
    PTHREAD_COND_INITIALIZER;

static volatile k_bool g_snapshot_running = K_FALSE;
static k_bool g_snapshot_started = K_FALSE;
static k_bool g_snapshot_pending = K_FALSE;

static k_u32 g_snapshot_width = 0;
static k_u32 g_snapshot_height = 0;
static k_u32 g_snapshot_frame_number = 0;
static k_u32 g_snapshot_next_index = 1;

static k_u32 g_snapshot_saved_count = 0;
static k_u32 g_snapshot_failed_count = 0;
static k_u32 g_snapshot_dropped_count = 0;


static unsigned char motion_clip_int(int value)
{
    if (value < 0)
        return 0;

    if (value > 255)
        return 255;

    return (unsigned char)value;
}


static void motion_yuv420sp_to_rgb888(const k_u8 *yuv,
                                      k_u32 width,
                                      k_u32 height,
                                      unsigned char *rgb)
{
    k_u32 x;
    k_u32 y;
    k_u32 y_index;
    k_u32 uv_index;
    k_u32 rgb_index;

    int yy;
    int uu;
    int vv;
    int c;
    int d;
    int e;
    int r;
    int g;
    int b;

    const k_u8 *y_plane = yuv;
    const k_u8 *uv_plane = yuv + width * height;

    for (y = 0; y < height; ++y)
    {
        for (x = 0; x < width; ++x)
        {
            y_index = y * width + x;
            uv_index = (y / 2U) * width + (x & ~1U);

            yy = y_plane[y_index];
            uu = uv_plane[uv_index + 0] - 128;
            vv = uv_plane[uv_index + 1] - 128;

            c = yy - 16;
            d = uu;
            e = vv;

            if (c < 0)
                c = 0;

            r = (298 * c + 409 * e + 128) >> 8;
            g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            b = (298 * c + 516 * d + 128) >> 8;

            rgb_index = y_index * 3U;

            rgb[rgb_index + 0] = motion_clip_int(r);
            rgb[rgb_index + 1] = motion_clip_int(g);
            rgb[rgb_index + 2] = motion_clip_int(b);
        }
    }
}


static int motion_snapshot_prepare_directory(void)
{
    int ret;

    ret = mkdir(MOTION_SNAPSHOT_DIR, 0777);

    if (ret == 0)
        return 0;

    if (errno == EEXIST)
        return 0;

    printf("motion_snapshot: mkdir %s failed, errno=%d\n",
           MOTION_SNAPSHOT_DIR,
           errno);

    return -1;
}


static void motion_snapshot_make_path(char *path,
                                      k_u32 path_size,
                                      k_u32 index)
{
    snprintf(path,
             path_size,
             MOTION_SNAPSHOT_DIR "/motion_%04u.jpg",
             index);
}


static k_u32 motion_snapshot_find_next_index(void)
{
    char path[MOTION_SNAPSHOT_PATH_SIZE];
    FILE *fp;
    k_u32 index;

    for (index = 1;
         index <= MOTION_SNAPSHOT_MAX_INDEX;
         ++index)
    {
        motion_snapshot_make_path(
            path,
            sizeof(path),
            index);

        fp = fopen(path, "rb");

        if (!fp)
            return index;

        fclose(fp);
    }

    return 1;
}


static int motion_snapshot_save_jpeg(const char *path,
                                     const k_u8 *yuv420sp,
                                     k_u32 width,
                                     k_u32 height,
                                     int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE *fp;
    unsigned char *rgb;
    JSAMPROW row_pointer[1];
    int row_stride;

    if (!path ||
        !yuv420sp ||
        width == 0 ||
        height == 0)
    {
        return -1;
    }

    rgb = (unsigned char *)malloc(
        (size_t)width * (size_t)height * 3U);

    if (!rgb)
    {
        printf("motion_snapshot: malloc rgb failed\n");
        return -1;
    }

    motion_yuv420sp_to_rgb888(
        yuv420sp,
        width,
        height,
        rgb);

    fp = fopen(path, "wb");

    if (!fp)
    {
        printf("motion_snapshot: fopen %s failed, errno=%d\n",
               path,
               errno);

        free(rgb);
        return -1;
    }

    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, fp);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    row_stride = width * 3;

    while (cinfo.next_scanline < cinfo.image_height)
    {
        row_pointer[0] =
            &rgb[cinfo.next_scanline * row_stride];

        jpeg_write_scanlines(
            &cinfo,
            row_pointer,
            1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    fclose(fp);
    free(rgb);

    return 0;
}


static void *motion_snapshot_thread(void *arg)
{
    char path[MOTION_SNAPSHOT_PATH_SIZE];
    k_u32 width;
    k_u32 height;
    k_u32 frame_number;
    k_u32 snapshot_index;
    int ret;

    (void)arg;

    printf("motion_snapshot: jpg thread started, directory=%s\n",
           MOTION_SNAPSHOT_DIR);

    while (1)
    {
        pthread_mutex_lock(&g_snapshot_lock);

        while (g_snapshot_running &&
               !g_snapshot_pending)
        {
            pthread_cond_wait(
                &g_snapshot_cond,
                &g_snapshot_lock);
        }

        if (!g_snapshot_running &&
            !g_snapshot_pending)
        {
            pthread_mutex_unlock(&g_snapshot_lock);
            break;
        }

        width = g_snapshot_width;
        height = g_snapshot_height;
        frame_number = g_snapshot_frame_number;
        snapshot_index = g_snapshot_next_index++;

        memcpy(
            g_snapshot_write_yuv,
            g_snapshot_pending_yuv,
            MOTION_AI_YUV_SIZE);

        g_snapshot_pending = K_FALSE;

        pthread_mutex_unlock(&g_snapshot_lock);

        motion_snapshot_make_path(
            path,
            sizeof(path),
            snapshot_index);

        ret = motion_snapshot_save_jpeg(
            path,
            g_snapshot_write_yuv,
            width,
            height,
            MOTION_JPEG_QUALITY);

        if (ret == 0)
        {
            g_snapshot_saved_count++;

            printf("motion_snapshot: saved %s, frame=%u, "
                   "size=%ux%u quality=%d\n",
                   path,
                   frame_number,
                   width,
                   height,
                   MOTION_JPEG_QUALITY);
        }
        else
        {
            g_snapshot_failed_count++;

            printf("motion_snapshot: save jpg failed, "
                   "index=%u frame=%u\n",
                   snapshot_index,
                   frame_number);
        }
    }

    printf("motion_snapshot: jpg thread stopped\n");

    return NULL;
}


int motion_snapshot_start(void)
{
    int ret;

    if (g_snapshot_started)
        return 0;

    if (motion_snapshot_prepare_directory() != 0)
        return -1;

    g_snapshot_next_index =
        motion_snapshot_find_next_index();

    g_snapshot_pending = K_FALSE;
    g_snapshot_saved_count = 0;
    g_snapshot_failed_count = 0;
    g_snapshot_dropped_count = 0;

    g_snapshot_running = K_TRUE;

    ret = pthread_create(
        &g_snapshot_tid,
        NULL,
        motion_snapshot_thread,
        NULL);

    if (ret != 0)
    {
        g_snapshot_running = K_FALSE;

        printf("motion_snapshot: pthread_create failed, "
               "ret=%d\n",
               ret);

        return -1;
    }

    g_snapshot_started = K_TRUE;

    printf("motion_snapshot: jpg initialized, "
           "next_index=%u\n",
           g_snapshot_next_index);

    return 0;
}


int motion_snapshot_request(const k_u8 *yuv420sp,
                            k_u32 width,
                            k_u32 height,
                            k_u32 frame_number)
{
    if (!g_snapshot_started ||
        !g_snapshot_running)
    {
        return -1;
    }

    if (!yuv420sp ||
        width != MOTION_AI_WIDTH ||
        height != MOTION_AI_HEIGHT)
    {
        return -1;
    }

    pthread_mutex_lock(&g_snapshot_lock);

    if (g_snapshot_pending)
    {
        g_snapshot_dropped_count++;

        pthread_mutex_unlock(&g_snapshot_lock);

        printf("motion_snapshot: busy, drop frame=%u, "
               "dropped=%u\n",
               frame_number,
               g_snapshot_dropped_count);

        return 1;
    }

    memcpy(
        g_snapshot_pending_yuv,
        yuv420sp,
        MOTION_AI_YUV_SIZE);

    g_snapshot_width = width;
    g_snapshot_height = height;
    g_snapshot_frame_number = frame_number;
    g_snapshot_pending = K_TRUE;

    pthread_cond_signal(&g_snapshot_cond);
    pthread_mutex_unlock(&g_snapshot_lock);

    printf("motion_snapshot: jpg queued frame=%u\n",
           frame_number);

    return 0;
}


void motion_snapshot_stop(void)
{
    if (!g_snapshot_started)
        return;

    pthread_mutex_lock(&g_snapshot_lock);

    g_snapshot_running = K_FALSE;

    pthread_cond_signal(&g_snapshot_cond);

    pthread_mutex_unlock(&g_snapshot_lock);

    pthread_join(g_snapshot_tid, NULL);

    g_snapshot_started = K_FALSE;
    g_snapshot_pending = K_FALSE;

    printf("motion_snapshot: jpg deinitialized, "
           "saved=%u failed=%u dropped=%u\n",
           g_snapshot_saved_count,
           g_snapshot_failed_count,
           g_snapshot_dropped_count);
}
