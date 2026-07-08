#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "k_type.h"
#include "k_video_comm.h"
#include "mpi_vicap_api.h"
#include "mpi_sys_api.h"

#include "motion_frame.h"
#include "motion_ipc.h"
#include "motion_osd.h"
#include "motion_snapshot.h"

#define MOTION_DUMP_TIMEOUT_MS      1000
#define MOTION_SAMPLE_INTERVAL_US   100000
#define MOTION_DEBUG_PGM_PATH       "/sharefs/ai_debug_stage2.pgm"

static pthread_t g_motion_frame_tid;
static volatile k_bool g_motion_frame_running = K_FALSE;
static k_bool g_motion_frame_started = K_FALSE;

static k_u8 g_motion_gray[MOTION_AI_GRAY_SIZE];
static k_u8 g_motion_yuv[MOTION_AI_YUV_SIZE];
static k_u8 g_motion_event_yuv[MOTION_AI_YUV_SIZE];


static int motion_save_pgm(const char *path,
                           const k_u8 *gray,
                           k_u32 width,
                           k_u32 height)
{
    FILE *fp;

    if (!path || !gray || width == 0 || height == 0)
        return -1;

    fp = fopen(path, "wb");

    if (!fp)
    {
        printf("motion_frame: fopen %s failed\n", path);
        return -1;
    }

    fprintf(fp, "P5\n%u %u\n255\n", width, height);

    if (fwrite(gray, 1, width * height, fp) != width * height)
    {
        printf("motion_frame: fwrite %s failed\n", path);
        fclose(fp);
        return -1;
    }

    fflush(fp);
    fclose(fp);

    printf("motion_frame: saved %s\n", path);
    return 0;
}


static void *motion_frame_thread(void *arg)
{
    k_s32 ret;
    k_video_frame_info dump_info;

    k_u32 valid_frame_count = 0;
    k_u32 mismatch_count = 0;

    k_bool file_saved = K_FALSE;
    k_bool ipc_ready = K_FALSE;

    k_s32 ipc_event_ret = 0;
    k_u32 ipc_event_frame = 0;

    (void)arg;

    printf("motion_frame: thread started, target=%ux%u\n",
           MOTION_AI_WIDTH,
           MOTION_AI_HEIGHT);

    if (motion_snapshot_start() != 0)
    {
        printf("motion_frame: motion_snapshot_start failed; "
               "motion detection will continue without snapshots\n");
    }

    if (motion_ipc_producer_start(
            MOTION_AI_WIDTH,
            MOTION_AI_HEIGHT) == 0)
    {
        ipc_ready = K_TRUE;
    }
    else
    {
        printf("motion_frame: IPC producer start failed; "
               "video will continue without external analysis\n");
    }

    while (g_motion_frame_running)
    {
        k_u32 width;
        k_u32 height;
        k_u32 stride;
        k_u32 map_size;
        k_u8 *y_addr;
        k_u32 row;
        k_bool frame_ready = K_FALSE;

        memset(&dump_info, 0, sizeof(dump_info));

        ret = kd_mpi_vicap_dump_frame(
            VICAP_DEV_ID_0,
            VICAP_CHN_ID_1,
            VICAP_DUMP_YUV,
            &dump_info,
            MOTION_DUMP_TIMEOUT_MS);

        if (ret != K_SUCCESS)
        {
            if (g_motion_frame_running)
            {
                printf("motion_frame: dump chn1 failed, ret=%d\n",
                       ret);
            }

            usleep(30000);
            continue;
        }

        width = dump_info.v_frame.width;
        height = dump_info.v_frame.height;
        stride = dump_info.v_frame.stride[0];

        if (stride < width)
            stride = width;

        if (valid_frame_count == 0 &&
            mismatch_count == 0)
        {
            printf("motion_frame: first frame width=%u "
                   "height=%u stride=%u phys=0x%lx\n",
                   width,
                   height,
                   stride,
                   dump_info.v_frame.phys_addr[0]);
        }

        if (width != MOTION_AI_WIDTH ||
            height != MOTION_AI_HEIGHT)
        {
            if (mismatch_count < 10)
            {
                printf("motion_frame: unexpected frame size %ux%u, "
                       "expected %ux%u\n",
                       width,
                       height,
                       MOTION_AI_WIDTH,
                       MOTION_AI_HEIGHT);
            }

            mismatch_count++;

            kd_mpi_vicap_dump_release(
                VICAP_DEV_ID_0,
                VICAP_CHN_ID_1,
                &dump_info);

            usleep(MOTION_SAMPLE_INTERVAL_US);
            continue;
        }

        /*
         * YUV420SP/NV12:
         *   Y  plane: stride * height
         *   UV plane: stride * height / 2
         *
         * Stage 2~6 只映射 Y 平面；
         * Stage 7 需要保存彩色 JPG，因此必须映射完整 YUV420SP。
         */
        map_size = stride * height * 3U / 2U;

        y_addr = (k_u8 *)kd_mpi_sys_mmap(
            dump_info.v_frame.phys_addr[0],
            map_size);

        if (!y_addr)
        {
            printf("motion_frame: mmap failed, phys=0x%lx size=%u\n",
                   dump_info.v_frame.phys_addr[0],
                   map_size);
        }
        else
        {
            /*
             * 复制 Y 平面：
             *   g_motion_gray 给 LK 光流使用；
             *   g_motion_yuv 的前 width*height 字节也保存 Y。
             */
            for (row = 0;
                 row < MOTION_AI_HEIGHT;
                 ++row)
            {
                memcpy(
                    g_motion_gray + row * MOTION_AI_WIDTH,
                    y_addr + row * stride,
                    MOTION_AI_WIDTH);

                memcpy(
                    g_motion_yuv + row * MOTION_AI_WIDTH,
                    y_addr + row * stride,
                    MOTION_AI_WIDTH);
            }

            /*
             * 复制 UV 交织平面。
             * 这里按 NV12/YUV420SP 处理：UVUVUV...
             */
            for (row = 0;
                 row < MOTION_AI_HEIGHT / 2U;
                 ++row)
            {
                memcpy(
                    g_motion_yuv
                        + MOTION_AI_WIDTH * MOTION_AI_HEIGHT
                        + row * MOTION_AI_WIDTH,
                    y_addr
                        + stride * MOTION_AI_HEIGHT
                        + row * stride,
                    MOTION_AI_WIDTH);
            }

            kd_mpi_sys_munmap(
                y_addr,
                map_size);

            valid_frame_count++;
            frame_ready = K_TRUE;
        }

        ret = kd_mpi_vicap_dump_release(
            VICAP_DEV_ID_0,
            VICAP_CHN_ID_1,
            &dump_info);

        if (ret != K_SUCCESS)
        {
            printf("motion_frame: dump release failed, ret=%d\n",
                   ret);
        }

        /*
         * 图像已经复制到本地 buffer，且 VICAP dump 帧已经释放。
         * 后续 IPC、OSD、快照不会长期占用采集缓冲。
         */
        if (frame_ready)
        {
            if (!file_saved &&
                valid_frame_count >= 30)
            {
                if (motion_save_pgm(
                        MOTION_DEBUG_PGM_PATH,
                        g_motion_gray,
                        MOTION_AI_WIDTH,
                        MOTION_AI_HEIGHT) == 0)
                {
                    file_saved = K_TRUE;
                }
            }

            /*
             * 动态 OSD 使用分析帧计数作为显示计时。
             * 当前分析频率约 10fps。
             */
            motion_osd_tick();

            if (ipc_ready)
            {
                if (motion_ipc_publish_frame(
                        g_motion_gray,
                        g_motion_yuv,
                        valid_frame_count) != 0)
                {
                    printf("motion_frame: IPC frame publish failed, "
                           "frame=%u\n",
                           valid_frame_count);
                }

                do
                {
                    ipc_event_ret =
                        motion_ipc_try_get_event(
                            g_motion_event_yuv,
                            sizeof(g_motion_event_yuv),
                            &ipc_event_frame);

                    if (ipc_event_ret > 0)
                    {
                        printf("motion_frame: IPC MOTION DETECTED, "
                               "event_frame=%u current_frame=%u\n",
                               ipc_event_frame,
                               valid_frame_count);

                        motion_osd_trigger();

                        if (motion_snapshot_request(
                                g_motion_event_yuv,
                                MOTION_AI_WIDTH,
                                MOTION_AI_HEIGHT,
                                ipc_event_frame) < 0)
                        {
                            printf("motion_frame: IPC snapshot "
                                   "request failed, frame=%u\n",
                                   ipc_event_frame);
                        }
                    }
                }
                while (ipc_event_ret > 0);

                if (ipc_event_ret < 0)
                {
                    printf("motion_frame: IPC event read failed\n");
                }
            }
        }

        /*
         * 摄像头主链路仍按原帧率运行。
         * 分析支路约每 100ms 取一帧，即约 10fps。
         */
        usleep(MOTION_SAMPLE_INTERVAL_US);
    }

    if (ipc_ready)
    {
        motion_ipc_producer_stop();
        ipc_ready = K_FALSE;
    }

    /*
     * 等待尚未写完的 JPG 快照完成后再退出。
     */
    motion_snapshot_stop();

    printf("motion_frame: thread stopped, valid_frames=%u\n",
           valid_frame_count);

    return NULL;
}


int motion_frame_start(void)
{
    int ret;

    if (g_motion_frame_started)
        return 0;

    g_motion_frame_running = K_TRUE;

    ret = pthread_create(
        &g_motion_frame_tid,
        NULL,
        motion_frame_thread,
        NULL);

    if (ret != 0)
    {
        g_motion_frame_running = K_FALSE;

        printf("motion_frame: pthread_create failed, ret=%d\n",
               ret);

        return -1;
    }

    g_motion_frame_started = K_TRUE;

    return 0;
}


void motion_frame_stop(void)
{
    if (!g_motion_frame_started)
        return;

    g_motion_frame_running = K_FALSE;

    /*
     * dump_frame 的超时为 1000ms，
     * 因此退出时最多可能等待约 1 秒。
     */
    pthread_join(
        g_motion_frame_tid,
        NULL);

    g_motion_frame_started = K_FALSE;
}
