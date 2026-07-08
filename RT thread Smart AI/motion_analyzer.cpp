#include <cstdio>

#include "k_type.h"

#include "motion_ipc.h"
#include "motion_lk.h"
#include "motion_light.h"
#include "motion_vision.h"

static k_u8 g_analyzer_gray[MOTION_IPC_GRAY_SIZE];
static k_u8 g_analyzer_yuv[MOTION_IPC_YUV_SIZE];

#define MOTION_VISION_KMODEL_PATH "/sharefs/app/yolov8n_320.kmodel"
#define MOTION_VISION_SCORE_THRES 0.35f
#define MOTION_VISION_NMS_THRES   0.45f
#define MOTION_VISION_DEBUG       0


int main(int argc, char **argv)
{
    k_u32 width = 0;
    k_u32 height = 0;
    k_u32 frame_number = 0;

    k_u32 processed_frames = 0;
    k_u32 published_events = 0;
    k_u32 light_hold_skipped = 0;

    int ret;
    int motion_event;
    int lk_ready = 0;
int vision_ready = 0;

    motion_light_result_t light_result;
motion_vision_result_t vision_result;

    (void)argc;
    (void)argv;

    printf("motion_analyzer: waiting for producer\n");

    /*
     * 允许先启动分析进程，再启动 sample_venc。
     * 最长等待 60 秒。
     */
    ret = motion_ipc_consumer_connect(60000);

    if (ret != 0)
    {
        printf("motion_analyzer: IPC connection failed\n");
        return 1;
    }

    if (motion_ipc_consumer_get_info(
            &width,
            &height) != 0)
    {
        printf("motion_analyzer: cannot read image geometry\n");
        motion_ipc_consumer_close();
        return 1;
    }

    if (motion_light_init(width, height) != 0)
    {
        printf("motion_analyzer: motion_light_init failed\n");
        motion_ipc_consumer_close();
        return 1;
    }

    if (motion_vision_init(
            MOTION_VISION_KMODEL_PATH,
            MOTION_VISION_SCORE_THRES,
            MOTION_VISION_NMS_THRES,
            MOTION_VISION_DEBUG) == 0)
    {
        vision_ready = 1;
    }
    else
    {
        printf("motion_analyzer: motion_vision_init failed; "
               "light-change mode will only suppress LK\n");
    }

    if (motion_lk_init(width, height) != 0)
    {
        printf("motion_analyzer: motion_lk_init failed\n");
        if (vision_ready)
        {
            motion_vision_deinit();
            vision_ready = 0;
        }
        motion_light_deinit();
        motion_ipc_consumer_close();
        return 1;
    }

    lk_ready = 1;

    printf("motion_analyzer: started, image=%ux%u\n",
           width,
           height);

    while (1)
    {
        ret = motion_ipc_wait_frame(
            g_analyzer_gray,
            sizeof(g_analyzer_gray),
            g_analyzer_yuv,
            sizeof(g_analyzer_yuv),
            &frame_number);

        if (ret == 0)
        {
            printf("motion_analyzer: producer requested stop\n");
            break;
        }

        if (ret < 0)
        {
            printf("motion_analyzer: wait frame failed\n");
            break;
        }

        processed_frames++;

        if (motion_light_update(
                g_analyzer_gray,
                frame_number,
                &light_result) != 0)
        {
            printf("motion_analyzer: light update failed, "
                   "frame=%u\n",
                   frame_number);
        }
        else
        {
            if (light_result.light_changed)
            {
                /*
                 * 整体光照突变时，LK 的亮度恒定假设被破坏。
                 * 这里立即重置 LK，避免光照突变后的第一帧
                 * 与突变前旧帧进行光流匹配。
                 */
                printf("motion_analyzer: light-change detected, "
                       "reset LK, frame=%u hold=%u\n",
                       frame_number,
                       light_result.hold_frames_left);

                if (lk_ready)
                {
                    motion_lk_deinit();
                    lk_ready = 0;
                }

                if (motion_lk_init(width, height) == 0)
                    lk_ready = 1;
                else
                    printf("motion_analyzer: LK reinit failed "
                           "after light change\n");
            }

            if (light_result.in_light_change)
            {
                light_hold_skipped++;

                /*
                 * Stage 7D:
                 *   光照突变保护期内不信任 LK。
                 *   如果视觉模型可用，则用 YOLOv8 / OBDet 判断是否有人。
                 */
                if (vision_ready)
                {
                    if (motion_vision_detect_yuv420sp(
                            g_analyzer_yuv,
                            width,
                            height,
                            &vision_result) == 0 &&
                        vision_result.has_person)
                    {
                        if (motion_ipc_publish_event(
                                g_analyzer_yuv,
                                frame_number) == 0)
                        {
                            published_events++;

                            printf("motion_analyzer: vision event "
                                   "published during light-change, "
                                   "frame=%u conf=%.3f total=%u\n",
                                   frame_number,
                                   vision_result.confidence,
                                   published_events);
                        }
                        else
                        {
                            printf("motion_analyzer: vision event "
                                   "publish failed, frame=%u\n",
                                   frame_number);
                        }
                    }
                    else
                    {
                        printf("motion_analyzer: light-change hold, "
                               "vision no-person, frame=%u "
                               "hold_left=%u mean_diff=%u "
                               "changed=%u/1000\n",
                               frame_number,
                               light_result.hold_frames_left,
                               light_result.mean_diff,
                               light_result.changed_ratio_permille);
                    }
                }
                else
                {
                    printf("motion_analyzer: light-change hold, "
                           "vision unavailable, skip LK, frame=%u "
                           "hold_left=%u mean_diff=%u "
                           "changed=%u/1000\n",
                           frame_number,
                           light_result.hold_frames_left,
                           light_result.mean_diff,
                           light_result.changed_ratio_permille);
                }

                continue;
            }
        }

        if (!lk_ready)
        {
            if (motion_lk_init(width, height) == 0)
                lk_ready = 1;
            else
            {
                printf("motion_analyzer: LK init failed "
                       "during normal mode\n");
                break;
            }
        }

        motion_event =
            motion_lk_process(g_analyzer_gray);

        if (motion_event > 0)
        {
            if (motion_ipc_publish_event(
                    g_analyzer_yuv,
                    frame_number) == 0)
            {
                published_events++;

                printf("motion_analyzer: event published, "
                       "frame=%u total=%u\n",
                       frame_number,
                       published_events);
            }
            else
            {
                printf("motion_analyzer: event publish failed, "
                       "frame=%u\n",
                       frame_number);
            }
        }
        else if (motion_event < 0)
        {
            printf("motion_analyzer: LK processing failed\n");
            break;
        }
    }

    if (lk_ready)
    {
        motion_lk_deinit();
        lk_ready = 0;
    }

    motion_light_deinit();
    motion_ipc_consumer_close();

    printf("motion_analyzer: stopped, "
           "processed=%u events=%u light_skip=%u\n",
           processed_frames,
           published_events,
           light_hold_skipped);

    return 0;
}
