#include <cstdio>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "k_type.h"
#include "motion_vision.h"
#include "motion_ob_det.h"

static OBDet *g_motion_detector = nullptr;
static int g_motion_vision_debug = 0;
static k_u32 g_motion_vision_call_count = 0;


static int motion_vision_yuv420sp_to_bgr(const k_u8 *yuv420sp,
                                         k_u32 width,
                                         k_u32 height,
                                         cv::Mat &bgr)
{
    if (!yuv420sp ||
        width == 0 ||
        height == 0)
    {
        return -1;
    }

    /*
     * 当前先按 NV12 处理：
     *   Y plane + UVUVUV...
     *
     * 如果后面发现模型识别效果明显异常，或者 JPG 偏紫/偏绿，
     * 再切换成 COLOR_YUV2BGR_NV21。
     */
    cv::Mat yuv(
        height + height / 2U,
        width,
        CV_8UC1,
        const_cast<k_u8 *>(yuv420sp));

    cv::cvtColor(
        yuv,
        bgr,
        cv::COLOR_YUV2BGR_NV12);

    return 0;
}


int motion_vision_init(const char *kmodel_path,
                       float score_thres,
                       float nms_thres,
                       int debug_mode)
{
    if (!kmodel_path)
        return -1;

    if (g_motion_detector)
        return 0;

    try
    {
        g_motion_detector =
            new OBDet(
                kmodel_path,
                score_thres,
                nms_thres,
                debug_mode);
    }
    catch (...)
    {
        printf("motion_vision: OBDet init exception, "
               "kmodel=%s\n",
               kmodel_path);

        g_motion_detector = nullptr;
        return -1;
    }

    g_motion_vision_debug = debug_mode;
    g_motion_vision_call_count = 0;

    printf("motion_vision: initialized, kmodel=%s "
           "score=%.2f nms=%.2f debug=%d\n",
           kmodel_path,
           score_thres,
           nms_thres,
           debug_mode);

    return 0;
}


int motion_vision_detect_yuv420sp(const k_u8 *yuv420sp,
                                  k_u32 width,
                                  k_u32 height,
                                  motion_vision_result_t *result)
{
    cv::Mat bgr;
    std::vector<Detection> detections;

    float best_conf = 0.0f;
    int best_index = -1;

    if (!g_motion_detector ||
        !yuv420sp ||
        !result)
    {
        return -1;
    }

    result->has_person = K_FALSE;
    result->confidence = 0.0f;
    result->class_id = -1;
    result->x = 0;
    result->y = 0;
    result->w = 0;
    result->h = 0;

    if (motion_vision_yuv420sp_to_bgr(
            yuv420sp,
            width,
            height,
            bgr) != 0)
    {
        return -1;
    }

    try
    {
        g_motion_detector->pre_process(bgr);
        g_motion_detector->inference();
        g_motion_detector->post_process(
            {width, height},
            detections);
    }
    catch (...)
    {
        printf("motion_vision: detect exception\n");
        return -1;
    }

    g_motion_vision_call_count++;

    for (size_t i = 0;
         i < detections.size();
         ++i)
    {
        /*
         * YOLO COCO class 0 = person
         */
        if (detections[i].class_id == 0 &&
            detections[i].confidence > best_conf)
        {
            best_conf = detections[i].confidence;
            best_index = (int)i;
        }
    }

    if (best_index >= 0)
    {
        const Detection &det = detections[best_index];

        result->has_person = K_TRUE;
        result->confidence = det.confidence;
        result->class_id = det.class_id;
        result->x = det.box.x;
        result->y = det.box.y;
        result->w = det.box.width;
        result->h = det.box.height;

        printf("motion_vision: PERSON detected, "
               "conf=%.3f box=(%d,%d,%d,%d) "
               "detections=%u call=%u\n",
               result->confidence,
               result->x,
               result->y,
               result->w,
               result->h,
               (unsigned)detections.size(),
               g_motion_vision_call_count);
    }
    else if (g_motion_vision_debug > 0)
    {
        printf("motion_vision: no person, "
               "detections=%u call=%u\n",
               (unsigned)detections.size(),
               g_motion_vision_call_count);
    }

    return 0;
}


void motion_vision_deinit(void)
{
    if (!g_motion_detector)
        return;

    delete g_motion_detector;
    g_motion_detector = nullptr;

    printf("motion_vision: deinitialized\n");
}
