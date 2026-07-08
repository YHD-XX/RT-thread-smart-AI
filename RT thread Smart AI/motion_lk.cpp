#include "motion_lk.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

/* 适用于 320×180、约 10fps 分析支路的初始参数。 */
static const int LK_MAX_CORNERS = 120;
static const int LK_MIN_VALID_POINTS = 12;
static const int LK_MIN_MOVING_POINTS = 8;

static const double LK_QUALITY_LEVEL = 0.01;
static const double LK_MIN_DISTANCE = 7.0;
static const double LK_MOVING_DISTANCE = 2.0;

/* 至少 25% 的有效角点发生明显位移。 */
static const int LK_MOTION_RATIO_PERCENT = 25;

/* 普通运动连续确认两次，降低单帧噪声误报。 */
static const int LK_CONFIRM_FRAMES = 2;

/* 10fps 下约 1 秒冷却时间。 */
static const int LK_EVENT_COOLDOWN_FRAMES = 10;

struct MotionLkState
{
    unsigned int width;
    unsigned int height;

    cv::Mat previous_gray;

    unsigned int frame_number;
    unsigned int event_number;

    int positive_streak;
    int cooldown_frames;
    bool initialized;
};

static MotionLkState g_lk = {
    0,
    0,
    cv::Mat(),
    0,
    0,
    0,
    0,
    false
};

static void motion_lk_reset_state(void)
{
    g_lk.previous_gray.release();

    g_lk.width = 0;
    g_lk.height = 0;
    g_lk.frame_number = 0;
    g_lk.event_number = 0;
    g_lk.positive_streak = 0;
    g_lk.cooldown_frames = 0;
    g_lk.initialized = false;
}

} /* namespace */

extern "C" int motion_lk_init(unsigned int width, unsigned int height)
{
    if (width == 0 || height == 0)
    {
        std::printf("motion_lk: invalid image size %ux%u\n",
                    width,
                    height);
        return -1;
    }

    motion_lk_reset_state();

    g_lk.width = width;
    g_lk.height = height;
    g_lk.initialized = true;

    std::printf("motion_lk: initialized, image=%ux%u, max_corners=%d\n",
                width,
                height,
                LK_MAX_CORNERS);

    return 0;
}

extern "C" int motion_lk_process(const unsigned char *gray)
{
    try
    {
        if (!g_lk.initialized || gray == NULL)
        {
            std::printf("motion_lk: process called before initialization\n");
            return -1;
        }

        /*
         * gray 指向 motion_frame.c 中会重复使用的缓冲区，
         * 因此当前帧必须复制到 OpenCV 自己管理的内存。
         */
        cv::Mat gray_view(
            static_cast<int>(g_lk.height),
            static_cast<int>(g_lk.width),
            CV_8UC1,
            const_cast<unsigned char *>(gray));

        cv::Mat current_gray = gray_view.clone();

        g_lk.frame_number++;

        if (g_lk.previous_gray.empty())
        {
            current_gray.copyTo(g_lk.previous_gray);

            std::printf("motion_lk: reference frame ready\n");
            return 0;
        }

        std::vector<cv::Point2f> previous_points;
        std::vector<cv::Point2f> current_points;
        std::vector<unsigned char> status;
        std::vector<float> error;

        cv::goodFeaturesToTrack(
            g_lk.previous_gray,
            previous_points,
            LK_MAX_CORNERS,
            LK_QUALITY_LEVEL,
            LK_MIN_DISTANCE,
            cv::Mat(),
            7,
            false,
            0.04);

        /*
         * 低纹理场景中可能找不到足够角点。
         * 此时只更新参考帧，不直接判定运动。
         */
        if (previous_points.size() < 8)
        {
            g_lk.positive_streak = 0;
            current_gray.copyTo(g_lk.previous_gray);

            if ((g_lk.frame_number % 10U) == 0U)
            {
                std::printf(
                    "motion_lk: frame=%u, too few corners=%u\n",
                    g_lk.frame_number,
                    static_cast<unsigned int>(previous_points.size()));
            }

            return 0;
        }

        cv::calcOpticalFlowPyrLK(
            g_lk.previous_gray,
            current_gray,
            previous_points,
            current_points,
            status,
            error,
            cv::Size(21, 21),
            3,
            cv::TermCriteria(
                cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                20,
                0.03),
            0,
            1e-4);

        int valid_points = 0;
        int moving_points = 0;
        double displacement_sum = 0.0;

        for (std::size_t i = 0;
             i < previous_points.size() && i < status.size();
             ++i)
        {
            if (!status[i])
                continue;

            const double dx =
                static_cast<double>(current_points[i].x -
                                    previous_points[i].x);

            const double dy =
                static_cast<double>(current_points[i].y -
                                    previous_points[i].y);

            const double displacement = std::sqrt(dx * dx + dy * dy);

            valid_points++;
            displacement_sum += displacement;

            if (displacement >= LK_MOVING_DISTANCE)
                moving_points++;
        }

        const int motion_ratio_percent =
            valid_points > 0
                ? moving_points * 100 / valid_points
                : 0;

        const int mean_displacement_x100 =
            valid_points > 0
                ? static_cast<int>(
                      displacement_sum * 100.0 /
                      static_cast<double>(valid_points))
                : 0;

        /*
         * 场景突然被遮挡时，部分角点可能直接跟踪失败。
         * 因此同时计算帧间平均灰度变化，作为 LK 失效时的辅助条件。
         */
        cv::Mat absolute_difference;
        cv::absdiff(
            g_lk.previous_gray,
            current_gray,
            absolute_difference);

        const double mean_difference =
            cv::mean(absolute_difference)[0];

        const int mean_difference_x10 =
            static_cast<int>(mean_difference * 10.0);

        const int total_corners =
            static_cast<int>(previous_points.size());

        const int lost_ratio_percent =
            total_corners > 0
                ? (total_corners - valid_points) * 100 / total_corners
                : 100;

        const bool normal_motion =
            valid_points >= LK_MIN_VALID_POINTS &&
            moving_points >= LK_MIN_MOVING_POINTS &&
            motion_ratio_percent >= LK_MOTION_RATIO_PERCENT &&
            mean_displacement_x100 >= 150;

        /*
         * 大面积遮挡、快速场景切换可能使大部分 LK 点丢失。
         * 只有“点大量丢失 + 帧间灰度变化较大”同时成立时，
         * 才认为是突发场景变化。
         */
        const bool abrupt_scene_change =
            total_corners >= 30 &&
            lost_ratio_percent >= 65 &&
            mean_difference_x10 >= 150;

        const bool candidate =
            normal_motion || abrupt_scene_change;

        if (candidate)
            g_lk.positive_streak++;
        else
            g_lk.positive_streak = 0;

        if (g_lk.cooldown_frames > 0)
            g_lk.cooldown_frames--;

        bool new_event = false;

        /*
         * 普通运动需要连续两帧；
         * 大面积突变可以立即触发一次事件。
         */
        if (g_lk.cooldown_frames == 0 &&
            (g_lk.positive_streak >= LK_CONFIRM_FRAMES ||
             abrupt_scene_change))
        {
            new_event = true;
            g_lk.event_number++;
            g_lk.cooldown_frames = LK_EVENT_COOLDOWN_FRAMES;
            g_lk.positive_streak = 0;
        }

        /*
         * 避免每帧打印占用串口：
         * 每 10 帧打印一次，候选运动和新事件立即打印。
         */
        if ((g_lk.frame_number % 10U) == 0U ||
            candidate ||
            new_event)
        {
            std::printf(
                "motion_lk: frame=%u corners=%d valid=%d "
                "moving=%d ratio=%d%% mean=%d.%02d "
                "diff=%d.%d lost=%d%% candidate=%d event=%d\n",
                g_lk.frame_number,
                total_corners,
                valid_points,
                moving_points,
                motion_ratio_percent,
                mean_displacement_x100 / 100,
                mean_displacement_x100 % 100,
                mean_difference_x10 / 10,
                mean_difference_x10 % 10,
                lost_ratio_percent,
                candidate ? 1 : 0,
                new_event ? 1 : 0);
        }

        if (new_event)
        {
            std::printf(
                "motion_lk: MOTION EVENT #%u at frame=%u\n",
                g_lk.event_number,
                g_lk.frame_number);
        }

        current_gray.copyTo(g_lk.previous_gray);

        return new_event ? 1 : 0;
    }
    catch (const cv::Exception &exception)
    {
        std::printf(
            "motion_lk: OpenCV exception: %s\n",
            exception.what());

        return -1;
    }
    catch (...)
    {
        std::printf("motion_lk: unknown exception\n");
        return -1;
    }
}

extern "C" void motion_lk_deinit(void)
{
    if (g_lk.initialized)
    {
        std::printf(
            "motion_lk: deinitialized, frames=%u events=%u\n",
            g_lk.frame_number,
            g_lk.event_number);
    }

    motion_lk_reset_state();
}
