#ifndef __MOTION_AI_UTILS_H__
#define __MOTION_AI_UTILS_H__

#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <random>
#include <stdint.h>
#include <string.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <nncase/functional/ai2d/ai2d_builder.h>

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::k230;
using namespace nncase::F::k230;

using namespace std;
using namespace cv;

typedef struct FrameSize
{
    size_t width;
    size_t height;
} FrameSize;

typedef struct FrameCHWSize
{
    size_t channel;
    size_t height;
    size_t width;
} FrameCHWSize;

struct Detection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};
};

typedef struct {
    Rect box;
    float confidence;
    int index;
} BBOX;

class Utils
{
public:
    static void bgr2rgb_and_hwc2chw(cv::Mat &ori_img,
                                    std::vector<uint8_t> &chw_vec);

    static void resize(FrameCHWSize ori_shape,
                       std::vector<uint8_t> &chw_vec,
                       runtime_tensor &ai2d_out_tensor);

    static void resize(std::unique_ptr<ai2d_builder> &builder,
                       runtime_tensor &ai2d_in_tensor,
                       runtime_tensor &ai2d_out_tensor);
};

#endif
