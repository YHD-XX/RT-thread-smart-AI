#include "motion_ai_utils.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::F::k230;

namespace
{
    namespace hrt = nncase::runtime::host_runtime_tensor;

    static void copy_u8_to_runtime_tensor(runtime_tensor &tensor, const uint8_t *data, size_t bytes)
    {
        auto host = tensor.impl()->to_host().unwrap();
        auto mapped = host->buffer().as_host().unwrap().map(map_access_::map_write).unwrap();
        auto buf = mapped.buffer();

        std::memcpy(reinterpret_cast<void *>(buf.data()), data, bytes);

        hrt::sync(tensor, sync_op_::sync_write_back, true).expect("sync write_back failed");
    }
}

void Utils::bgr2rgb_and_hwc2chw(cv::Mat &ori_img, vector<uint8_t> &chw_vec)
{
    if (ori_img.empty())
    {
        chw_vec.clear();
        return;
    }

    cv::Mat rgb_img;
    cv::cvtColor(ori_img, rgb_img, cv::COLOR_BGR2RGB);

    vector<cv::Mat> channels;
    cv::split(rgb_img, channels);

    const size_t hw = static_cast<size_t>(rgb_img.rows) * static_cast<size_t>(rgb_img.cols);
    chw_vec.resize(hw * 3);

    std::memcpy(chw_vec.data(), channels[0].data, hw);
    std::memcpy(chw_vec.data() + hw, channels[1].data, hw);
    std::memcpy(chw_vec.data() + hw * 2, channels[2].data, hw);
}

void Utils::resize(FrameCHWSize input_shape, vector<uint8_t> &chw_vec, runtime_tensor &ai2d_out_tensor)
{
    /*
     * 当前 analyzer 输入是 BGR Mat，OBDet::pre_process 会先转成 RGB+CHW。
     * 这里不用 K230 ai2d，直接用 OpenCV 将 CHW 的 3 个通道 resize 到模型输入尺寸，
     * 然后写入 ai2d_out_tensor。
     */

    auto out_shape = ai2d_out_tensor.shape();

    if (out_shape.size() < 4)
    {
        throw std::runtime_error("invalid output tensor shape");
    }

    const int in_c = static_cast<int>(input_shape.channel);
    const int in_h = static_cast<int>(input_shape.height);
    const int in_w = static_cast<int>(input_shape.width);

    const int out_c = static_cast<int>(out_shape[1]);
    const int out_h = static_cast<int>(out_shape[2]);
    const int out_w = static_cast<int>(out_shape[3]);

    if (in_c != 3 || out_c != 3)
    {
        throw std::runtime_error("only 3-channel RGB CHW tensor is supported");
    }

    const size_t in_hw = static_cast<size_t>(in_h) * static_cast<size_t>(in_w);
    const size_t out_hw = static_cast<size_t>(out_h) * static_cast<size_t>(out_w);

    vector<uint8_t> out_chw(out_hw * 3);

    for (int c = 0; c < 3; ++c)
    {
        cv::Mat in_ch(in_h, in_w, CV_8UC1, chw_vec.data() + static_cast<size_t>(c) * in_hw);
        cv::Mat out_ch(out_h, out_w, CV_8UC1, out_chw.data() + static_cast<size_t>(c) * out_hw);

        cv::resize(in_ch, out_ch, cv::Size(out_w, out_h), 0, 0, cv::INTER_LINEAR);
    }

    copy_u8_to_runtime_tensor(ai2d_out_tensor, out_chw.data(), out_chw.size());
}

void Utils::resize(unique_ptr<ai2d_builder> &builder, runtime_tensor &ai2d_in_tensor, runtime_tensor &ai2d_out_tensor)
{
    if (!builder)
    {
        throw std::runtime_error("ai2d builder is null");
    }

    builder->invoke(ai2d_in_tensor, ai2d_out_tensor).expect("ai2d invoke failed");
}
