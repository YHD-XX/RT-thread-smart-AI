#ifndef __MOTION_VISION_H__
#define __MOTION_VISION_H__

#include "k_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    k_bool has_person;
    float confidence;
    int class_id;
    int x;
    int y;
    int w;
    int h;
} motion_vision_result_t;

int motion_vision_init(const char *kmodel_path,
                       float score_thres,
                       float nms_thres,
                       int debug_mode);

int motion_vision_detect_yuv420sp(const k_u8 *yuv420sp,
                                  k_u32 width,
                                  k_u32 height,
                                  motion_vision_result_t *result);

void motion_vision_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
