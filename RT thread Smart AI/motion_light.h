#ifndef __MOTION_LIGHT_H__
#define __MOTION_LIGHT_H__

#include "k_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    k_bool light_changed;
    k_bool in_light_change;

    k_u32 frame_number;
    k_u32 mean_y;
    k_u32 prev_mean_y;
    k_u32 mean_diff;

    /*
     * 千分比：
     * 550 表示 55.0%
     */
    k_u32 changed_ratio_permille;
    k_u32 dominant_ratio_permille;

    k_u32 hold_frames_left;

} motion_light_result_t;


int motion_light_init(k_u32 width,
                      k_u32 height);

int motion_light_update(const k_u8 *gray,
                        k_u32 frame_number,
                        motion_light_result_t *result);

void motion_light_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
