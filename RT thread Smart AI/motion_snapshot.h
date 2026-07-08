#ifndef __MOTION_SNAPSHOT_H__
#define __MOTION_SNAPSHOT_H__

#include "k_type.h"

#ifdef __cplusplus
extern "C" {
#endif

int motion_snapshot_start(void);

/*
 * 输入为 320x180 YUV420SP/NV12 彩色帧。
 * 保存为 /sharefs/snapshot_jpg/motion_xxxx.jpg。
 */
int motion_snapshot_request(const k_u8 *yuv420sp,
                            k_u32 width,
                            k_u32 height,
                            k_u32 frame_number);

void motion_snapshot_stop(void);

#ifdef __cplusplus
}
#endif

#endif
