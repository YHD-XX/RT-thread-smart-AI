#ifndef __MOTION_IPC_H__
#define __MOTION_IPC_H__

#include "k_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_IPC_WIDTH       320U
#define MOTION_IPC_HEIGHT      180U
#define MOTION_IPC_GRAY_SIZE   (MOTION_IPC_WIDTH * MOTION_IPC_HEIGHT)
#define MOTION_IPC_YUV_SIZE    (MOTION_IPC_WIDTH * MOTION_IPC_HEIGHT * 3U / 2U)

/* 主采集进程接口 */
int motion_ipc_producer_start(k_u32 width, k_u32 height);

int motion_ipc_publish_frame(const k_u8 *gray,
                             const k_u8 *yuv420sp,
                             k_u32 frame_number);

int motion_ipc_try_get_event(k_u8 *yuv420sp,
                             k_u32 yuv_capacity,
                             k_u32 *frame_number);

void motion_ipc_producer_stop(void);

/* 独立分析进程接口 */
int motion_ipc_consumer_connect(k_u32 timeout_ms);

int motion_ipc_consumer_get_info(k_u32 *width,
                                 k_u32 *height);

int motion_ipc_wait_frame(k_u8 *gray,
                          k_u32 gray_capacity,
                          k_u8 *yuv420sp,
                          k_u32 yuv_capacity,
                          k_u32 *frame_number);

int motion_ipc_publish_event(const k_u8 *yuv420sp,
                             k_u32 frame_number);

void motion_ipc_consumer_close(void);

#ifdef __cplusplus
}
#endif

#endif
