#ifndef __MOTION_OSD_H__
#define __MOTION_OSD_H__

#include "k_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 将动态 OSD 模块绑定到 sample_venc 已经申请并映射的
 * ARGB8888 VB 缓冲区。
 */
int motion_osd_bind(k_u64 phys_addr,
                    void *virt_addr,
                    k_u32 width,
                    k_u32 height,
                    k_u32 buffer_size);

/* 收到运动事件后显示提示。 */
void motion_osd_trigger(void);

/*
 * 每取得一张有效分析帧时调用一次。
 * 约15次后自动隐藏，对应约1.5秒。
 */
void motion_osd_tick(void);

/* 清空提示并解除绑定。 */
void motion_osd_unbind(void);

#ifdef __cplusplus
}
#endif

#endif
