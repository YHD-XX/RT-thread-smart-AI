#ifndef __MOTION_LK_H__
#define __MOTION_LK_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化 LK 运动检测器。
 * 返回 0 表示成功，负数表示失败。
 */
int motion_lk_init(unsigned int width, unsigned int height);

/*
 * 输入一帧连续排列的 8 位灰度图。
 *
 * 返回值：
 *   1：产生一次新的运动事件
 *   0：本帧没有产生新事件
 *  -1：处理失败
 */
int motion_lk_process(const unsigned char *gray);

/* 释放内部 OpenCV 图像和状态。 */
void motion_lk_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
