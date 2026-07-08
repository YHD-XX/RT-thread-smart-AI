#ifndef __MOTION_FRAME_H__
#define __MOTION_FRAME_H__

#define MOTION_AI_WIDTH       320U
#define MOTION_AI_HEIGHT      180U
#define MOTION_AI_BUF_CNT     4U
#define MOTION_AI_GRAY_SIZE   (MOTION_AI_WIDTH * MOTION_AI_HEIGHT)
#define MOTION_AI_YUV_SIZE    (MOTION_AI_WIDTH * MOTION_AI_HEIGHT * 3U / 2U)

#define MOTION_AI_BUF_SIZE \
    (((MOTION_AI_WIDTH * MOTION_AI_HEIGHT * 3U / 2U) + 0xfffU) & ~0xfffU)

int motion_frame_start(void);
void motion_frame_stop(void);

#endif
