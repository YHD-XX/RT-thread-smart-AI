/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include "k_module.h"
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_video_comm.h"
#include "k_sys_comm.h"
#include "mpi_vb_api.h"
#include "mpi_venc_api.h"
#include "mpi_sys_api.h"
#include "k_venc_comm.h"
#include "mpi_vvi_api.h"
#include "k_datafifo.h"
#include "motion_frame.h"
#include "motion_osd.h"

#define VENC_MAX_IN_FRAMES   30
#define ENABLE_VENC_DEBUG    1

//#define ENABLE_VDSS 1
#ifdef ENABLE_VDSS
    #include "k_vdss_comm.h"
    #include "mpi_vdss_api.h"
#else
    #include "mpi_vicap_api.h"
#endif

#ifdef ENABLE_VENC_DEBUG
    #define venc_debug  printf
#else
    #define venc_debug(ARGS...)
#endif

#define MAX_WIDTH 1920
#define MAX_HEIGHT 1080
#define STREAM_BUF_SIZE ((MAX_WIDTH*MAX_HEIGHT/2 + 0xfff) & ~0xfff)
#define FRAME_BUF_SIZE ((MAX_WIDTH*MAX_HEIGHT*2 + 0xfff) & ~0xfff)
#define OSD_MAX_WIDTH 320
#define OSD_MAX_HEIGHT 64
#define OSD_BUF_SIZE (OSD_MAX_WIDTH * OSD_MAX_HEIGHT * 4)
#define INPUT_BUF_CNT   6
#define OUTPUT_BUF_CNT  15
#define OSD_BUF_CNT     20

extern const unsigned int osd_data;
extern const int osd_data_size;

#define VI_ALIGN_UP(addr, size) (((addr)+((size)-1U))&(~((size)-1U)))

typedef enum
{
    VENC_SAMPLE_STATUS_IDLE = 0,
    VENC_SAMPLE_STATUS_INIT,
    VENC_SAMPLE_STATUS_START,
    VENC_SAMPLE_STATUS_BINDED,
    VENC_SAMPLE_STATUS_UNBINDED,
    VENC_SAMPLE_STATUE_RUNING,
    VENC_SAMPLE_STATUS_STOPED,
    VENC_SAMPLE_STATUS_BUTT
} VENC_SAMPLE_STATUS;

typedef struct
{
    k_u32 osd_width;
    k_u32 osd_height;
    k_u32 osd_phys_addr[VENC_MAX_IN_FRAMES][3];
    void *osd_virt_addr[VENC_MAX_IN_FRAMES][3];
    k_u32 osd_startx;
    k_u32 osd_starty;
    k_venc_2d_src_dst_fmt video_fmt;
    k_venc_2d_osd_fmt osd_fmt;
    k_u16 bg_alpha;
    k_u16 osd_alpha;
    k_u16 video_alpha;
    k_venc_2d_add_order add_order;
    k_u32 bg_color;
    k_u16 osd_coef[K_VENC_2D_COEF_NUM];
    k_u8 osd_region_num;
    k_bool osd_matrix_en;
} osd_conf_t;

typedef struct
{
    k_u16 width;
    k_u16 height;
    k_u16 line_width;
    k_u16 startx;
    k_u16 starty;
} border_conf_t;

typedef struct
{
    k_u32 ch_id;
    k_u32 output_frames;
} output_info;

typedef struct
{
    k_u32 chnum;
    pthread_t output_tid;
    k_bool osd_enable;
    osd_conf_t *osd_conf;
    k_vb_blk_handle osd_blk_handle;
    k_bool ch_done;
} venc_conf_t;

static char out_filename[50] = {"\0"};
static FILE *output_file = NULL;
static VENC_SAMPLE_STATUS g_venc_sample_status = VENC_SAMPLE_STATUS_IDLE;
static venc_conf_t g_venc_conf;
static k_u32 intbuf_size=0;
static k_u32 enc_width=1280;
static k_u32 enc_height=720;

/*
 * DataFIFO + RTSP bridge
 *
 * Big core:
 *   VENC H.264/H.265 packet -> DataFIFO writer
 *
 * Little core:
 *   ./rtspServer -p <PhyAddr> -t h264
 *   VLC: rtsp://<little_core_ip>:8554/BackChannelTest
 *
 * The little-core rtspServer expects every DataFIFO block to be:
 *   [k_u64 pts][k_u32 len][encoded stream bytes]
 */
#define READER_INDEX    0
#define WRITER_INDEX    1
#define DATAFIFO_BLOCK_LEN 1024000

static k_datafifo_handle hDataFifo[2] = {
    (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE,
    (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE
};

static k_char *g_datafifo_buf = NULL;
static k_bool g_datafifo_inited = K_FALSE;
static k_bool g_datafifo_enable = K_TRUE;
static const char *g_rtsp_codec_type = "h264";
static k_bool g_motion_analysis_enable = K_FALSE;

/*
 * RTSP compatibility fix:
 * Some RTSP clients, especially VLC, may connect after the encoder has already
 * emitted the initial H.264/H.265 parameter-set header. In that case the client
 * can connect successfully but cannot decode any image. Cache K_VENC_HEADER and
 * resend it immediately before every detected IDR frame.
 *
 * Do not depend on kd_mpi_venc_request_idr(), because some K230 VENC channels
 * report "idr should be enabled" and reject manual IDR requests.
 */
#define VENC_HEADER_CACHE_MAX    (256 * 1024)
static k_u8  g_venc_header_cache[VENC_HEADER_CACHE_MAX];
static k_u32 g_venc_header_cache_len = 0;
static k_u32 g_rtsp_header_resend_count = 0;

static inline void CHECK_RET(k_s32 ret, const char *func, const int line)
{
    if (ret)
        printf("error ret %d, func %s line %d\n", ret, func, line);
}

static void sample_vicap_config(k_u32 ch, k_u32 width, k_u32 height, k_vicap_sensor_type sensor_type)
{
#ifdef ENABLE_VDSS
    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;

    mpi_vdss_rst_all(2);

    memset(&dev_attr, 0, sizeof(dev_attr));
    dev_attr.dev_num = ch;
    dev_attr.height = height;
    dev_attr.width = width;
    dev_attr.sensor_type = 1;

    dev_attr.artr.csi = CSI0;
    dev_attr.artr.type = CLOSE_3D_MODE;
    dev_attr.artr.mode = LINERA_MODE;
    dev_attr.artr.dev_format[0] = RAW10;
    dev_attr.artr.phy_attr.lan_num = MIPI_1LAN;
    dev_attr.artr.phy_attr.freq = MIPI_800M;
    dev_attr.artr.bind_dvp = DVP_CSI1_FLASE_TRIGGER0;

    kd_mpi_vdss_set_dev_attr(&dev_attr);

    memset(&chn_attr, 0, sizeof(chn_attr));
    chn_attr.format = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    chn_attr.height = height;
    chn_attr.width = width;
    chn_attr.enable = 1;

    kd_mpi_vdss_set_chn_attr(ch, ch, &chn_attr);
#else
    k_s32 ret;
    k_vicap_dev vicap_dev = VICAP_DEV_ID_0;
    k_vicap_chn vicap_chn = ch;
    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;
    k_vicap_sensor_info sensor_info;


    memset(&dev_attr, 0, sizeof(k_vicap_dev_attr));
    memset(&chn_attr, 0, sizeof(k_vicap_chn_attr));
    memset(&sensor_info, 0, sizeof(k_vicap_sensor_info));

    sensor_info.sensor_type = sensor_type;
    ret = kd_mpi_vicap_get_sensor_info(sensor_info.sensor_type, &sensor_info);
    CHECK_RET(ret, __func__, __LINE__);

    dev_attr.acq_win.width = sensor_info.width;
    dev_attr.acq_win.height = sensor_info.height;
    dev_attr.mode = VICAP_WORK_ONLINE_MODE;

    dev_attr.pipe_ctrl.bits.ae_enable = 1;
    dev_attr.pipe_ctrl.bits.awb_enable = 1;

    memcpy(&dev_attr.sensor_info, &sensor_info, sizeof(k_vicap_sensor_info));

    ret = kd_mpi_vicap_set_dev_attr(vicap_dev, dev_attr);
    CHECK_RET(ret, __func__, __LINE__);

    width = VI_ALIGN_UP(width,16);
    chn_attr.out_win.width = width;
    chn_attr.out_win.height = height;

    chn_attr.crop_win = chn_attr.out_win;
    chn_attr.scale_win = chn_attr.out_win;
    chn_attr.crop_enable = K_FALSE;
    chn_attr.scale_enable = K_FALSE;
    chn_attr.chn_enable = K_TRUE;
    chn_attr.alignment = 12;

    //chn_attr.bit_width = ISP_PIXEL_YUV_8_BIT;
    chn_attr.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    chn_attr.buffer_num = INPUT_BUF_CNT;
    chn_attr.buffer_size = (width * height * 3 / 2 + 0xfff) & ~ 0xfff;
    //chn_attr.block_type = ISP_BUFQUE_TIMEOUT_TYPE;
    //chn_attr.wait_time = 500;

    ret = kd_mpi_vicap_set_chn_attr(vicap_dev, vicap_chn, chn_attr);
    CHECK_RET(ret, __func__, __LINE__);

    /*
     * Motion-analysis channel:
     * VICAP chn 0 remains bound to VENC.
     * VICAP chn 1 produces a low-resolution YUV420SP image for LK flow.
     */
    if (g_motion_analysis_enable)
    {
        memset(&chn_attr, 0, sizeof(k_vicap_chn_attr));

        chn_attr.out_win.width = MOTION_AI_WIDTH;
        chn_attr.out_win.height = MOTION_AI_HEIGHT;

        chn_attr.crop_win = chn_attr.out_win;
        chn_attr.scale_win = chn_attr.out_win;
        chn_attr.crop_enable = K_FALSE;
        chn_attr.scale_enable = K_FALSE;
        chn_attr.chn_enable = K_TRUE;
        chn_attr.alignment = 12;

        chn_attr.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        chn_attr.buffer_num = MOTION_AI_BUF_CNT;
        chn_attr.buffer_size = MOTION_AI_BUF_SIZE;

        ret = kd_mpi_vicap_set_chn_attr(
            vicap_dev,
            VICAP_CHN_ID_1,
            chn_attr);
        CHECK_RET(ret, __func__, __LINE__);

        printf("motion VICAP channel configured: %ux%u, buffers=%u, size=%u\n",
               MOTION_AI_WIDTH,
               MOTION_AI_HEIGHT,
               MOTION_AI_BUF_CNT,
               MOTION_AI_BUF_SIZE);
    }

    ret = kd_mpi_vicap_init(vicap_dev);
    CHECK_RET(ret, __func__, __LINE__);
#endif
}

void sample_vicap_start(k_u32 ch)
{
#ifdef ENABLE_VDSS
    k_s32 ret;
    ret = kd_mpi_vdss_start_pipe(0, ch);
    CHECK_RET(ret, __func__, __LINE__);
#else
    k_s32 ret;

    ret = kd_mpi_vicap_start_stream(VICAP_DEV_ID_0);
    CHECK_RET(ret, __func__, __LINE__);
#endif
}

void sample_vicap_stop(k_u32 ch)
{
#ifdef ENABLE_VDSS
    k_s32 ret;
    ret = kd_mpi_vdss_stop_pipe(0, ch);
    CHECK_RET(ret, __func__, __LINE__);
#else
    k_s32 ret;

    ret = kd_mpi_vicap_stop_stream(VICAP_DEV_ID_0);
    CHECK_RET(ret, __func__, __LINE__);
    ret = kd_mpi_vicap_deinit(VICAP_DEV_ID_0);
    CHECK_RET(ret, __func__, __LINE__);
#endif
}

static k_s32 sample_vb_init(k_u32 ch_cnt, k_bool osd_enable)
{
    k_s32 ret;
    k_vb_config config;
    k_u32 motion_pool_id = 0;

    memset(&config, 0, sizeof(config));

    config.max_pool_cnt = 2;

    if (osd_enable)
    {
        config.max_pool_cnt = 3;
        config.comm_pool[2].blk_cnt = OSD_BUF_CNT * ch_cnt;
        config.comm_pool[2].blk_size = OSD_BUF_SIZE;
        config.comm_pool[2].mode = VB_REMAP_MODE_NOCACHE;
    }

    if (g_motion_analysis_enable)
    {
        /*
         * If OSD is enabled, pool 2 is already occupied by OSD,
         * so the motion-analysis YUV pool uses pool 3.
         */
        motion_pool_id = osd_enable ? 3 : 2;
        config.max_pool_cnt = motion_pool_id + 1;

        config.comm_pool[motion_pool_id].blk_cnt = MOTION_AI_BUF_CNT;
        config.comm_pool[motion_pool_id].blk_size = MOTION_AI_BUF_SIZE;
        config.comm_pool[motion_pool_id].mode = VB_REMAP_MODE_NOCACHE;

        printf("motion VB pool=%u, blocks=%u, block_size=%u\n",
               motion_pool_id,
               MOTION_AI_BUF_CNT,
               MOTION_AI_BUF_SIZE);
    }
    config.comm_pool[0].blk_cnt = INPUT_BUF_CNT * ch_cnt;
    config.comm_pool[0].blk_size = FRAME_BUF_SIZE;
    config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;
    config.comm_pool[1].blk_cnt = OUTPUT_BUF_CNT * ch_cnt;
    config.comm_pool[1].blk_size = STREAM_BUF_SIZE;
    config.comm_pool[1].mode = VB_REMAP_MODE_NOCACHE;

    ret = kd_mpi_vb_set_config(&config);

    venc_debug("-----------venc sample test------------------------\n");

    if (ret)
        venc_debug("vb_set_config failed ret:%d\n", ret);

    ret = kd_mpi_vb_init();
    if (ret)
        venc_debug("vb_init failed ret:%d\n", ret);

    return ret;
}

static k_s32 sample_vb_exit(void)
{
    k_s32 ret;
    ret = kd_mpi_vb_exit();
    if (ret)
        printf("vb_exit failed ret:%d\n", ret);
    return ret;
}

static void datafifo_release(void *pStream)
{
    printf("datafifo release %p\n", pStream);
}

static int datafifo_init(void)
{
    k_s32 ret;
    k_u64 phyAddr = 0;

    if (!g_datafifo_enable)
    {
        printf("datafifo disabled\n");
        return 0;
    }

    k_datafifo_params_s writer_params = {
        10,
        DATAFIFO_BLOCK_LEN,
        K_TRUE,
        DATAFIFO_WRITER
    };

    g_datafifo_buf = (k_char *)malloc(DATAFIFO_BLOCK_LEN);
    if (!g_datafifo_buf)
    {
        printf("datafifo malloc %d failed\n", DATAFIFO_BLOCK_LEN);
        return -1;
    }
    memset(g_datafifo_buf, 0, DATAFIFO_BLOCK_LEN);

    ret = kd_datafifo_open(&hDataFifo[WRITER_INDEX], &writer_params);
    if (K_SUCCESS != ret)
    {
        printf("open datafifo error:%x\n", ret);
        free(g_datafifo_buf);
        g_datafifo_buf = NULL;
        return -1;
    }

    ret = kd_datafifo_cmd(hDataFifo[WRITER_INDEX], DATAFIFO_CMD_GET_PHY_ADDR, &phyAddr);
    if (K_SUCCESS != ret)
    {
        printf("get datafifo phy addr error:%x\n", ret);
        kd_datafifo_close(hDataFifo[WRITER_INDEX]);
        hDataFifo[WRITER_INDEX] = (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE;
        free(g_datafifo_buf);
        g_datafifo_buf = NULL;
        return -1;
    }

    printf("\n================ RTSP DATAFIFO ================\n");
    printf("PhyAddr: %lx\n", phyAddr);
    printf("Little core command:\n");
    printf("./rtspServer -p %lx -t %s\n", phyAddr, g_rtsp_codec_type);
    printf("VLC URL:\n");
    printf("rtsp://<little_core_ip>:8554/BackChannelTest\n");
    printf("===============================================\n\n");

    ret = kd_datafifo_cmd(
        hDataFifo[WRITER_INDEX],
        DATAFIFO_CMD_SET_DATA_RELEASE_CALLBACK,
        (void *)datafifo_release
    );
    if (K_SUCCESS != ret)
    {
        printf("set datafifo release callback error:%x\n", ret);
        kd_datafifo_close(hDataFifo[WRITER_INDEX]);
        hDataFifo[WRITER_INDEX] = (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE;
        free(g_datafifo_buf);
        g_datafifo_buf = NULL;
        return -1;
    }

    g_datafifo_inited = K_TRUE;
    printf("datafifo_init finish\n");
    return 0;
}

static void datafifo_deinit(void)
{
    if (!g_datafifo_inited)
        return;

    kd_datafifo_write(hDataFifo[WRITER_INDEX], NULL);
    kd_datafifo_close(hDataFifo[WRITER_INDEX]);
    hDataFifo[WRITER_INDEX] = (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE;

    if (g_datafifo_buf)
    {
        free(g_datafifo_buf);
        g_datafifo_buf = NULL;
    }

    g_datafifo_inited = K_FALSE;
    printf("datafifo_deinit finish\n");
}

static void datafifo_send_venc_pack(k_u64 pts, k_u8 *data, k_u32 len)
{
    k_s32 ret;
    k_u32 availWriteLen = 0;

    if (!g_datafifo_inited || !g_datafifo_buf || !data || len == 0)
        return;

    if (len + sizeof(k_u64) + sizeof(k_u32) > DATAFIFO_BLOCK_LEN)
    {
        printf("datafifo packet too large, len=%u, block=%d, drop\n", len, DATAFIFO_BLOCK_LEN);
        return;
    }

    /*
     * Flush and reclaim released blocks first. This follows the official
     * pose_det_rtsp_plug writer pattern.
     */
    ret = kd_datafifo_write(hDataFifo[WRITER_INDEX], NULL);
    if (K_SUCCESS != ret)
    {
        printf("datafifo flush write error:%x\n", ret);
        return;
    }

    ret = kd_datafifo_cmd(
        hDataFifo[WRITER_INDEX],
        DATAFIFO_CMD_GET_AVAIL_WRITE_LEN,
        &availWriteLen
    );
    if (K_SUCCESS != ret)
    {
        printf("get available write len error:%x\n", ret);
        return;
    }

    if (availWriteLen < DATAFIFO_BLOCK_LEN)
    {
        /*
         * Reader is not started or network side is slower than encoder.
         * Drop this packet instead of blocking the encoder forever.
         */
        return;
    }

    memset(g_datafifo_buf, 0, DATAFIFO_BLOCK_LEN);
    memcpy(g_datafifo_buf, &pts, sizeof(k_u64));
    memcpy(g_datafifo_buf + sizeof(k_u64), &len, sizeof(k_u32));
    memcpy(g_datafifo_buf + sizeof(k_u64) + sizeof(k_u32), data, len);

    ret = kd_datafifo_write(hDataFifo[WRITER_INDEX], g_datafifo_buf);
    if (K_SUCCESS != ret)
    {
        printf("datafifo write error:%x\n", ret);
        return;
    }

    ret = kd_datafifo_cmd(hDataFifo[WRITER_INDEX], DATAFIFO_CMD_WRITE_DONE, NULL);
    if (K_SUCCESS != ret)
    {
        printf("datafifo write done error:%x\n", ret);
        return;
    }
}


static k_bool h264_stream_has_idr(const k_u8 *data, k_u32 len)
{
    k_u32 i = 0;

    if (!data || len < 5)
        return K_FALSE;

    while (i + 5 < len)
    {
        k_u32 nal_pos = 0;

        if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01)
        {
            nal_pos = i + 3;
        }
        else if (i + 4 < len && data[i] == 0x00 && data[i + 1] == 0x00 &&
                 data[i + 2] == 0x00 && data[i + 3] == 0x01)
        {
            nal_pos = i + 4;
        }
        else
        {
            i++;
            continue;
        }

        if (nal_pos < len)
        {
            k_u8 nal_type = data[nal_pos] & 0x1F;
            if (nal_type == 5)   /* H.264 IDR slice */
                return K_TRUE;
        }

        i = nal_pos + 1;
    }

    return K_FALSE;
}

static k_bool h265_stream_has_idr(const k_u8 *data, k_u32 len)
{
    k_u32 i = 0;

    if (!data || len < 6)
        return K_FALSE;

    while (i + 6 < len)
    {
        k_u32 nal_pos = 0;

        if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01)
        {
            nal_pos = i + 3;
        }
        else if (i + 4 < len && data[i] == 0x00 && data[i + 1] == 0x00 &&
                 data[i + 2] == 0x00 && data[i + 3] == 0x01)
        {
            nal_pos = i + 4;
        }
        else
        {
            i++;
            continue;
        }

        if (nal_pos + 1 < len)
        {
            k_u8 nal_type = (data[nal_pos] >> 1) & 0x3F;
            if (nal_type == 19 || nal_type == 20)  /* H.265 IDR_W_RADL / IDR_N_LP */
                return K_TRUE;
        }

        i = nal_pos + 2;
    }

    return K_FALSE;
}

static k_bool venc_stream_has_idr(const k_u8 *data, k_u32 len)
{
    if (g_rtsp_codec_type && strcmp(g_rtsp_codec_type, "h265") == 0)
        return h265_stream_has_idr(data, len);

    return h264_stream_has_idr(data, len);
}

static void *output_thread(void *arg)
{
    k_venc_stream output;
    int out_cnt, out_frames;
    k_s32 ret;
    int i;
    k_u32 total_len = 0;
    output_info *info = (output_info *)arg;

    k_u32 detected_idr_count = 0;

    out_cnt = 0;
    out_frames = 0;
    g_venc_header_cache_len = 0;
    g_rtsp_header_resend_count = 0;

    if (datafifo_init() != 0)
    {
        printf("datafifo_init failed, continue with file output only\n");
    }

    while (1)
    {
        k_venc_chn_status status;
        memset(&output, 0, sizeof(output));

        ret = kd_mpi_venc_query_status(info->ch_id, &status);
        CHECK_RET(ret, __func__, __LINE__);
        if (ret != K_SUCCESS)
        {
            usleep(10000);
            continue;
        }

        if (status.cur_packs > 0)
            output.pack_cnt = status.cur_packs;
        else
            output.pack_cnt = 1;

        output.pack = malloc(sizeof(k_venc_pack) * output.pack_cnt);
        if (!output.pack)
        {
            printf("malloc output.pack failed\n");
            break;
        }

        ret = kd_mpi_venc_get_stream(info->ch_id, &output, -1);
        CHECK_RET(ret, __func__, __LINE__);
        if (ret != K_SUCCESS)
        {
            free(output.pack);
            usleep(10000);
            continue;
        }

        out_cnt += output.pack_cnt;
        for (i = 0; i < output.pack_cnt; i++)
        {
            k_u8 *pData;

            pData = (k_u8 *)kd_mpi_sys_mmap(output.pack[i].phys_addr, output.pack[i].len);
            if (!pData)
            {
                printf("kd_mpi_sys_mmap failed, phys=0x%lx, len=%u\n",
                    output.pack[i].phys_addr, output.pack[i].len);
                continue;
            }

            /*
             * Cache SPS/PPS for H.264 or VPS/SPS/PPS for H.265.
             * K_VENC_HEADER is normally emitted at the beginning of the stream.
             */
            if (output.pack[i].type == K_VENC_HEADER)
            {
                if (output.pack[i].len <= VENC_HEADER_CACHE_MAX)
                {
                    memcpy(g_venc_header_cache, pData, output.pack[i].len);
                    g_venc_header_cache_len = output.pack[i].len;
                    printf("cache venc header, len=%u\n", g_venc_header_cache_len);
                }
                else
                {
                    printf("venc header too large, len=%u, cache max=%d\n",
                        output.pack[i].len, VENC_HEADER_CACHE_MAX);
                }
            }
            else
            {
                out_frames++;
            }

            /*
             * 1) Keep file output for debug.
             *    The debug file remains the original encoder output and does not include
             *    repeated cached headers inserted only for RTSP clients.
             */
            if (output_file)
                fwrite(pData, 1, output.pack[i].len, output_file);

            /*
             * 2) For live RTSP, detect real IDR frames in the Annex-B stream and resend
             *    cached SPS/PPS immediately before that IDR. This does not rely on
             *    kd_mpi_venc_request_idr(), which may be disabled by this SDK/channel.
             */
            if (output.pack[i].type != K_VENC_HEADER &&
                g_venc_header_cache_len > 0 &&
                venc_stream_has_idr(pData, output.pack[i].len))
            {
                detected_idr_count++;
                datafifo_send_venc_pack(output.pack[i].pts, g_venc_header_cache, g_venc_header_cache_len);
                g_rtsp_header_resend_count++;
                printf("rtsp resend header before detected IDR, header_len=%u, idr_count=%u, resend_count=%u\n",
                    g_venc_header_cache_len, detected_idr_count, g_rtsp_header_resend_count);
            }

            /*
             * 3) Send current encoded stream to little-core rtspServer through DataFIFO.
             *    The little-core reader expects:
             *      [k_u64 pts][k_u32 len][stream bytes]
             */
            datafifo_send_venc_pack(output.pack[i].pts, pData, output.pack[i].len);

            kd_mpi_sys_munmap(pData, output.pack[i].len);
            total_len += output.pack[i].len;
        }

        ret = kd_mpi_venc_release_stream(info->ch_id, &output);
        CHECK_RET(ret, __func__, __LINE__);

        free(output.pack);
    }

    datafifo_deinit();

    if (output_file)
    {
        fclose(output_file);
        output_file = NULL;
    }

    venc_debug("%s>done, ch %d: out_frames %d, size %d bits\n",
        __func__, info->ch_id, out_frames, total_len * 8);

    return arg;
}

static void sample_vi_bind_venc(k_u32 chn_id)
{
    k_mpp_chn venc_mpp_chn;
    k_mpp_chn vi_mpp_chn;
    k_s32 ret;

#ifdef ENABLE_VDSS
    vi_mpp_chn.mod_id = K_ID_VICAP;
#else
    vi_mpp_chn.mod_id = K_ID_VI;
#endif

    venc_mpp_chn.mod_id = K_ID_VENC;
    venc_mpp_chn.dev_id = 0;
    venc_mpp_chn.chn_id = chn_id;

    vi_mpp_chn.dev_id = chn_id;
    vi_mpp_chn.chn_id = chn_id;
    ret = kd_mpi_sys_bind(&vi_mpp_chn, &venc_mpp_chn);
    if (ret)
    {
        printf("kd_mpi_sys_bind failed:0x%x\n", ret);
    }

    return;
}

static void sample_vi_unbind_venc(k_u32 chn_id)
{
    k_mpp_chn venc_mpp_chn;
    k_mpp_chn vi_mpp_chn;

    venc_mpp_chn.mod_id = K_ID_VENC;
    venc_mpp_chn.dev_id = 0;
    venc_mpp_chn.chn_id = chn_id;

#ifdef ENABLE_VDSS
    vi_mpp_chn.mod_id = K_ID_VICAP;
#else
    vi_mpp_chn.mod_id = K_ID_VI;
#endif
    vi_mpp_chn.dev_id = chn_id;
    vi_mpp_chn.chn_id = chn_id;
    kd_mpi_sys_unbind(&vi_mpp_chn, &venc_mpp_chn);

    return;
}

static k_s32 prepare_osd(osd_conf_t *osd_conf, k_vb_blk_handle *osd_blk_handle)
{
    int osd_byte;
    int i;
    k_vb_blk_handle handle;
    k_s32 pool_id = 0;
    k_u64 phys_addr = 0;
    k_u8 *virt_addr_osd;
    k_s32 osd_pool_id;

    switch (osd_conf->osd_fmt)
    {
    case K_VENC_2D_OSD_FMT_ARGB8888:
        osd_byte = 4;
        break;
    case K_VENC_2D_OSD_FMT_ARGB4444:
        osd_byte = 2;
        break;
    case K_VENC_2D_OSD_FMT_ARGB1555:
        osd_byte = 2;
        break;
    default:
        osd_byte = 4;
        break;
    }
    venc_debug("osd_byte = %d\n", osd_byte);

    for (i = 0; i < 1; i++)
    {
        handle = kd_mpi_vb_get_block(VB_INVALID_POOLID, OSD_BUF_SIZE, NULL);

        if (handle == VB_INVALID_HANDLE)
        {
            printf("%s osd get vb block error\n", __func__);
            break;
        }
        *osd_blk_handle = handle;

        pool_id = kd_mpi_vb_handle_to_pool_id(handle);
        if (pool_id == VB_INVALID_POOLID)
        {
            printf("%s osd get pool id error\n", __func__);
            break;
        }
        osd_pool_id = pool_id;
        printf("%s>osd_pool_id = %d\n", __func__, osd_pool_id);

        phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);
        if (phys_addr == 0)
        {
            printf("%s osd get phys addr error\n", __func__);
            break;
        }

        printf("%s>i %d, phys_addr 0x%lx, blk_size %d\n", __func__, i, phys_addr, OSD_BUF_SIZE);
        virt_addr_osd = (k_u8 *)kd_mpi_sys_mmap_cached(phys_addr, OSD_BUF_SIZE);

        if (virt_addr_osd == NULL)
        {
            printf("%s osd mmap error\n", __func__);
            break;
        }
        printf("%s>i %d, osd virt_addr_osd %p\n", __func__, i, virt_addr_osd);

        memcpy(virt_addr_osd, &osd_data, osd_data_size);
        osd_conf->osd_phys_addr[i][0] = phys_addr;
        osd_conf->osd_virt_addr[i][0] = virt_addr_osd;
        kd_mpi_sys_mmz_flush_cache(osd_conf->osd_phys_addr[i][0], osd_conf->osd_virt_addr[i][0], OSD_BUF_SIZE);
    }
    return K_SUCCESS;
}

k_s32 sample_exit(venc_conf_t *venc_conf)
{
    int ch = 0;
    int ret = 0;

    printf("%s>g_venc_sample_status = %d\n", __FUNCTION__, g_venc_sample_status);

    /*
     * Stop the dump thread before stopping/deinitializing VICAP.
     */
    motion_frame_stop();
    motion_osd_unbind();

    switch (g_venc_sample_status)
    {
    case VENC_SAMPLE_STATUE_RUNING:
    case VENC_SAMPLE_STATUS_BINDED:
        sample_vicap_stop(ch);
        sample_vi_unbind_venc(ch);
    case VENC_SAMPLE_STATUS_START:
        kd_mpi_venc_stop_chn(ch);
        if (venc_conf->osd_enable)
            kd_mpi_venc_detach_2d(ch);
    case VENC_SAMPLE_STATUS_INIT:
        kd_mpi_venc_destroy_chn(ch);
        if (venc_conf->osd_enable)
        {
            ret = kd_mpi_vb_release_block(venc_conf->osd_blk_handle);
            CHECK_RET(ret, __func__, __LINE__);

            for (int k = 0; k < venc_conf->osd_conf->osd_region_num; k++)
            {
                printf("osd_conf->osd_virt_addr[%d][0] 0x%p\n", k, venc_conf->osd_conf->osd_virt_addr[k][0]);
                ret = kd_mpi_sys_munmap(venc_conf->osd_conf->osd_virt_addr[k][0], OSD_BUF_SIZE);
                CHECK_RET(ret, __func__, __LINE__);
            }
        }
        break;
    default:
        break;
    }

    pthread_cancel(venc_conf->output_tid);
    pthread_join(venc_conf->output_tid, NULL);

    datafifo_deinit();

    venc_debug("kill ch %d thread done! ch_done %d, chnum %d\n", ch, g_venc_conf.ch_done, g_venc_conf.chnum);

    ret = kd_mpi_venc_close_fd();
    CHECK_RET(ret, __func__, __LINE__);

    if (output_file)
    {
        fclose(output_file);
        output_file = NULL;
    }
    sample_vb_exit();

    g_venc_conf.ch_done = K_TRUE;

    return K_SUCCESS;
}

static void *exit_app(void *arg)
{
    venc_conf_t *venc_conf = (venc_conf_t *)arg;
    while (getchar() != 'q')
    {
        usleep(10000);
    }
    sample_exit(venc_conf);
    return K_SUCCESS;
}

k_s32 sample_venc_h265(k_vicap_sensor_type sensor_type)
{
    int chnum = 1;
    int ch = 0;
    k_u32 output_frames = 10;
    k_u32 bitrate   = 4000;   //kbps
    int width       = enc_width;
    int height      = enc_height;
    k_venc_rc_mode rc_mode  = K_VENC_RC_MODE_CBR;
    k_payload_type type     = K_PT_H265;
    k_venc_profile profile  = VENC_PROFILE_H265_MAIN;
    int ret = 0;

    sample_vb_init(chnum, K_FALSE);

    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.pic_width = width;
    attr.venc_attr.pic_height = height;
    attr.venc_attr.stream_buf_size = STREAM_BUF_SIZE;
    attr.venc_attr.stream_buf_cnt = OUTPUT_BUF_CNT;

    attr.rc_attr.rc_mode = rc_mode;
    attr.rc_attr.cbr.src_frame_rate = 30;
    attr.rc_attr.cbr.dst_frame_rate = 30;
    attr.rc_attr.cbr.bit_rate = bitrate;

    attr.venc_attr.type = type;
    attr.venc_attr.profile = profile;
    venc_debug("payload type is H265\n");

    ret = kd_mpi_venc_create_chn(ch, &attr);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_INIT;

    if (intbuf_size > 0)
    {
        kd_mpi_venc_set_intbuf_size(ch, intbuf_size);
        printf("%s>intbuf_size %d\n", __func__, intbuf_size);
    }

    ret = kd_mpi_venc_start_chn(ch);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_START;

    sample_vicap_config(ch, width, height, sensor_type);
    sample_vi_bind_venc(ch);
    sample_vicap_start(ch);
    g_venc_sample_status = VENC_SAMPLE_STATUS_BINDED;

    output_info info;
    memset(&info, 0, sizeof(info));
    info.ch_id = ch;
    info.output_frames = output_frames;

    pthread_create(&g_venc_conf.output_tid, NULL, output_thread, &info);
    g_venc_sample_status = VENC_SAMPLE_STATUE_RUNING;

    printf("press 'q' to exit application!!\n");
    while (!g_venc_conf.ch_done)
    {
        usleep(10000);
    }
    return K_SUCCESS;
}

k_s32 sample_venc_jpeg(k_vicap_sensor_type sensor_type)
{
    int chnum = 1;
    int ch = 0;
    k_u32 output_frames = 10;
    int width       = enc_width;
    int height      = enc_height;
    k_venc_rc_mode rc_mode  = K_VENC_RC_MODE_MJPEG_FIXQP;
    k_payload_type type     = K_PT_JPEG;
    k_u32 q_factor          = 45;
    int ret = 0;

    sample_vb_init(chnum, K_FALSE);

    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.pic_width = width;
    attr.venc_attr.pic_height = height;
    attr.venc_attr.stream_buf_size = STREAM_BUF_SIZE;
    attr.venc_attr.stream_buf_cnt = OUTPUT_BUF_CNT;

    attr.venc_attr.type = type;
    attr.rc_attr.rc_mode = rc_mode;
    attr.rc_attr.mjpeg_fixqp.src_frame_rate = 30;
    attr.rc_attr.mjpeg_fixqp.dst_frame_rate = 30;
    attr.rc_attr.mjpeg_fixqp.q_factor = q_factor;

    venc_debug("payload type is JPEG\n");

    ret = kd_mpi_venc_create_chn(ch, &attr);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_INIT;

    if (intbuf_size > 0)
    {
        kd_mpi_venc_set_intbuf_size(ch, intbuf_size);
        printf("%s>intbuf_size %d\n", __func__, intbuf_size);
    }

    ret = kd_mpi_venc_start_chn(ch);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_START;

    sample_vicap_config(ch, width, height, sensor_type);
    sample_vi_bind_venc(ch);
    sample_vicap_start(ch);
    g_venc_sample_status = VENC_SAMPLE_STATUS_BINDED;

    output_info info;
    memset(&info, 0, sizeof(info));
    info.ch_id = ch;
    info.output_frames = output_frames;

    pthread_create(&g_venc_conf.output_tid, NULL, output_thread, &info);
    g_venc_sample_status = VENC_SAMPLE_STATUE_RUNING;

    printf("press 'q' to exit application!!\n");
    while (!g_venc_conf.ch_done)
    {
        usleep(10000);
    }
    return K_SUCCESS;
}

k_s32 sample_venc_osd_h264(k_vicap_sensor_type sensor_type)
{
    int chnum = 1;
    int ch = 0;
    k_u32 output_frames = 10;
    k_u32 bitrate   = 4000;   //kbps
    int width       = enc_width;
    int height      = enc_height;
    k_venc_rc_mode rc_mode  = K_VENC_RC_MODE_CBR;
    k_payload_type type     = K_PT_H264;
    k_venc_profile profile  = VENC_PROFILE_H264_HIGH;

    osd_conf_t osd_conf =
    {
        .osd_width  = OSD_MAX_WIDTH,
        .osd_height = OSD_MAX_HEIGHT,
        .osd_startx = 16,
        .osd_starty = 16,
        .bg_alpha   = 200,
        .osd_alpha  = 200,
        .video_alpha = 200,
        .add_order  = K_VENC_2D_ADD_ORDER_VIDEO_OSD,
        .bg_color   = (200 << 16) | (128 << 8) | (128 << 0),
        .osd_fmt    = K_VENC_2D_OSD_FMT_ARGB8888,
        .osd_region_num = 1
    };
    int ret = 0;

    g_motion_analysis_enable = K_TRUE;
    sample_vb_init(chnum, K_TRUE);
    g_venc_conf.osd_conf = &osd_conf;
    g_venc_conf.osd_enable = K_TRUE;
    prepare_osd(&osd_conf, &g_venc_conf.osd_blk_handle);

    if (motion_osd_bind(
            (k_u64)osd_conf.osd_phys_addr[0][0],
            osd_conf.osd_virt_addr[0][0],
            osd_conf.osd_width,
            osd_conf.osd_height,
            OSD_BUF_SIZE) != 0)
    {
        printf(
            "motion_osd_bind failed; "
            "video will continue without dynamic OSD\n");
    }

    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.pic_width = width;
    attr.venc_attr.pic_height = height;
    attr.venc_attr.stream_buf_size = STREAM_BUF_SIZE;
    attr.venc_attr.stream_buf_cnt = OUTPUT_BUF_CNT;

    attr.rc_attr.rc_mode = rc_mode;
    attr.rc_attr.cbr.src_frame_rate = 15;
    attr.rc_attr.cbr.dst_frame_rate = 15;
    attr.rc_attr.cbr.gop = 15;
    attr.rc_attr.cbr.bit_rate = bitrate;

    attr.venc_attr.type = type;
    attr.venc_attr.profile = profile;
    venc_debug("payload type is H264\n");

    ret = kd_mpi_venc_create_chn(ch, &attr);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_INIT;

    if (intbuf_size > 0)
    {
        kd_mpi_venc_set_intbuf_size(ch, intbuf_size);
        printf("%s>intbuf_size %d\n", __func__, intbuf_size);
    }

    ret = kd_mpi_venc_start_chn(ch);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_START;

    kd_mpi_venc_attach_2d(ch);
    CHECK_RET(ret, __func__, __LINE__);

    ret = kd_mpi_venc_set_2d_mode(ch, K_VENC_2D_CALC_MODE_OSD);
    CHECK_RET(ret, __func__, __LINE__);

    k_venc_2d_osd_attr venc_2d_osd_attr;
    venc_2d_osd_attr.width  = osd_conf.osd_width;
    venc_2d_osd_attr.height = osd_conf.osd_height;
    venc_2d_osd_attr.startx = osd_conf.osd_startx;
    venc_2d_osd_attr.starty = osd_conf.osd_starty;
    venc_2d_osd_attr.phys_addr[0] = osd_conf.osd_phys_addr[0][0];
    venc_2d_osd_attr.phys_addr[1] = osd_conf.osd_phys_addr[0][1];
    venc_2d_osd_attr.phys_addr[2] = osd_conf.osd_phys_addr[0][2];
    venc_2d_osd_attr.bg_alpha = osd_conf.bg_alpha;
    venc_2d_osd_attr.osd_alpha = osd_conf.osd_alpha;
    venc_2d_osd_attr.video_alpha = osd_conf.video_alpha;
    venc_2d_osd_attr.add_order = osd_conf.add_order;
    venc_2d_osd_attr.bg_color = osd_conf.bg_color;
    venc_2d_osd_attr.fmt = osd_conf.osd_fmt;
    printf("%s>osd phys addr 0x%08x 0x%08x 0x%08x\n", __FUNCTION__, venc_2d_osd_attr.phys_addr[0], venc_2d_osd_attr.phys_addr[1], venc_2d_osd_attr.phys_addr[2]);

    ret = kd_mpi_venc_set_2d_osd_param(ch, 0, &venc_2d_osd_attr);
    CHECK_RET(ret, __func__, __LINE__);

    sample_vicap_config(ch, width, height, sensor_type);
    sample_vi_bind_venc(ch);
    sample_vicap_start(ch);

    if (motion_frame_start() != 0)
    {
        printf("motion_frame_start failed; RTSP will continue without analysis\n");
    }

    g_venc_sample_status = VENC_SAMPLE_STATUS_BINDED;

    output_info info;
    memset(&info, 0, sizeof(info));
    info.ch_id = ch;
    info.output_frames = output_frames;

    pthread_create(&g_venc_conf.output_tid, NULL, output_thread, &info);
    g_venc_sample_status = VENC_SAMPLE_STATUE_RUNING;

    printf("press 'q' to exit application!!\n");
    while (!g_venc_conf.ch_done)
    {
        usleep(10000);
    }
    return K_SUCCESS;
}

k_s32 sample_venc_osd_border_h265(k_vicap_sensor_type sensor_type)
{
    int chnum = 1;
    int ch = 0;
    k_u32 output_frames = 10;
    k_u32 bitrate   = 4000;   //kbps
    int width       = enc_width;
    int height      = enc_height;

    k_venc_rc_mode rc_mode  = K_VENC_RC_MODE_CBR;
    k_payload_type type     = K_PT_H265;
    k_venc_profile profile  = VENC_PROFILE_H265_MAIN;

    osd_conf_t osd_conf =
    {
        .osd_width  = 40,
        .osd_height = 40,
        .osd_startx = 4,
        .osd_starty = 4,
        .bg_alpha   = 200,
        .osd_alpha  = 200,
        .video_alpha = 200,
        .add_order  = K_VENC_2D_ADD_ORDER_VIDEO_OSD,
        .bg_color   = (200 << 16) | (128 << 8) | (128 << 0),
        .osd_fmt    = K_VENC_2D_OSD_FMT_ARGB8888,
        .osd_region_num = 1
    };
    border_conf_t border_conf =
    {
        .width = 40,
        .height = 40,
        .line_width = 2,
        .startx = 4,
        .starty = 4
    };
    int ret = 0;

    sample_vb_init(chnum, K_TRUE);
    g_venc_conf.osd_conf = &osd_conf;
    g_venc_conf.osd_enable = K_TRUE;
    prepare_osd(&osd_conf, &g_venc_conf.osd_blk_handle);

    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.pic_width = width;
    attr.venc_attr.pic_height = height;
    attr.venc_attr.stream_buf_size = STREAM_BUF_SIZE;
    attr.venc_attr.stream_buf_cnt = OUTPUT_BUF_CNT;

    attr.rc_attr.rc_mode = rc_mode;
    attr.rc_attr.cbr.src_frame_rate = 30;
    attr.rc_attr.cbr.dst_frame_rate = 30;
    attr.rc_attr.cbr.bit_rate = bitrate;

    attr.venc_attr.type = type;
    attr.venc_attr.profile = profile;
    venc_debug("payload type is H264\n");

    ret = kd_mpi_venc_create_chn(ch, &attr);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_INIT;

    if (intbuf_size > 0)
    {
        kd_mpi_venc_set_intbuf_size(ch, intbuf_size);
        printf("%s>intbuf_size %d\n", __func__, intbuf_size);
    }

    ret = kd_mpi_venc_start_chn(ch);
    CHECK_RET(ret, __func__, __LINE__);
    g_venc_sample_status = VENC_SAMPLE_STATUS_START;

    kd_mpi_venc_attach_2d(ch);
    CHECK_RET(ret, __func__, __LINE__);

    ret = kd_mpi_venc_set_2d_mode(ch, K_VENC_2D_CALC_MODE_OSD_BORDER);
    CHECK_RET(ret, __func__, __LINE__);

    k_venc_2d_osd_attr venc_2d_osd_attr;
    venc_2d_osd_attr.width  = osd_conf.osd_width;
    venc_2d_osd_attr.height = osd_conf.osd_height;
    venc_2d_osd_attr.startx = osd_conf.osd_startx;
    venc_2d_osd_attr.starty = osd_conf.osd_starty;
    venc_2d_osd_attr.phys_addr[0] = osd_conf.osd_phys_addr[0][0];
    venc_2d_osd_attr.phys_addr[1] = osd_conf.osd_phys_addr[0][1];
    venc_2d_osd_attr.phys_addr[2] = osd_conf.osd_phys_addr[0][2];
    venc_2d_osd_attr.bg_alpha = osd_conf.bg_alpha;
    venc_2d_osd_attr.osd_alpha = osd_conf.osd_alpha;
    venc_2d_osd_attr.video_alpha = osd_conf.video_alpha;
    venc_2d_osd_attr.add_order = osd_conf.add_order;
    venc_2d_osd_attr.bg_color = osd_conf.bg_color;
    venc_2d_osd_attr.fmt = osd_conf.osd_fmt;
    printf("%s>osd phys addr 0x%08x 0x%08x 0x%08x\n", __FUNCTION__, venc_2d_osd_attr.phys_addr[0], venc_2d_osd_attr.phys_addr[1], venc_2d_osd_attr.phys_addr[2]);

    ret = kd_mpi_venc_set_2d_osd_param(ch, 0, &venc_2d_osd_attr);
    CHECK_RET(ret, __func__, __LINE__);

    k_venc_2d_border_attr venc_2d_border_attr;
    venc_2d_border_attr.width = border_conf.width;
    venc_2d_border_attr.height = border_conf.height;
    venc_2d_border_attr.line_width = border_conf.line_width;
    venc_2d_border_attr.color = (100 << 16) | (200 << 8) | (70 << 0);
    venc_2d_border_attr.startx = border_conf.startx;
    venc_2d_border_attr.starty = border_conf.starty;
    ret = kd_mpi_venc_set_2d_border_param(ch, 0, &venc_2d_border_attr);
    CHECK_RET(ret, __func__, __LINE__);

    sample_vicap_config(ch, width, height, sensor_type);
    sample_vi_bind_venc(ch);
    sample_vicap_start(ch);
    g_venc_sample_status = VENC_SAMPLE_STATUS_BINDED;

    output_info info;
    memset(&info, 0, sizeof(info));
    info.ch_id = ch;
    info.output_frames = output_frames;

    pthread_create(&g_venc_conf.output_tid, NULL, output_thread, &info);
    g_venc_sample_status = VENC_SAMPLE_STATUE_RUNING;

    printf("press 'q' to exit application!!\n");
    while (!g_venc_conf.ch_done)
    {
        usleep(10000);
    }
    return K_SUCCESS;
}

void sample_venc_usage(char *sPrgNm)
{
    printf("Usage : %s [index] -sensor [sensor_index] -o [filename]\n", sPrgNm);
    printf("index:\n");
    printf("\t  0) H.265e.\n");
    printf("\t  1) JPEG encode.\n");
    printf("\t  2) OSD + H.264e.\n");
    printf("\t  3) OSD + Border + H.265e.\n");
    printf("\n");
    printf("sensor_index:\n");
    printf("\t  see vicap doc. For your board, GC2093 is usually 52.\n");
    printf("\n");
    printf("extra options:\n");
    printf("\t  -o [filename]    dump encoded stream to file for debug\n");
    printf("\t  -buf [size]      set venc internal buffer size\n");
    printf("\t  -w [width]       encode width, default 1280\n");
    printf("\t  -h [height]      encode height, default 720\n");
    printf("\t  -no-df           disable DataFIFO output\n");
    printf("\n");
    printf("RTSP/DataFIFO usage example:\n");
    printf("\t  big core:    %s 2 -sensor 52 -o /sharefs/rtsp_debug.h264\n", sPrgNm);
    printf("\t  little core: ./rtspServer -p <PhyAddr> -t h264\n");
    printf("\t  VLC:         rtsp://<little_core_ip>:8554/BackChannelTest\n");
    return;
}

int main(int argc, char *argv[])
{
    k_vicap_sensor_type sensor_type = GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
    pthread_t exit_thread_handle;
    int case_index = 0;
    int ret = 0;

    printf("argc = %d\n", argc);

    if ((argc == 1) || (!strncmp(argv[1], "-h", 2)))
    {
        sample_venc_usage(argv[0]);
        return K_SUCCESS;
    }

    if ((atoi(argv[1]) < 0) || (atoi(argv[1]) > 3))
    {
        printf("index error\n");
        sample_venc_usage(argv[0]);
        return K_FAILED;
    }

    case_index = atoi(argv[1]);
    for (int i = 2; i < argc; i++)
    {
        if (strcmp(argv[i], "-sensor") == 0)
        {
            sensor_type = (k_vicap_sensor_type)atoi(argv[i + 1]);
        }
        else if (strcmp(argv[i], "-o") == 0)
        {
            strcpy(out_filename, argv[i + 1]);
            if ((output_file = fopen(out_filename, "wb")) == NULL)
            {
                venc_debug("Cannot open output file\n");
            }
            printf("out_filename: %s\n", out_filename);
        }
        else if(strcmp(argv[i], "-buf") == 0)
        {
            intbuf_size = atoi(argv[i + 1]);
        }
        else if(strcmp(argv[i], "-w") == 0)
        {
            enc_width = atoi(argv[i + 1]);
        }
        else if(strcmp(argv[i], "-h") == 0)
        {
            enc_height = atoi(argv[i + 1]);
        }
        else if(strcmp(argv[i], "-no-df") == 0)
        {
            g_datafifo_enable = K_FALSE;
        }
    }

    if (case_index == 0 || case_index == 3)
        g_rtsp_codec_type = "h265";
    else if (case_index == 2)
        g_rtsp_codec_type = "h264";
    else
        g_datafifo_enable = K_FALSE; /* JPEG is not a continuous RTSP video stream here. */

    memset(&g_venc_conf, 0, sizeof(venc_conf_t));
    pthread_create(&exit_thread_handle, NULL, exit_app, &g_venc_conf);

    switch (case_index)
    {
    case 0:
        ret = sample_venc_h265(sensor_type);
        break;
    case 1:
        ret = sample_venc_jpeg(sensor_type);
        break;
    case 2:
        ret = sample_venc_osd_h264(sensor_type);
        break;
    case 3:
        ret = sample_venc_osd_border_h265(sensor_type);
        break;

    default:
        printf("the index is invaild!\n");
        sample_venc_usage(argv[0]);
        return K_FAILED;
    }

    if (K_SUCCESS == ret)
    {
        printf("sample_venc pass.\n");
    }
    else
    {
        printf("sample_venc fail.\n");
    }

    printf("sample encode done!\n");

    return 0;
}
