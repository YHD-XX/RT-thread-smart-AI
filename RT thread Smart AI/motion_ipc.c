#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "motion_ipc.h"

#define MOTION_IPC_SHM_NAME        "/k230_motion_stage6_shm"
#define MOTION_IPC_FRAME_SEM_NAME  "/k230_motion_stage6_frame"
#define MOTION_IPC_EVENT_SEM_NAME  "/k230_motion_stage6_event"

#define MOTION_IPC_MAGIC           0x4D4F5436U
#define MOTION_IPC_VERSION         1U
#define MOTION_IPC_RETRY_US        100000U

typedef struct
{
    k_u32 magic;
    k_u32 version;
    k_u32 width;
    k_u32 height;
    k_u32 gray_size;

    volatile k_u32 producer_alive;
    volatile k_u32 analyzer_alive;
    volatile k_u32 stop_requested;

    volatile k_u32 frame_sequence;
    volatile k_u32 frame_index;
    volatile k_u32 frame_number;

    k_u8 frame_gray_buffers[2][MOTION_IPC_GRAY_SIZE];
    k_u8 frame_yuv_buffers[2][MOTION_IPC_YUV_SIZE];

    volatile k_u32 event_sequence;
    volatile k_u32 event_index;
    volatile k_u32 event_frame_number;

    k_u8 event_yuv_buffers[2][MOTION_IPC_YUV_SIZE];

} motion_ipc_shared_t;


static int g_ipc_fd = -1;
static motion_ipc_shared_t *g_ipc_shared = NULL;

static sem_t *g_ipc_frame_sem = SEM_FAILED;
static sem_t *g_ipc_event_sem = SEM_FAILED;

static k_bool g_ipc_is_producer = K_FALSE;
static k_bool g_ipc_is_consumer = K_FALSE;

static k_u32 g_last_event_sequence = 0;
static k_u32 g_last_frame_sequence = 0;


static void motion_ipc_memory_barrier(void)
{
    __sync_synchronize();
}


static void motion_ipc_coalesced_post(sem_t *sem)
{
    if (sem == SEM_FAILED)
        return;

    /*
     * 信号量最多保留一个待处理通知。
     * 分析进程速度不足时直接处理最新帧，
     * 避免积压大量旧帧。
     */
    while (sem_trywait(sem) == 0)
    {
    }

    sem_post(sem);
}


static void motion_ipc_close_handles(void)
{
    if (g_ipc_frame_sem != SEM_FAILED)
    {
        sem_close(g_ipc_frame_sem);
        g_ipc_frame_sem = SEM_FAILED;
    }

    if (g_ipc_event_sem != SEM_FAILED)
    {
        sem_close(g_ipc_event_sem);
        g_ipc_event_sem = SEM_FAILED;
    }

    if (g_ipc_shared &&
        g_ipc_shared != MAP_FAILED)
    {
        munmap(g_ipc_shared,
               sizeof(motion_ipc_shared_t));

        g_ipc_shared = NULL;
    }

    if (g_ipc_fd >= 0)
    {
        close(g_ipc_fd);
        g_ipc_fd = -1;
    }
}


int motion_ipc_producer_start(k_u32 width,
                              k_u32 height)
{
    if (g_ipc_is_producer)
        return 0;

    if (width != MOTION_IPC_WIDTH ||
        height != MOTION_IPC_HEIGHT)
    {
        printf("motion_ipc: unsupported geometry=%ux%u\n",
               width,
               height);

        return -1;
    }

    /*
     * 清理上一次异常退出可能遗留的对象。
     */
    shm_unlink(MOTION_IPC_SHM_NAME);
    sem_unlink(MOTION_IPC_FRAME_SEM_NAME);
    sem_unlink(MOTION_IPC_EVENT_SEM_NAME);

    g_ipc_fd = shm_open(
        MOTION_IPC_SHM_NAME,
        O_CREAT | O_EXCL | O_RDWR,
        0666);

    if (g_ipc_fd < 0)
    {
        printf("motion_ipc: producer shm_open failed, errno=%d\n",
               errno);
        return -1;
    }

    if (ftruncate(
            g_ipc_fd,
            sizeof(motion_ipc_shared_t)) != 0)
    {
        printf("motion_ipc: ftruncate failed, errno=%d\n",
               errno);

        motion_ipc_close_handles();
        shm_unlink(MOTION_IPC_SHM_NAME);
        return -1;
    }

    g_ipc_shared = (motion_ipc_shared_t *)mmap(
        NULL,
        sizeof(motion_ipc_shared_t),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        g_ipc_fd,
        0);

    if (g_ipc_shared == MAP_FAILED)
    {
        printf("motion_ipc: producer mmap failed, errno=%d\n",
               errno);

        g_ipc_shared = NULL;
        motion_ipc_close_handles();
        shm_unlink(MOTION_IPC_SHM_NAME);
        return -1;
    }

    g_ipc_frame_sem = sem_open(
        MOTION_IPC_FRAME_SEM_NAME,
        O_CREAT | O_EXCL,
        0666,
        0);

    if (g_ipc_frame_sem == SEM_FAILED)
    {
        printf("motion_ipc: frame sem_open failed, errno=%d\n",
               errno);

        motion_ipc_close_handles();
        shm_unlink(MOTION_IPC_SHM_NAME);
        sem_unlink(MOTION_IPC_FRAME_SEM_NAME);
        return -1;
    }

    g_ipc_event_sem = sem_open(
        MOTION_IPC_EVENT_SEM_NAME,
        O_CREAT | O_EXCL,
        0666,
        0);

    if (g_ipc_event_sem == SEM_FAILED)
    {
        printf("motion_ipc: event sem_open failed, errno=%d\n",
               errno);

        motion_ipc_close_handles();
        shm_unlink(MOTION_IPC_SHM_NAME);
        sem_unlink(MOTION_IPC_FRAME_SEM_NAME);
        sem_unlink(MOTION_IPC_EVENT_SEM_NAME);
        return -1;
    }

    memset(g_ipc_shared,
           0,
           sizeof(motion_ipc_shared_t));

    g_ipc_shared->magic = MOTION_IPC_MAGIC;
    g_ipc_shared->version = MOTION_IPC_VERSION;
    g_ipc_shared->width = width;
    g_ipc_shared->height = height;
    g_ipc_shared->gray_size = width * height;
    g_ipc_shared->producer_alive = 1;
    g_ipc_shared->stop_requested = 0;

    motion_ipc_memory_barrier();

    g_last_event_sequence = 0;
    g_ipc_is_producer = K_TRUE;

    printf("motion_ipc: producer started, shm_size=%u, "
           "image=%ux%u\n",
           (unsigned int)sizeof(motion_ipc_shared_t),
           width,
           height);

    return 0;
}


int motion_ipc_publish_frame(const k_u8 *gray,
                             const k_u8 *yuv420sp,
                             k_u32 frame_number)
{
    k_u32 next_index;
    k_u32 sequence;

    if (!g_ipc_is_producer ||
        !g_ipc_shared ||
        !gray ||
        !yuv420sp)
    {
        return -1;
    }

    if (g_ipc_shared->stop_requested)
        return -1;

    next_index =
        (g_ipc_shared->frame_index + 1U) & 1U;

    memcpy(
        g_ipc_shared->frame_gray_buffers[next_index],
        gray,
        MOTION_IPC_GRAY_SIZE);

    memcpy(
        g_ipc_shared->frame_yuv_buffers[next_index],
        yuv420sp,
        MOTION_IPC_YUV_SIZE);

    motion_ipc_memory_barrier();

    sequence =
        g_ipc_shared->frame_sequence + 1U;

    g_ipc_shared->frame_number = frame_number;
    g_ipc_shared->frame_index = next_index;
    g_ipc_shared->frame_sequence = sequence;

    motion_ipc_memory_barrier();

    motion_ipc_coalesced_post(g_ipc_frame_sem);

    return 0;
}


int motion_ipc_try_get_event(k_u8 *yuv420sp,
                             k_u32 yuv_capacity,
                             k_u32 *frame_number)
{
    k_u32 sequence_before;
    k_u32 sequence_after;
    k_u32 index_before;
    k_u32 index_after;
    int retry;

    if (!g_ipc_is_producer ||
        !g_ipc_shared ||
        !yuv420sp ||
        yuv_capacity < MOTION_IPC_YUV_SIZE)
    {
        return -1;
    }

    if (sem_trywait(g_ipc_event_sem) != 0)
    {
        if (errno == EAGAIN)
            return 0;

        return -1;
    }

    for (retry = 0; retry < 3; ++retry)
    {
        sequence_before =
            g_ipc_shared->event_sequence;

        index_before =
            g_ipc_shared->event_index & 1U;

        motion_ipc_memory_barrier();

        memcpy(
            yuv420sp,
            g_ipc_shared->event_yuv_buffers[index_before],
            MOTION_IPC_YUV_SIZE);

        motion_ipc_memory_barrier();

        sequence_after =
            g_ipc_shared->event_sequence;

        index_after =
            g_ipc_shared->event_index & 1U;

        if (sequence_before == sequence_after &&
            index_before == index_after)
        {
            if (sequence_after ==
                g_last_event_sequence)
            {
                return 0;
            }

            g_last_event_sequence =
                sequence_after;

            if (frame_number)
            {
                *frame_number =
                    g_ipc_shared->event_frame_number;
            }

            return 1;
        }
    }

    printf("motion_ipc: unstable event copy\n");
    return -1;
}


void motion_ipc_producer_stop(void)
{
    if (!g_ipc_is_producer)
        return;

    if (g_ipc_shared)
    {
        g_ipc_shared->stop_requested = 1;
        g_ipc_shared->producer_alive = 0;

        motion_ipc_memory_barrier();

        /*
         * 唤醒可能阻塞在sem_wait中的分析进程。
         */
        sem_post(g_ipc_frame_sem);

        usleep(100000);
    }

    motion_ipc_close_handles();

    shm_unlink(MOTION_IPC_SHM_NAME);
    sem_unlink(MOTION_IPC_FRAME_SEM_NAME);
    sem_unlink(MOTION_IPC_EVENT_SEM_NAME);

    g_ipc_is_producer = K_FALSE;

    printf("motion_ipc: producer stopped\n");
}


int motion_ipc_consumer_connect(k_u32 timeout_ms)
{
    k_u32 elapsed_ms = 0;

    if (g_ipc_is_consumer)
        return 0;

    while (elapsed_ms <= timeout_ms)
    {
        g_ipc_fd = shm_open(
            MOTION_IPC_SHM_NAME,
            O_RDWR,
            0666);

        if (g_ipc_fd >= 0)
            break;

        usleep(MOTION_IPC_RETRY_US);
        elapsed_ms += 100;
    }

    if (g_ipc_fd < 0)
    {
        printf("motion_ipc: consumer shm_open timeout, "
               "errno=%d\n",
               errno);
        return -1;
    }

    g_ipc_shared = (motion_ipc_shared_t *)mmap(
        NULL,
        sizeof(motion_ipc_shared_t),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        g_ipc_fd,
        0);

    if (g_ipc_shared == MAP_FAILED)
    {
        printf("motion_ipc: consumer mmap failed, errno=%d\n",
               errno);

        g_ipc_shared = NULL;
        motion_ipc_close_handles();
        return -1;
    }

    elapsed_ms = 0;

    while (elapsed_ms <= timeout_ms)
    {
        if (g_ipc_shared->magic == MOTION_IPC_MAGIC &&
            g_ipc_shared->version == MOTION_IPC_VERSION)
        {
            break;
        }

        usleep(MOTION_IPC_RETRY_US);
        elapsed_ms += 100;
    }

    if (g_ipc_shared->magic != MOTION_IPC_MAGIC)
    {
        printf("motion_ipc: invalid shared memory magic\n");
        motion_ipc_close_handles();
        return -1;
    }

    elapsed_ms = 0;

    while (elapsed_ms <= timeout_ms)
    {
        g_ipc_frame_sem = sem_open(
            MOTION_IPC_FRAME_SEM_NAME,
            0);

        if (g_ipc_frame_sem != SEM_FAILED)
            break;

        usleep(MOTION_IPC_RETRY_US);
        elapsed_ms += 100;
    }

    if (g_ipc_frame_sem == SEM_FAILED)
    {
        printf("motion_ipc: consumer frame sem timeout, "
               "errno=%d\n",
               errno);

        motion_ipc_close_handles();
        return -1;
    }

    elapsed_ms = 0;

    while (elapsed_ms <= timeout_ms)
    {
        g_ipc_event_sem = sem_open(
            MOTION_IPC_EVENT_SEM_NAME,
            0);

        if (g_ipc_event_sem != SEM_FAILED)
            break;

        usleep(MOTION_IPC_RETRY_US);
        elapsed_ms += 100;
    }

    if (g_ipc_event_sem == SEM_FAILED)
    {
        printf("motion_ipc: consumer event sem timeout, "
               "errno=%d\n",
               errno);

        motion_ipc_close_handles();
        return -1;
    }

    g_ipc_shared->analyzer_alive = 1;
    motion_ipc_memory_barrier();

    g_last_frame_sequence = 0;
    g_ipc_is_consumer = K_TRUE;

    printf("motion_ipc: consumer connected, image=%ux%u\n",
           g_ipc_shared->width,
           g_ipc_shared->height);

    return 0;
}


int motion_ipc_consumer_get_info(k_u32 *width,
                                 k_u32 *height)
{
    if (!g_ipc_is_consumer ||
        !g_ipc_shared ||
        !width ||
        !height)
    {
        return -1;
    }

    *width = g_ipc_shared->width;
    *height = g_ipc_shared->height;

    return 0;
}


int motion_ipc_wait_frame(k_u8 *gray,
                          k_u32 gray_capacity,
                          k_u8 *yuv420sp,
                          k_u32 yuv_capacity,
                          k_u32 *frame_number)
{
    k_u32 sequence_before;
    k_u32 sequence_after;
    k_u32 index_before;
    k_u32 index_after;
    int retry;
    int ret;

    if (!g_ipc_is_consumer ||
        !g_ipc_shared ||
        !gray ||
        !yuv420sp ||
        gray_capacity < MOTION_IPC_GRAY_SIZE ||
        yuv_capacity < MOTION_IPC_YUV_SIZE)
    {
        return -1;
    }

    do
    {
        ret = sem_wait(g_ipc_frame_sem);
    }
    while (ret != 0 && errno == EINTR);

    if (ret != 0)
    {
        printf("motion_ipc: sem_wait failed, errno=%d\n",
               errno);
        return -1;
    }

    if (g_ipc_shared->stop_requested ||
        !g_ipc_shared->producer_alive)
    {
        return 0;
    }

    for (retry = 0; retry < 3; ++retry)
    {
        sequence_before =
            g_ipc_shared->frame_sequence;

        index_before =
            g_ipc_shared->frame_index & 1U;

        motion_ipc_memory_barrier();

        memcpy(
            gray,
            g_ipc_shared->frame_gray_buffers[index_before],
            MOTION_IPC_GRAY_SIZE);

        memcpy(
            yuv420sp,
            g_ipc_shared->frame_yuv_buffers[index_before],
            MOTION_IPC_YUV_SIZE);

        motion_ipc_memory_barrier();

        sequence_after =
            g_ipc_shared->frame_sequence;

        index_after =
            g_ipc_shared->frame_index & 1U;

        if (sequence_before == sequence_after &&
            index_before == index_after)
        {
            if (sequence_after ==
                g_last_frame_sequence)
            {
                return 1;
            }

            g_last_frame_sequence =
                sequence_after;

            if (frame_number)
            {
                *frame_number =
                    g_ipc_shared->frame_number;
            }

            return 1;
        }
    }

    printf("motion_ipc: unstable frame copy\n");
    return -1;
}


int motion_ipc_publish_event(const k_u8 *yuv420sp,
                             k_u32 frame_number)
{
    k_u32 next_index;
    k_u32 sequence;

    if (!g_ipc_is_consumer ||
        !g_ipc_shared ||
        !yuv420sp)
    {
        return -1;
    }

    next_index =
        (g_ipc_shared->event_index + 1U) & 1U;

    memcpy(
        g_ipc_shared->event_yuv_buffers[next_index],
        yuv420sp,
        MOTION_IPC_YUV_SIZE);

    motion_ipc_memory_barrier();

    sequence =
        g_ipc_shared->event_sequence + 1U;

    g_ipc_shared->event_frame_number = frame_number;
    g_ipc_shared->event_index = next_index;
    g_ipc_shared->event_sequence = sequence;

    motion_ipc_memory_barrier();

    motion_ipc_coalesced_post(g_ipc_event_sem);

    return 0;
}


void motion_ipc_consumer_close(void)
{
    if (!g_ipc_is_consumer)
        return;

    if (g_ipc_shared)
    {
        g_ipc_shared->analyzer_alive = 0;
        motion_ipc_memory_barrier();
    }

    motion_ipc_close_handles();

    g_ipc_is_consumer = K_FALSE;

    printf("motion_ipc: consumer closed\n");
}
