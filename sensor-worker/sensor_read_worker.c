#include <pigpio.h>
#include <signal.h>
#include <stdlib.h>

#include "config.h"
#include "workers.h"

void interrupt_handler(int _) {
    continue_flag = 0;
}

void reset_lod_buffer(int lod_index) {
    lod_infos[lod_index].progress = 0;
    for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
        lod_infos[lod_index].min_buffer[val_index] = 0xFFFF;
        lod_infos[lod_index].max_buffer[val_index] = 0;
        lod_infos[lod_index].avg_buffer[val_index] = 0;
    }
}

#include "ads8588h/initialize.h"
void initialize() {
    // Ensure that all these values are initialized correctly without garbage
    // values.
    for (int lod = 0; lod < NUM_AUX_LODS; lod++) {
        lod_infos[lod].progress = 0;
        lod_infos[lod].fbuf_read_index = 0;
        lod_infos[lod].fbuf_write_index = 0;
        reset_lod_buffer(lod);
    }
    // Set up the block sizes for each lod. LOD 6 writes about 1 frame every
    // second. 
    lod_infos[0].file_write_block_size = FILE_BLOCK_SIZE;
    lod_infos[1].file_write_block_size = FILE_BLOCK_SIZE;
    lod_infos[2].file_write_block_size = FILE_BLOCK_SIZE;
    lod_infos[3].file_write_block_size = FRAME_SIZE * 16;
    lod_infos[4].file_write_block_size = FRAME_SIZE * 4;
    lod_infos[5].file_write_block_size = FRAME_SIZE;
    lod_infos[6].file_write_block_size = FRAME_SIZE;

    if (gpioInitialise() < 0) {
        printf("[SENSOR] Failed to initialize GPIO! Make sure you are ");
        printf("with sudo.\n");
        exit(1);
    }

    initialize_gpio();

    gpioSetSignalFunc(SIGINT, interrupt_handler);

    printf("[SENSOR] Successfully initialized GPIO.\n");
}

void commit_lod(int lod_index) {
    // Actually average the averages.
    // We wait to do this step until after we have written every value because
    // it gives us higher precision.
    volatile struct LodInfo *this_lod = &lod_infos[lod_index];
    for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
        (*this_lod).avg_buffer[val_index] /= LOD_SAMPLE_INTERVAL;
    }
    // If this is not the highest-level LOD, then update the next LOD's min, max, and avg.
    if (lod_index < NUM_AUX_LODS - 1) {
        volatile struct LodInfo *next_lod = &lod_infos[lod_index + 1];
        for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
            if (this_lod->min_buffer[val_index] < next_lod->min_buffer[val_index]) {
                next_lod->min_buffer[val_index] = this_lod->min_buffer[val_index];
            }
            if (this_lod->max_buffer[val_index] > next_lod->max_buffer[val_index]) {
                next_lod->max_buffer[val_index] = this_lod->max_buffer[val_index];
            }
            next_lod->avg_buffer[val_index] += this_lod->avg_buffer[val_index];
        }
        next_lod->progress += 1;
        if (next_lod->progress == LOD_SAMPLE_INTERVAL) {
            commit_lod(lod_index + 1);
        }
    }
    // Write new values to file buffer. We have to separate them into three frames to make sure
    // it writes correctly to the file buffer. The file buffer size is a multiple of two, but it
    // is not a multiple of six, so we cannot write them all at once.
    for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
        int write_index = (*this_lod).fbuf_write_index;
        (*this_lod).file_buffer[write_index + 0] = (*this_lod).min_buffer[val_index] >> 8;
        (*this_lod).file_buffer[write_index + 1] = (*this_lod).min_buffer[val_index] & 0xFF;
        (*this_lod).fbuf_write_index = (write_index + 2) % FILE_BUFFER_SIZE;
    }
    for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
        int write_index = (*this_lod).fbuf_write_index;
        (*this_lod).file_buffer[write_index + 0] = (*this_lod).max_buffer[val_index] >> 8;
        (*this_lod).file_buffer[write_index + 1] = (*this_lod).max_buffer[val_index] & 0xFF;
        (*this_lod).fbuf_write_index = (write_index + 2) % FILE_BUFFER_SIZE;
    }
    for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
        int write_index = (*this_lod).fbuf_write_index;
        (*this_lod).file_buffer[write_index + 0] = (*this_lod).avg_buffer[val_index] >> 8;
        (*this_lod).file_buffer[write_index + 1] = (*this_lod).avg_buffer[val_index] & 0xFF;
        (*this_lod).fbuf_write_index = (write_index + 2) % FILE_BUFFER_SIZE;
    }
    // Reset the buffer to a state where new values can be written to it.
    reset_lod_buffer(lod_index);
}

void use_realtime_priority() {
    pthread_t this_thread = pthread_self();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    int success = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (success != 0) {
        printf("[SENSOR] Failed to switch to realtime priority.\n");
        exit(1);
    }
    int policy = 0;
    success = pthread_getschedparam(this_thread, &policy, &params);
    if (success != 0 || policy != SCHED_FIFO) {
        printf("[SENSOR] Failed to switch to realtime priority.\n");
        exit(1);
    }
    printf("[SENSOR] Successfully switched to realtime priority.\n");
}

#include "ads8588h/loop.h"
void *sensor_read_worker(void *args) {
    // Change to realtime priority to ensure we can get sensor data as soon as it is available, and
    // avoid the tiny data buffer provided by the pi from filling up.
    use_realtime_priority();
    initialize();

    sensor_read_loop();

    gpioTerminate();
}