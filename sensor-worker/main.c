#include <errno.h>
#include <pigpio.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h> // For sleep command.

#include "config.h"
#include "workers.h"

FILE *primary_file;
int pbuf_read_index = 0;

struct LodInfo {
    int min_buffer[FRAME_LEN];
    int max_buffer[FRAME_LEN];
    int avg_buffer[FRAME_LEN];
    // How many frames have been accumulated in the three buffers.
    int progress; 

    FILE *target_file;
    char file_buffer[FILE_BUFFER_SIZE];
    int fbuf_read_index;
    int fbuf_write_index;
    // LODs which have a lot of data will use a larger block size to be more
    // efficient. LODs which have less data will use a smaller block size to be
    // updated more often.
    int file_write_block_size;
} lod_infos[NUM_AUX_LODS];

void begin_data_log_file() {
    printf("[WRITER] Starting new file.\n");

    uint64_t timestamp = (uint64_t) time(NULL);
    // 400 is kind of overkill, just being safe.
    char folder_name[400], file_name[400]; 
    sprintf(folder_name, "%s/datalog_%.20lli", OUTPUT_DIR, timestamp);
    mkdir(folder_name, 0755);

    // Write basic information.
    sprintf(file_name, "%s/info.json", folder_name);
    FILE *info_file = fopen(file_name, "wb");
    fprintf(info_file, "{");
    fprintf(info_file, "\n\t\"version\": %i,", 0);
    fprintf(info_file, "\n\t\"num_adcs\": %i,", NUM_ADCS);
    fprintf(info_file, "\n\t\"num_channels\": %i,", NUM_CHANNELS);
    fprintf(info_file, "\n\t\"cycle_time_us\": %i,", CYCLE_TIME);
    // Count the primary file in the total LOD count.
    fprintf(info_file, "\n\t\"total_num_lods\": %i,", NUM_AUX_LODS + 1);
    fprintf(info_file, "\n\t\"lod_sample_interval\": %i", LOD_SAMPLE_INTERVAL);
    fprintf(info_file, "\n}\n");
    fclose(info_file);

    // Open a file to write the primary info into.
    sprintf(file_name, "%s/0.bin", folder_name);
    primary_file = fopen(file_name, "wb");

    for (int lod = 0; lod < NUM_AUX_LODS; lod++) {
        // Open an additional file for each LOD.
        sprintf(file_name, "%s/%i.bin", folder_name, lod + 1);
        lod_infos[lod].target_file = fopen(file_name, "wb");
    }
}

// chunk_size must be a factor of FILE_BUFFER_SIZE
void write_new_data_to_file(
    volatile char *source, FILE *target, int *read_index, int write_index, int chunk_size 
) {
    int unwritten_bytes;
    if (*read_index == write_index) {
        unwritten_bytes = 0;
    } else if (write_index > *read_index) {
        unwritten_bytes = write_index - *read_index;
    } else {
        // The write index has looped around to the start of the buffer.
        unwritten_bytes = FILE_BUFFER_SIZE - (*read_index - write_index);
    }
    if (unwritten_bytes > FILE_BUFFER_SIZE / 2) {
        printf(
            "[WRITER] WARNING: A file buffer is kind of full. (%i/%i)\n",
            unwritten_bytes, FILE_BUFFER_SIZE
        );
    }
    while (unwritten_bytes > chunk_size) {
        // We can safely assume the data does not wrap around because the
        // size of the whole buffer is a multiple of chunk_size
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
        fwrite(&source[*read_index], 1, chunk_size, target);
        #pragma GCC diagnostic pop
        unwritten_bytes -= chunk_size;
        *read_index = (*read_index + chunk_size) % FILE_BUFFER_SIZE;
    }
}

void update_files() {
    write_new_data_to_file(
        primary_file_buffer, 
        primary_file, 
        &pbuf_read_index, 
        pbuf_write_index, 
        FILE_BLOCK_SIZE
    );
    for (int lod_index = 0; lod_index < NUM_AUX_LODS; lod_index++) {
        write_new_data_to_file(
            lod_infos[lod_index].file_buffer,
            lod_infos[lod_index].target_file,
            &lod_infos[lod_index].fbuf_read_index,
            lod_infos[lod_index].fbuf_write_index,
            lod_infos[lod_index].file_write_block_size
        );
    }
}

void *file_worker(void *args) {
    while (continue_flag) {
        // Every time write_to_file_flag is set, a new file is created.
        if (write_to_file_flag) {
            begin_data_log_file();
            // Skip to the most currently written frame.
            pbuf_read_index = pbuf_write_index;
            pbuf_read_index -= pbuf_write_index % FRAME_SIZE;
            while (continue_flag && write_to_file_flag) {
                update_files();
                usleep(4000);
            }
            printf("[WRITER] Closing files.\n");
            fclose(primary_file);
            for (int lod_index = 0; lod_index < NUM_AUX_LODS; lod_index++) {
                fclose(lod_infos[lod_index].target_file);
            }
        }
        sleep(1);
    }
    return NULL;
}

void interrupt_handler(int _) {
    continue_flag = 0;
}

#include "ads8588h/initialize.h"
void initialize() {
    if (gpioInitialise() < 0) {
        printf("[SENSOR] Failed to initialize GPIO! Make sure you are ");
        printf("with sudo.\n");
        exit(1);
    }

    initialize_gpio();

    gpioSetSignalFunc(SIGINT, interrupt_handler);

    printf("[SENSOR] Successfully initialized GPIO.\n");
}

void reset_lod_buffer(int lod_index) {
    lod_infos[lod_index].progress = 0;
    for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
        lod_infos[lod_index].min_buffer[val_index] = 0xFFFF;
        lod_infos[lod_index].max_buffer[val_index] = 0;
        lod_infos[lod_index].avg_buffer[val_index] = 0;
    }
}

void commit_lod(int lod_index) {
    // Actually average the averages.
    // We wait to do this step until after we have written every value because
    // it gives us higher precision.
    struct LodInfo *this_lod = &lod_infos[lod_index];
    for (int val_index = 0; val_index < FRAME_LEN; val_index++) {
        (*this_lod).avg_buffer[val_index] /= LOD_SAMPLE_INTERVAL;
    }
    // If this is not the highest-level LOD, then update the next LOD's min, max, and avg.
    if (lod_index < NUM_AUX_LODS - 1) {
        struct LodInfo *next_lod = &lod_infos[lod_index + 1];
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

int main() {
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

    pthread_t file_worker_id;
    pthread_create(&file_worker_id, NULL, file_worker, NULL);
    pthread_t sensor_worker_id;;
    pthread_create(&sensor_worker_id, NULL, sensor_read_worker, NULL);
    pthread_t command_worker_id;;
    pthread_create(&command_worker_id, NULL, command_worker, NULL);
    pthread_t stream_worker_id;;
    pthread_create(&stream_worker_id, NULL, realtime_stream_worker, NULL);
    printf("[MAIN  ] Started all workers.\n");

    pthread_join(file_worker_id, NULL);
    pthread_join(sensor_worker_id, NULL);
    // This worker might hang waiting for connections, we should manually
    // cancel it.
    pthread_cancel(command_worker_id);
    pthread_join(command_worker_id, NULL);
    // Same thing here.
    pthread_cancel(stream_worker_id);
    pthread_join(stream_worker_id, NULL);
    printf("[MAIN  ] All workers shut down successfully.\n");
}
