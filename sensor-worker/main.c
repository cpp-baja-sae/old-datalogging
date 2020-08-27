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

static volatile int continue_flag = 1;

#include "ads8588h/defines.h"

#define FILE_BLOCK_SIZE 4096 // Write to the file in 4k blocks.
#define FILE_BUFFER_SIZE (BATCH_SIZE * FILE_BLOCK_SIZE)
// How many additional versions of the file to create with sequentially lower
// resolutions.
#define NUM_AUX_LODS 7 
// How much the sample rate should be divided for each sequential file.
#define LOD_SAMPLE_INTERVAL 4

// The name of the socket to be created to allow this program to talk to other
// programs on the pi, allowing it to be controlled from outside. This socket
// is used by the web server to change settings and check status. IPC stands
// for Inter-Process Communication.
#define IPC_ID "/tmp/.crunch_ipc"
// Same as above, but this socket is used to stream real-time data. Using two
// separate sockets simplifies communication protocols. The above socket mostly
// does small 32-byte long requests / responses, like a conversation. This
// socket just constantly yells data at whoever wants to listen.
#define IPC_STREAM_ID "/tmp/.crunch_stream"
#define IPC_COMMAND_STOP 'x'
#define IPC_COMMAND_BEGIN_FILE 'b'
#define IPC_COMMAND_END_FILE 'e'
#define IPC_COMMAND_GET_CONFIG 'c'
#define IPC_COMMAND_SET_STREAM_INTERVAL 'i'
// For simplicity's sake, we always send responses of a fixed size in response
// to messages received over the IPC socket.
#define IPC_RESPONSE_SIZE 32

// Must not end with a slash.
const char *output_dir = "/root/datalogs";

FILE *primary_file;
char primary_file_buffer[FILE_BUFFER_SIZE];
int pbuf_read_index = 0;
int pbuf_stream_index = 0;
int pbuf_write_index = 0;

struct LodInfo {
    int min_buffer[NUM_ADCS * NUM_CHANNELS];
    int max_buffer[NUM_ADCS * NUM_CHANNELS];
    int avg_buffer[NUM_ADCS * NUM_CHANNELS];
    // How many batches have been accumulated in the three buffers.
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

int write_to_file_flag = 1;
int stream_batch_interval = 100; // Only stream every [var]th batch.

void begin_data_log_file() {
    printf("[WRITER] Starting new file.\n");

    uint64_t timestamp = (uint64_t) time(NULL);
    // 400 is kind of overkill, just being safe.
    char folder_name[400], file_name[400]; 
    sprintf(folder_name, "%s/datalog_%.20lli", output_dir, timestamp);
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
    char *source, FILE *target, int *read_index, int write_index, int chunk_size 
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
        fwrite(&source[*read_index], 1, chunk_size, target);
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
            // Skip to the most currently written batch.
            pbuf_read_index = pbuf_write_index;
            pbuf_read_index -= pbuf_stream_index % BATCH_SIZE;
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
    for (int val_index = 0; val_index < NUM_ADCS * NUM_CHANNELS; val_index++) {
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
    for (int val_index = 0; val_index < NUM_CHANNELS * NUM_ADCS; val_index++) {
        (*this_lod).avg_buffer[val_index] /= LOD_SAMPLE_INTERVAL;
    }
    // If this is not the highest-level LOD, then update the next LOD's min, max, and avg.
    if (lod_index < NUM_AUX_LODS - 1) {
        struct LodInfo *next_lod = &lod_infos[lod_index + 1];
        for (int val_index = 0; val_index < NUM_CHANNELS * NUM_ADCS; val_index++) {
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
    // Write new values to file buffer. We have to separate them into three batches to make sure
    // it writes correctly to the file buffer. The file buffer size is a multiple of two, but it
    // is not a multiple of six, so we cannot write them all at once.
    for (int val_index = 0; val_index < NUM_CHANNELS * NUM_ADCS; val_index++) {
        int write_index = (*this_lod).fbuf_write_index;
        (*this_lod).file_buffer[write_index + 0] = (*this_lod).min_buffer[val_index] >> 8;
        (*this_lod).file_buffer[write_index + 1] = (*this_lod).min_buffer[val_index] & 0xFF;
        (*this_lod).fbuf_write_index = (write_index + 2) % FILE_BUFFER_SIZE;
    }
    for (int val_index = 0; val_index < NUM_CHANNELS * NUM_ADCS; val_index++) {
        int write_index = (*this_lod).fbuf_write_index;
        (*this_lod).file_buffer[write_index + 0] = (*this_lod).max_buffer[val_index] >> 8;
        (*this_lod).file_buffer[write_index + 1] = (*this_lod).max_buffer[val_index] & 0xFF;
        (*this_lod).fbuf_write_index = (write_index + 2) % FILE_BUFFER_SIZE;
    }
    for (int val_index = 0; val_index < NUM_CHANNELS * NUM_ADCS; val_index++) {
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

void run_command(
    char command, 
    char *message, 
    int message_length, 
    char **response, 
    int *response_length
) {
    *response_length = IPC_RESPONSE_SIZE;
    *response = malloc(IPC_RESPONSE_SIZE);
    memset(*response, '\0', IPC_RESPONSE_SIZE);
    (*response)[0] = command;

    switch (command) {
    case IPC_COMMAND_STOP:
        printf("[SOCKET] Received stop command.\n");
        continue_flag = 0;
        break;
    case IPC_COMMAND_BEGIN_FILE:
        printf("[SOCKET] Received begin file command.\n");
        write_to_file_flag = 1;
        break;
    case IPC_COMMAND_END_FILE:
        printf("[SOCKET] Received end file command.\n");
        write_to_file_flag = 0;
        break;
    case IPC_COMMAND_GET_CONFIG:
        printf("[SOCKET] Received get config command.\n");
        // Response format:
        // 1: NUM_ADCS
        // 2: NUM_CHANNELS (per adc)
        // 3: 0x01 if currently writing to a file.
        // 4-5: stream_batch_interval
        (*response)[1] = NUM_ADCS;
        (*response)[2] = NUM_CHANNELS;
        (*response)[3] = write_to_file_flag;
        (*response)[4] = stream_batch_interval >> 8;
        (*response)[5] = stream_batch_interval % 0xFF;
        break;
    case IPC_COMMAND_SET_STREAM_INTERVAL:
        printf("[SOCKET] Received change stream interval command.\n");
        // 1 byte for command, 2 bytes for value to change to.
        if (message_length == 3) {
            stream_batch_interval = message[0] << 8 | message[1];
        } else {
            printf(
                "[SOCKET] Error, message length must be 3 for this command.\n"
            );
        }
        break;
    default:
        printf("[SOCKET] Invalid IPC command: %c.\n", command);
        break;
    }
}

void *socket_worker(void *args) {
    int socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        printf("[SOCKET] Failed to create socket.\n");
        exit(1);
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr)); // Clear everything.
    addr.sun_family = AF_UNIX; // Local Unix socket, not an inet socket.
    // Set the socket's path to be IPC_ID.
    strncpy(addr.sun_path, IPC_ID, sizeof(addr.sun_path) - 1);
    // Unbind any existing socket. This is bad practice if more than one
    // instance of the application is running at a time, but we won't be doing
    // that.
    unlink(IPC_ID);
    // Bind the socket that was created to the address we just specified.
    int result = bind(socket_fd, (struct sockaddr*) &addr, sizeof(addr));
    if (result < 0) {
        printf("[SOCKET] Failed to bind socket, error code %i.\n", errno);
        exit(1);
    }
    
    // Allow up to 5 clients to wait to connect.
    if (listen(socket_fd, 5) < 0) {
        printf("[SOCKET] Failed to listen(), error code %i.\n", errno);
        exit(1);
    }

    printf("[SOCKET] Setup complete.\n");

    char buffer[1];
    while (continue_flag) {
        struct sockaddr_un client_addr;
        int addr_len = sizeof(client_addr);
        printf("[SOCKET] Waiting for connection.\n");
        int client_fd = accept(
            socket_fd, 
            (struct sockaddr*) 
            &client_addr, 
            &addr_len
        );
        if (client_fd < 0) {
            printf("[SOCKET] Bad connection attempt, error code %i.\n", errno);
            // This is a recoverable error.
            continue;
        }
        printf("[SOCKET] Accepted an incoming connection.\n");
        while (continue_flag) {
            // The first byte is the length of the message.
            char length = '\0';
            int result = read(client_fd, &length, 1);
            if (result < 0) {
                printf("[SOCKET] Could not read message from client, ");
                printf("error code %i.\n", errno);
                // The client may have disconnected. Try to connect to a new
                // client.
                break;
            }
            int ilength = (int) length;
            if (ilength <= 0) {
                printf("[SOCKET] Messages cannot be zero bytes long.\n");
                // It is possible the client disconnected.
                break;
            }

            char *message = (char*) malloc(ilength);
            int real_length = read(client_fd, message, ilength);
            if (real_length < 0) {
                printf("[SOCKET] Could not read message from client, ");
                printf("error code %i.\n", errno);
                free(message);
                break;
            }
            if (real_length != ilength) {
                printf(
                    "[SOCKET] Expected %i bytes, but got %i bytes.\n",
                    ilength,
                    real_length
                );
                free(message);
                continue;
            }

            // First byte of the message is which command to use.
            char command = message[0];
            char *response;
            int response_length;
            run_command(command, message, ilength, &response, &response_length);

            if (write(client_fd, response, response_length) < 0) {
                printf(
                    "[SOCKET] Failed to send response, error code %i.\n",
                    errno
                );
                free(message);
                free(response);
                break;
            }
            free(message);
            free(response);
        }
        close(client_fd);
    }

    if (close(socket_fd) < 0) {
        printf("[SOCKET] Unable to close socket, error code %i.\n", errno);
        return NULL;
    } 
    if (unlink(IPC_ID) < 0) {
        printf("[SOCKET] Unable to unlink socket, error code %i.\n", errno);
        return NULL;
    } 
    printf("[SOCKET] Successfully cleaned up socket.\n");
}

int send_unstreamed_data(int client_fd) {
    int sample_interval = stream_batch_interval * BATCH_SIZE;
    int unwritten_bytes;
    if (pbuf_stream_index == pbuf_write_index) {
        unwritten_bytes = 0;
    } else if (pbuf_write_index > pbuf_stream_index) {
        unwritten_bytes = pbuf_write_index - pbuf_stream_index;
    } else {
        // The write index has looped around to the start of the buffer.
        unwritten_bytes 
            = FILE_BUFFER_SIZE - (pbuf_stream_index - pbuf_write_index);
    }
    if (unwritten_bytes > FILE_BUFFER_SIZE / 4) {
        printf("[STREAM] WARNING: Having troubles keeping up!\n");
    }

    while (unwritten_bytes > sample_interval) {
        int result 
            = write(client_fd, &primary_file_buffer[pbuf_stream_index], BATCH_SIZE);
        if (result < 0) {
            // We failed to write, the client has probably disconnected.
            return 0;
        }
        pbuf_stream_index 
            = (pbuf_stream_index + sample_interval) % FILE_BUFFER_SIZE;
        unwritten_bytes -= sample_interval;
    }

    return 1; // We are good to continue.
}

void *realtime_stream_worker(void *args) {
    int socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        printf("[STREAM] Failed to create socket.\n");
        exit(1);
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, IPC_STREAM_ID, sizeof(addr.sun_path) - 1);
    unlink(IPC_STREAM_ID);
    int result = bind(socket_fd, (struct sockaddr*) &addr, sizeof(addr));
    if (result < 0) {
        printf("[STREAM] Failed to bind socket, error code %i.\n", errno);
        exit(1);
    }
    
    if (listen(socket_fd, 5) < 0) {
        printf("[STREAM] Failed to listen(), error code %i.\n", errno);
        exit(1);
    }

    printf("[STREAM] Setup complete.\n");

    while (continue_flag) {
        struct sockaddr_un client_addr;
        int addr_len = sizeof(client_addr);
        printf("[STREAM] Waiting for connection.\n");
        int client_fd = accept(
            socket_fd, 
            (struct sockaddr*) 
            &client_addr, 
            &addr_len
        );
        if (client_fd < 0) {
            printf("[STREAM] Bad connection attempt, error code %i.\n", errno);
            continue;
        }
        printf("[STREAM] Accepted an incoming connection.\n");
        // Skip to the most currently written batch.
        pbuf_stream_index = pbuf_write_index;
        pbuf_stream_index -= pbuf_stream_index % BATCH_SIZE;
        while (continue_flag && send_unstreamed_data(client_fd)) {
            usleep(1000);
        }
        close(client_fd);
    }

    if (close(socket_fd) < 0) {
        printf("[STREAM] Unable to close socket, error code %i.\n", errno);
        return NULL;
    } 
    if (unlink(IPC_STREAM_ID) < 0) {
        printf("[STREAM] Unable to unlink socket, error code %i.\n", errno);
        return NULL;
    } 
    printf("[STREAM] Successfully cleaned up socket.\n");
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
    // Set up the block sizes for each lod. LOD 6 writes about 1 batch every
    // second. 
    lod_infos[0].file_write_block_size = FILE_BLOCK_SIZE;
    lod_infos[1].file_write_block_size = FILE_BLOCK_SIZE;
    lod_infos[2].file_write_block_size = FILE_BLOCK_SIZE;
    lod_infos[3].file_write_block_size = BATCH_SIZE * 16;
    lod_infos[4].file_write_block_size = BATCH_SIZE * 4;
    lod_infos[5].file_write_block_size = BATCH_SIZE;
    lod_infos[6].file_write_block_size = BATCH_SIZE;

    pthread_t file_worker_id;
    pthread_create(&file_worker_id, NULL, file_worker, NULL);
    pthread_t sensor_worker_id;;
    pthread_create(&sensor_worker_id, NULL, sensor_read_worker, NULL);
    pthread_t socket_worker_id;;
    pthread_create(&socket_worker_id, NULL, socket_worker, NULL);
    pthread_t stream_worker_id;;
    pthread_create(&stream_worker_id, NULL, realtime_stream_worker, NULL);
    printf("[MAIN  ] Started all workers.\n");

    pthread_join(file_worker_id, NULL);
    pthread_join(sensor_worker_id, NULL);
    // This worker might hang waiting for connections, we should manually
    // cancel it.
    pthread_cancel(socket_worker_id);
    pthread_join(socket_worker_id, NULL);
    // Same thing here.
    pthread_cancel(stream_worker_id);
    pthread_join(stream_worker_id, NULL);
    printf("[MAIN  ] All workers shut down successfully.\n");
}
