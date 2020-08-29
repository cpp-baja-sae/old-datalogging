#ifndef WORKERS_H_
#define WORKERS_H_

extern volatile int continue_flag;
extern volatile int write_to_file_flag;
// Only stream every (var)th frame.
extern volatile int stream_frame_interval;

void *file_worker(void *args);
void *sensor_read_worker(void *args);
void *command_worker(void *args);
void *realtime_stream_worker(void *args);

#endif