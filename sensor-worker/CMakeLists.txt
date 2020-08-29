cmake_minimum_required(VERSION 3.10)

project(crunch C)

add_executable(
  crunch 
  main.c 
  workers.c 
  command_worker.c 
  file_worker.c
  realtime_stream_worker.c
  sensor_read_worker.c
)

# Default communication library for raspi chipset, has low-level functions
# allowing for super-fast reads / writes with the ADC.
target_link_libraries(crunch pigpio)

# Include pthreads library
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(crunch Threads::Threads)