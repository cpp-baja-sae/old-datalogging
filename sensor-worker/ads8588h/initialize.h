#include <pigpio.h>
#include <unistd.h>

void initialize_gpio() {
    gpioSetMode(PIN_CS, PI_OUTPUT);
    gpioSetMode(PIN_SCLK, PI_OUTPUT);
    gpioSetMode(PIN_CONVST, PI_OUTPUT);
    gpioSetMode(PIN_RESET, PI_OUTPUT);

    gpioWrite(PIN_RESET, 0);
    usleep(20);
    gpioWrite(PIN_RESET, 1);
    usleep(20);
    gpioWrite(PIN_RESET, 0);
}