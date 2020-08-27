#include <pigpio.h>

void initialize_gpio() {
    gpioSetMode(PIN_SPI_CLK, PI_OUTPUT);
    gpioSetMode(PIN_SPI_MOSI, PI_OUTPUT);
    gpioSetMode(PIN_SPI_SS, PI_OUTPUT);
}