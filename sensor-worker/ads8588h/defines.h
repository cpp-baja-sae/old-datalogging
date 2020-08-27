#define PIN_CS 26      // GPIO pin for chip select.
#define PIN_SCLK 21    // GPIO pin for serial data clock.
#define PIN_CONVST 16  // GPIO pin for conversion start.
#define PIN_RESET 20   // GPIO pin for resetting all ADCs.
#define PIN_DOUTA0 2   // GPIO pin for DOUTA on ADC 0.
#define PIN_DOUTB0 4   // GPIO pin for DOUTB on ADC 0.
#define PIN_DOUTA1 17  // GPIO pin for DOUTA on ADC 1.
#define PIN_DOUTB1 22  // GPIO pin for DOUTB on ADC 1.
#define PIN_DOUTA2 10  // GPIO pin for DOUTA on ADC 2.
#define PIN_DOUTB2 11  // GPIO pin for DOUTB on ADC 2.
#define PIN_DOUTA3 0   // GPIO pin for DOUTA on ADC 3.
#define PIN_DOUTB3 6   // GPIO pin for DOUTB on ADC 3.
#define PIN_UNUSED 7


#define NUM_ADCS 4      // Number of analog->digital converters.
#define NUM_CHANNELS 8  // How many channels exist on each ADC.
// BATCH_SIZE must be a factor of FILE_BLOCK_SIZE
#define BATCH_SIZE (NUM_ADCS * NUM_CHANNELS * 2) // 2 bytes for every value.
#define FOR_EVERY_ADC(MACRO) \
    MACRO(0) \
    MACRO(1) \
    MACRO(2) \
    MACRO(3) \
    // MACRO(4) \
    // MACRO(5) \
    // MACRO(6) \
    // MACRO(7) 

// How many times to read every channel from every ADC. Increasing this value
// also increases the space used to store these results, making the whole
// program less buggy in the case of random hiccups.
#define NUM_BATCHES 16
#define NUM_CYCLES NUM_BATCHES
#define CYCLE_TIME 500   // Delay in microseconds between each read operation.
