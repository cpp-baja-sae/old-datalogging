#define PIN_SPI_SS 25    // GPIO pin for SPI slave select.
#define PIN_SPI_CLK 5    // GPIO pin for SPI clock.
#define PIN_SPI_MOSI 12  // GPIO pin for master output / slave input
#define PIN_SPI_MISO0 4  // GPIO pin for master input / slave output for ADC #0
#define PIN_SPI_MISO1 6  // GPIO pin for master input / slave output for ADC #1
#define PIN_SPI_MISO2 7  // GPIO pin for master input / slave output for ADC #2
#define PIN_SPI_MISO3 8  // GPIO pin for master input / slave output for ADC #3
#define PIN_SPI_MISO4 9  // GPIO pin for master input / slave output for ADC #4
#define PIN_SPI_MISO5 10 // GPIO pin for master input / slave output for ADC #5
#define PIN_SPI_MISO6 11 // GPIO pin for master input / slave output for ADC #6
#define PIN_SPI_MISO7 12 // GPIO pin for master input / slave output for ADC #7

#define OUTPUT_LENGTH 10 // How many bits should be collected per reading.
#define START_BIT 8      // Bit index where the actual value starts.
#define END_BIT (START_BIT + OUTPUT_LENGTH - 1)

#define NUM_ADCS 2       // Number of analog->digital converters.
#define NUM_CHANNELS 8   // How many channels to poll from each ADC.
// BATCH_SIZE must be a factor of FILE_BLOCK_SIZE
#define BATCH_SIZE (NUM_ADCS * NUM_CHANNELS * 2) // 2 bytes for every value.
#define FOR_EVERY_ADC(MACRO) \
    MACRO(0) \
    MACRO(1) \
    // MACRO(2) \
    // MACRO(3) \
    // MACRO(4) \
    // MACRO(5) \
    // MACRO(6) \
    // MACRO(7) 

// How many times to read every channel from every ADC. Increasing this value
// also increases the space used to store these results, making the whole
// program less buggy in the case of random hiccups.
#define NUM_BATCHES 32
#define NUM_CYCLES (NUM_BATCHES * NUM_CHANNELS)
#define CYCLE_TIME 40    // Delay in microseconds between each read operation.