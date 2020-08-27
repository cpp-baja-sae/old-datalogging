#include <pigpio.h>

int create_tx_waveform() {
    rawSPI_t spi_info = {
        .clk = PIN_SPI_CLK,
        .mosi = PIN_SPI_MOSI,
        .ss_pol = 1, // Resting value for the slave select pin.
        .ss_us = 1, // How long to wait after switching the slave select value.
        .clk_pol = 0, // Resting value for the clock pin.
        .clk_pha = 0, // 0 indicates sample on 1st clock edge, 1 indicates 2nd edge.
        .clk_us = 1, // Microseconds between clock pulses.
    };

    int time_offset = 0;
    for (int cycle = 0; cycle < NUM_CYCLES; cycle++) {
        int channel = cycle % NUM_CHANNELS;
        char data_buf[1];
        // First bit tells the circuit we want to read.
        // Second bit says we do not want to use differential mode.
        // Next three bits say which channel to read from.
        // Last three bits aren't sent.
        data_buf[0] = 0b11000000 | (channel << 3);
        // Generates a waveform which requests data from the SPI bus.
        rawWaveAddSPI(
            &spi_info,   // Information about which pins to set.
            time_offset, // When to start sending the waveform.
            PIN_SPI_SS,  // Which slave select pin to use.
            data_buf,    // What data to send.
            5,           // How many bits to write from the buffer.
            START_BIT,   // Which received bit to start storing.
            END_BIT,     // Which received bit to stop storing on.
            END_BIT      // Overall, how many bits to read (with/out storing).
        );
        time_offset += CYCLE_TIME;
    }

    // These two pulses do nothing for CYCLE_TIME * NUM_CYCLES microseconds.
    // This ensures that there is still a delay after the last spi transaction
    // before the whole waveform starts over again.
    gpioPulse_t end_padding[2];
    end_padding[0].gpioOn = 0;
    end_padding[0].gpioOff  = 0;
    end_padding[0].usDelay = time_offset;
    // This second pulse is necessary to make the first delay actually occur.
    end_padding[1].gpioOn = 0;
    end_padding[1].gpioOff  = 0;
    end_padding[1].usDelay = 0;
    gpioWaveAddGeneric(2, end_padding);

    int wave_id = gpioWaveCreate();
    if (wave_id < 0) {
        printf("[SENSOR] Failed to create SPI waveform! Try reducing ");
        printf("NUM_BATCHES.\n");
        exit(1);
    }

    printf("[SENSOR] Successfully created SPI waveform.\n");
    return wave_id;
}

void sensor_read_loop() {
    int spi_waveform_id = create_tx_waveform();

    rawWaveInfo_t spi_waveform_info = rawWaveInfo(spi_waveform_id);

    // Control blocks store the output values of the waveform. By checking which
    // control block is currently in use, we can determine how far we have
    // progressed along the waveform. The lowest control block is the start, 
    // the highest is the end.
    int bottom_cb = spi_waveform_info.botCB;
    int num_cbs = spi_waveform_info.numCB;
    int cbs_per_cycle = num_cbs / NUM_CYCLES;

    // OOLs (no idea what it stands for) store values read from pins. Each
    // OOL stores a 32-bit field containing the value of every GPIO pin at
    // the time it was captured. There are 10 OOLs for each cycle since we
    // are capturing 10 bits every cycle. OOLs are allocated from the top down,
    // so the top OOL corresponds to the first bit of the first read.
    int top_ool = spi_waveform_info.topOOL;

    gpioWaveTxSend(spi_waveform_id, PI_WAVE_MODE_REPEAT);

    int reading_from = 0;
    int values[NUM_ADCS];
    while (continue_flag) {
        int current_cycle_in_progress 
            = (rawWaveCB() - bottom_cb) / cbs_per_cycle % NUM_CYCLES;

        int cycles_read = 0;
        while (reading_from != current_cycle_in_progress) {
            for (int adc = 0; adc < NUM_ADCS; adc++) {
                values[adc] = 0;
            }

            int current_ool = top_ool - reading_from * OUTPUT_LENGTH - 1;
            int current_channel = reading_from % NUM_CHANNELS;

            // Take the bit which represents what value the pin had and
            // turn it into the corresponding bit for the output value.
            // (E.G. the third bit of the value will appear in the third
            // value on pin 15, necessitating a right-shift of 12.)
            #define STORE_BIT(ADC_INDEX) \
                if (PIN_SPI_MISO##ADC_INDEX >= bit) { \
                    values[ADC_INDEX] |= \
                        (pin_values & (1 << PIN_SPI_MISO##ADC_INDEX))  \
                        >> (PIN_SPI_MISO##ADC_INDEX - bit); \
                } else { \
                    values[ADC_INDEX] |= \
                        (pin_values & (1 << PIN_SPI_MISO##ADC_INDEX))  \
                        << (bit - PIN_SPI_MISO##ADC_INDEX); \
                }
            for (int bit = OUTPUT_LENGTH - 1; bit >= 0; bit--) {
                uint32_t pin_values = rawWaveGetOut(current_ool);
                FOR_EVERY_ADC(STORE_BIT)
                current_ool--;
            }
            #undef STORE_BIT

            for (int adc = 0; adc < NUM_ADCS; adc++) {
                // Two bytes per value, more than enough precision.
                primary_file_buffer[pbuf_write_index] = (values[adc] & 0x0F00) >> 8;
                primary_file_buffer[pbuf_write_index + 1] = values[adc] & 0x00FF;
                pbuf_write_index = (pbuf_write_index + 2) % FILE_BUFFER_SIZE;

                int lbuf_index = adc * NUM_CHANNELS + current_channel;
                if (lod_infos[0].min_buffer[lbuf_index] > values[adc]) {
                    lod_infos[0].min_buffer[lbuf_index] = values[adc];
                }
                if (lod_infos[0].max_buffer[lbuf_index] < values[adc]) {
                    lod_infos[0].max_buffer[lbuf_index] = values[adc];
                }
                lod_infos[0].avg_buffer[lbuf_index] += values[adc];
            }

            // If we just wrote the last channel...
            if (current_channel == NUM_CHANNELS - 1) {
                lod_infos[0].progress += 1;
                // If we have written enough samples to the first LOD...
                if (lod_infos[0].progress == LOD_SAMPLE_INTERVAL) {
                    // Save the information and propogate it up to higher LODs
                    // as necessary.
                    commit_lod(0);
                }
            }

            reading_from = (reading_from + 1) % NUM_CYCLES;
            cycles_read += 1;
        }
        if (cycles_read > NUM_CYCLES / 2) {
            printf(
                "[SENSOR] WARNING: Data buffer was pretty full. (%i/%i)\n",
                cycles_read, NUM_CYCLES
            );
        }
        usleep(500);
    }
}