#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"

// ---------------------------------------------------------------------------------------------
// *** Pico pins ***
// ---------------------------------------------------------------------------------------------
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17 // called SSN by the TDC
#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_INTERRUPT 20
// ---------------------------------------------------------------------------------------------
// *** Configuration Registers ***
// ---------------------------------------------------------------------------------------------
const uint8_t config_register[17] = {0x31, 0x81, 0x9F, 0x40, 0x0D, 0x03, 0xC0, 0x53,
                                     0xA1, 0x13, 0x00, 0x0A, 0xCC, 0xCC, 0x31, 0x8E, 0x04}; // modified
// const char config_register[17] = {0x31, 0x01, 0x1F, 0x40, 0x0D, 0x03, 0xC0, 0x53,
//                                   0xA1, 0x13, 0x00, 0x0A, 0xCC, 0xCC, 0x31, 0x8E, 0x04}; //original
// A typical config settings = { config00, config01, … , config16 }
// ---------------------------------------------------------------------------------------------
// *** SPI Opcodes ***
// ---------------------------------------------------------------------------------------------
const uint8_t spiopc_power = 0x30;        // opcode for "Power on Reset"
const uint8_t spiopc_init = 0x18;         // opcode for "Initialize Chip and Start Measurement"
const uint8_t spiopc_write_config = 0x80; // opcode for "Write Configuration"
const uint8_t spiopc_read_config = 0x40;  // opcode for "Read Configuration"
const uint8_t spiopc_read_results = 0x60; // opcode for "Read Measurement Results"
// ---------------------------------------------------------------------------------------------
// *** SPI Addresses ***
// ---------------------------------------------------------------------------------------------
const uint8_t reference_index_ch1_byte3 = 8; //
const uint8_t reference_index_ch1_byte2 = 9;
const uint8_t reference_index_ch1_byte1 = 10;
const uint8_t stopresult_ch1_byte3 = 11;
const uint8_t stopresult_ch1_byte2 = 12;
const uint8_t stopresult_ch1_byte1 = 13;
// . . . .
const uint8_t stopresult_ch4_byte3 = 29;
const uint8_t stopresult_ch4_byte2 = 30;
const uint8_t stopresult_ch4_byte1 = 31;
// ---------------------------------------------------------------------------------------------
// *** Other Variables ***
// --------------------------------------------------------------------------------------------
uint8_t Buffer[1];            // read buffer variable used to copy the SPI data
uint8_t writeBuffer[1];       // write buffer used to output SPI data
int reference_index[4] = {0}; // reference index data array {Ch1, Ch2, Ch3, Ch4}
int stopresult[4] = {0};      // stop result data array {Ch1, Ch2, Ch3, Ch4}
bool config_error = false;    // flag that indicates if the config registers
bool measure = true;          // flag that indicates if measurements are to be read from the TDC
// note - the flag prevents readout, the TDC will still measure until its FIFO is saturated; those initial few values will then be read out when measurement is restarted

void restart()
{
    watchdog_reboot(0, 0, 0); // reboots the chip
}

int main(void)
{
    // -----------------------------------------------------------------------------------------
    // *** Power on reset ***
    // -----------------------------------------------------------------------------------------
    stdio_init_all();
    char userinput = getchar(); // wait on any character before starting the program
    // communication through USB

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000 * 1000);
    // spi_init(SPI_PORT, 61035);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Initialize interrupt pin
    gpio_set_function(PIN_INTERRUPT, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_INTERRUPT, GPIO_IN);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_put(PIN_CS, 0);

    writeBuffer[0] = spiopc_power;
    spi_write_blocking(SPI_PORT, writeBuffer, 1); // Opcode for "Power On Reset" is sent over SPI

    gpio_put(PIN_CS, 1);
    busy_wait_us(100); // wait for the TDC to reset
    gpio_put(PIN_CS, 0);

    // -----------------------------------------------------------------------------------------
    // *** Writing the configuration registers ***
    // -----------------------------------------------------------------------------------------
    writeBuffer[0] = spiopc_write_config + 00;
    spi_write_blocking(SPI_PORT, writeBuffer, 1); // Opcode for "Write Configuration"
                                                  // and config address (00) are sent over SPI
    for (size_t i = 0; i < 17; i++)               // Send all 17 config registers via SPI
    {
        writeBuffer[0] = config_register[i];
        // printf("Writing %x\n", writeBuffer[0]); //debug
        spi_write_blocking(SPI_PORT, writeBuffer, 1);
    }
    // printf("%d", gpio_get(PIN_INTERRUPT)); //debug
    gpio_put(PIN_CS, 1);
    gpio_put(PIN_CS, 0);

    // -----------------------------------------------------------------------------------------
    // *** Verification of config registers ***
    // -----------------------------------------------------------------------------------------
    writeBuffer[0] = spiopc_read_config + 00;
    spi_write_blocking(SPI_PORT, writeBuffer, 1); // Opcode for "Read Configuration"
                                                  // and config address (00) are sent over SPI
    for (size_t i = 0; i < 17; i++)
    {
        spi_read_blocking(SPI_PORT, 0, Buffer, 1); // read one byte from SPI to Buffer variable
        // printf("reading: %x\n", Buffer[0]); //debug
        if (config_register[i] != Buffer[0])
            config_error = true;
        // if there was a failure in writing the config
        // registers, then the config_error flag is raised.
    }
    //
    // -----------------------------------------------------------------------------------------
    // *** Initialize and start the measurement ***
    // -----------------------------------------------------------------------------------------
    // printf("Error: %d\n", config_error); //debug

    if (config_error == false)
    {
        // printf("Inside the correct config\n"); // debug

        gpio_put(PIN_CS, 1);
        gpio_put(PIN_CS, 0);

        writeBuffer[0] = spiopc_init;
        spi_write_blocking(SPI_PORT, writeBuffer, 1); // Opcode for "Initialize" is sent over SPI
        gpio_put(PIN_CS, 1);
        busy_wait_us(100); // wait for the TDC to initialize

        // This is required to start measuring process
        // *************************************************************************************
        // End of the configuration settings. After now the time measurement will start.
        // This code is designed to use SPI to read the measurement data from GPX2.
        // Using LVDS as a output interface requires additional hardware and code.
        // *************************************************************************************
        // -----------------------------------------------------------------------------------------
        // *** Readout of measurement data via SPI ***
        // -----------------------------------------------------------------------------------------

        // printf("%d", gpio_get(PIN_INTERRUPT)); //debug

        while (true)
        {
            if (measure) //if measure flag is set
            {
                while (gpio_get(PIN_INTERRUPT) != 0)
                    tight_loop_contents();
                // wait till the Interrupt pin is low

                // printf("Wow, an interrupt!\n"); //debug

                gpio_put(PIN_CS, 0);

                writeBuffer[0] = spiopc_read_results + reference_index_ch1_byte3; // Opcode for "Read Result" and data address are sent
                spi_write_blocking(SPI_PORT, writeBuffer, 1);

                memset(reference_index, 0, sizeof reference_index); //result arrays are zeroed
                memset(stopresult, 0, sizeof stopresult);
                for (size_t i = 0; i < 4; i++)
                {
                    spi_read_blocking(SPI_PORT, 0, Buffer, 1); // read one byte from SPI to Buffer
                    reference_index[i] = reference_index[i]    // Data is shifted 16 Bits to the left
                                         + (Buffer[0] << 16);  // and added to the reference_index
                    spi_read_blocking(SPI_PORT, 0, Buffer, 1); // read one byte from SPI to Buffer
                    reference_index[i] = reference_index[i]    // Data is shifted 8 Bits to the left
                                         + (Buffer[0] << 8);   // and added to the reference_index
                    spi_read_blocking(SPI_PORT, 0, Buffer, 1); // read one byte from SPI to Buffer
                    reference_index[i] = reference_index[i]    // Data is directly added to reference_index
                                         + Buffer[0];
                    // The complete reference index (3 Bytes)
                    // has been received.
                    spi_read_blocking(SPI_PORT, 0, Buffer, 1); // Same process as reference_index
                    stopresult[i] = stopresult[i]              // is repeated for stop results
                                    + (Buffer[0] << 16);
                    spi_read_blocking(SPI_PORT, 0, Buffer, 1);
                    stopresult[i] = stopresult[i] + (Buffer[0] << 8);
                    spi_read_blocking(SPI_PORT, 0, Buffer, 1);
                    stopresult[i] = stopresult[i] + Buffer[0];
                    // The complete stopresult (3 Bytes)
                    // has been received
                }

                for (size_t i = 0; i < 4; i++) // print stop results
                {
                    printf("stop%d: %d,", i + 1, stopresult[i]);
                }
                for (size_t i = 0; i < 4; i++) // then print reference indeces
                {
                    printf("ref%d: %d,", i + 1, reference_index[i]);
                }
                printf("\n");

                gpio_put(PIN_CS, 1);
            }
            userinput = getchar_timeout_us(0); //non-blocking user read
            if (userinput == 'p') // p pauses measurements
            {
                measure = false;
            }
            else if (userinput == 'r') // r restarts measurements
            {
                measure = true;
            }
            else if (userinput == 'q') // q resets the pico
            {
                restart();
            }
        }
    }
    printf("Outside the correct config\n"); // config failure
}
