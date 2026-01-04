#include "pico/stdlib.h"
#include "hardware/spi.h"
#include<studio.h>

//pin definitions-adjust to wiring
#define PIN_SPI_SCK 18 //SPI clock (SCK)
#define PIN_SPI_MOSI 19 //SPI MOSI(controller->GPX2)
#define PIN_SPI_MISO 16 //SPI MISO(GPX2->controller)
#define PIN_SPI_CS 17 //chip select (SSN) 
#define PIN_GPZ_INT 15 //GPX2 interrupt output

//spi opcodes for tdc-gpx2
#define OPC_POWER_RESET 0x30 //power-on reset
#define OPC_INIT 0x18  //initialize chip and start measurement
#define OPC_WRITE_CONFIG 0x80 //write configuration
#define OPC_READ_CONFIG 0x40 //read configuration
#define OPC_READ_RESULTS 0x60 //read measurement results

//configuration registers, 17 bytes, from datasheet
static uint8_t gpx2_config[17]={
    0x31, 0x01, 0x1F, 0x40,
    0x0D, 0x03, 0xC0, 0x53,
    0xA1, 0x13, 0x00, 0x0A, 
    0xCC, 0xCC, 0x31, 0x8E,
};

//helper:control chip select (SSN)

static inline void gpx2_cs_low(void){
    gpio_put(PIN_SPI_CS, 0); //drive CS low->select GPX2
}
static inline void gpx2_cs_high(void){
    gpio_put(PIN_SPI_CS, 1); //drive CS high->deselect GPX2
}
