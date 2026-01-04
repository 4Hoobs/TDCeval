#include "pico/stdlib.h"
#include "hardware/spi.h"
#include<stdio.h>
#include<stdbool.h>

//pin definitions-adjust to wiring
#define SPI_PORT spi0
#define PIN_SPI_SCK 18 //SPI clock (SCK)
#define PIN_SPI_MOSI 19 //SPI MOSI(controller->GPX2)
#define PIN_SPI_MISO 16 //SPI MISO(GPX2->controller)
#define PIN_SPI_CS 17 //chip select (SSN) 
#define PIN_GPX_INT 20 //GPX2 interrupt output

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
    0x04
};
//frequency config
void gpx2_set_refclk_divisions(uint32_t divisions){
    //divisions must fit in 20bits
    divisions &=0XFFFFF;
    //config[3]=lowest 8bits
    gpx2_config[3]=(divisions & 0xFF);
    //config[4]=middle 8bits
    gpx2_config[4]=((divisions>>8)&0xFF);
    //config[5]=upper 4bits
    gpx2_config[5]&=0xF0; //clear lower 4 bits
    gpx2_config[5]|=((divisions>>16)&0x0F);
}
//helper:calculate frequency
uint32_t gpx2_compute_divisions_from_freq(uint32_t freq_hz){
    //period in ps =1e12/freq
    return(1000000000000ULL / freq_hz);
}
//helper:control chip select (SSN)

static inline void gpx2_cs_low(void){
    gpio_put(PIN_SPI_CS, 0); //drive CS low->select GPX2
}
static inline void gpx2_cs_high(void){
    gpio_put(PIN_SPI_CS, 1); //drive CS high->deselect GPX2
}


//send one byte over spi(blocking)
static void gpx2_spi_send_byte(uint8_t value){
    spi_write_blocking (SPI_PORT, &value, 1);
}
//read one byte over spi (blocking)
static uint8_t gpx2_spi_read_byte(void){
    uint8_t rx=0;
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1) ;
    return rx;
}
//write configuration registers to GPX2

static bool gpx2_write_config(const uint8_t *cfg){
    gpx2_cs_low();

    //opcode for write config+start address
    gpx2_spi_send_byte(OPC_WRITE_CONFIG+0x00);

    //send all 17 config bytes
    for (int i=0; i<17; i++){
        gpx2_spi_send_byte(cfg[i]);
    }
    gpx2_cs_high();
    // sleep_us(5);
    return true;
}
//read back and verify config. registers
static bool gpx2_verify_config(const uint8_t *cfg){
    gpx2_cs_low();

    //opcode for read config. +start address (0x00)
    gpx2_spi_send_byte(OPC_READ_CONFIG+0x00);
    //read 17 bytes and compare to local config.
    uint8_t rx;
    for (int i=0; i<17; i++){
        rx=gpx2_spi_read_byte();
        if (rx !=cfg[i]){
            gpx2_cs_high();
            return false;
        }
    }
    gpx2_cs_high();
    return true;
}
//send initialize and start measurement
static void gpx2_start_measurement(void){
    gpx2_cs_low();
    gpx2_spi_send_byte(OPC_INIT);
    gpx2_cs_high();
    busy_wait_us(100);
}
//read 24bit value from SPI, GPX2 send values as 3-byte big-endian

static uint32_t gpx2_read_24bit(void){
    uint32_t value=0;

    value|=((uint32_t)gpx2_spi_read_byte())<<16;
    value|=((uint32_t)gpx2_spi_read_byte())<<8;
    value|=((uint32_t)gpx2_spi_read_byte());

    return value;
}
//read measurement results for 4 channels
static void gpx2_read_results(uint32_t reference_index[4],
                              uint32_t stop_results[4]){
    //wait until gpx2 puls interrupt pin low
    while (gpio_get(PIN_GPX_INT) !=0){
        tight_loop_contents(); //small idle loop
    }
    gpx2_cs_low();
    gpx2_spi_send_byte(OPC_READ_RESULTS+8);

    for (int ch=0; ch<4; ch++){
        //3 bytes reference index
        reference_index[ch]=gpx2_read_24bit();
        //3 bytes stop results
        stop_results[ch]=gpx2_read_24bit();
    }
    gpx2_cs_high();
                              }

typedef enum{
    GPX2_HIRES_OFF=0,
    GPX2_HIRES_2X=1,
    GPX2_HIRES_4X=2
} gpx2_hires_mode_t;

static void gpx2_set_hires(gpx2_hires_mode_t mode){
    //mask for bits 6-7
    const uint8_t HIRES_MASK=0xC0; //1100 0000
    //clear bits 6-7
    gpx2_config[1] &= ~HIRES_MASK;
    //insert new mode(shifted into bits 6-7)
    gpx2_config[1] |= (mode<<6);
}


//main
int main(){
    stdio_init_all(); //enable usb serial output
    char userinput = getchar();
    //initialize SPI hardware
    spi_init(SPI_PORT, 4*1000*1000); //SPI_PORT at 1MHz
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);

    //initialie chip select pin
    gpio_init(PIN_SPI_CS);
    gpio_set_function(PIN_SPI_CS, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpx2_cs_high();

    //initialize interrupt pin
    gpio_init(PIN_GPX_INT);
    gpio_set_function(PIN_GPX_INT, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_GPX_INT, GPIO_IN);

    // sleep_ms(10); //small delay after power up

    //power-on reset command
    gpx2_cs_low();
    gpx2_spi_send_byte(OPC_POWER_RESET);
    gpx2_cs_high();
    busy_wait_us(100);
    // sleep_ms(5);
    
    //set frequency
    uint32_t divisions = gpx2_compute_divisions_from_freq(5000000); // 5 MHz
    gpx2_set_refclk_divisions(divisions);

    //modify config
    gpx2_set_hires(GPX2_HIRES_OFF); //_2X or _4X

    //write config to GPX2
    gpx2_write_config(gpx2_config);
    // if (!){
    //     printf("Failed to write config\n");
    //     while(1){tight_loop_contents();}
    // }
    //verify config
    if (!gpx2_verify_config(gpx2_config)){
        printf("Config verification failes\n");
        while (1){tight_loop_contents();}
    }
    printf("Config OK, starting measurement...\n");

    //start measurement
    gpx2_start_measurement();

    uint32_t reference_index[4]={0};
    uint32_t stop_results[4]={0};

    while(true){
        //read masurement results
        gpx2_read_results(reference_index, stop_results);

        //print results for all 4 channels
        for(int ch=0; ch<4; ch++){
            printf("CH%d: REF=%lu   STOP=%lu\n", 
            ch + 1,
            (unsigned long)reference_index[ch],
            (unsigned long)stop_results[ch]);
        }
        //wait before next read
        // sleep_ms(100);
    }
    return 0;
}

