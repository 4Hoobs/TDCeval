#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <stdbool.h>
#include "hardware/watchdog.h"

// pin definitions-adjust to wiring
#define SPI_PORT spi0
#define PIN_SPI_SCK 18  // SPI clock (SCK)
#define PIN_SPI_MOSI 19 // SPI MOSI(controller->GPX2)
#define PIN_SPI_MISO 16 // SPI MISO(GPX2->controller)
#define PIN_SPI_CS 17   // chip select (SSN)
#define PIN_GPX_INT 20  // GPX2 interrupt output

// spi opcodes for tdc-gpx2
#define OPC_POWER_RESET 0x30  // power-on reset
#define OPC_INIT 0x18         // initialize chip and start measurement
#define OPC_WRITE_CONFIG 0x80 // write configuration
#define OPC_READ_CONFIG 0x40  // read configuration
#define OPC_READ_RESULTS 0x60 // read measurement results

//spi speed in Hz
#define GPX2_SPI_SPEED_HZ (4*1000*1000)

// configuration registers, 17 bytes, from datasheet
static uint8_t gpx2_config[17] = {
    0x31, 0x01, 0x1F, 0x40,
    0x0D, 0x03, 0xC0, 0x53,
    0xA1, 0x13, 0x00, 0x0A,
    0xCC, 0xCC, 0x31, 0x8E,
    0x04};

static uint8_t pins[4] = {0};
bool measure = true;
bool clk_reset = false;

static void restart()
{
    watchdog_reboot(0, 0, 0); // reboots the chip
}

/**
 *  User input config modification
 */

static void gpx2_set_input_pins(uint8_t pins[4])
{
    gpx2_config[0] &= ~0b00001111;
    uint8_t aux = 0;
    for (size_t i = 0; i < 4; i++)
    {
        if (pins[i] != 0)
        {
            aux |= (1 << i);
        }
    }
    gpx2_config[0] |= aux;
}
static void gpx2_set_refclk(uint8_t clk)
{
    clk &= 0x1;
    gpx2_config[0]&=~(1<<4);
    gpx2_config[0] |= (clk << 4);
}
static void gpx2_set_input_processing(uint8_t pins[4])
{
    gpx2_config[1] &= ~0b00001111;
    uint8_t aux = 0;
    for (size_t i = 0; i < 4; i++)
    {
        if (pins[i] != 0)
        {
            aux |= (1 << i);
        }
    }
    gpx2_config[1] |= aux;
}
typedef enum
{
    GPX2_COMBINE_NONE = 0,
    GPX2_COMBINE_PULSE_DISTANCE = 1,
    GPX2_COMBINE_PULSE_WIDTH = 2
} gpx2_channel_combine_t;
gpx2_channel_combine_t gpx2_channel_combine_mode_converter(char mode)
{
    if (mode == 'D' || mode == 'd')
    {
        return GPX2_COMBINE_PULSE_DISTANCE;
    }
    else if (mode == 'W' || mode == 'w')
    {
        return GPX2_COMBINE_PULSE_WIDTH;
    }
    return GPX2_COMBINE_NONE;
}
static void gpx2_set_channel_combine(gpx2_channel_combine_t mode)
{
    gpx2_config[1] &= ~0b00110000;
    gpx2_config[1] |= (mode << 4);
}

typedef enum
{
    GPX2_HIRES_OFF = 0,
    GPX2_HIRES_2X = 1,
    GPX2_HIRES_4X = 2
} gpx2_hires_mode_t;
static gpx2_hires_mode_t gpx2_hires_mode_converter(uint8_t mode)
{
    if (mode == 2)
    {
        return GPX2_HIRES_2X;
    }
    else if (mode == 4)
    {
        return GPX2_HIRES_4X;
    }
    return GPX2_HIRES_OFF;
}

static void gpx2_set_hires(gpx2_hires_mode_t mode)
{
    // mask for bits 6-7
    const uint8_t HIRES_MASK = 0xC0; // 1100 0000
    // clear bits 6-7
    gpx2_config[1] &= ~HIRES_MASK;
    // insert new mode(shifted into bits 6-7)
    gpx2_config[1] |= (mode << 6);
}
static void gpx2_set_common_fifo(uint8_t mode)
{
    mode &= 1;
    gpx2_config[2] &=~(1<<6);//clear bit before setting
    gpx2_config[2] |= (mode << 6);
}
static void gpx2_set_blockwise_fifo(uint8_t mode)
{
    mode &= 1;
    gpx2_config[2]&=~(1<<7);//clear bit before setting
    gpx2_config[2] |= (mode << 7);
}
// frequency config
static void gpx2_set_refclk_divisions(uint32_t divisions)
{
    // divisions must fit in 20bits
    divisions &= 0XFFFFF;
    // config[3]=lowest 8bits
    gpx2_config[3] = (divisions & 0xFF);
    // config[4]=middle 8bits
    gpx2_config[4] = ((divisions >> 8) & 0xFF);
    // config[5]=upper 4bits
    gpx2_config[5] &= 0xF0; // clear lower 4 bits
    gpx2_config[5] |= ((divisions >> 16) & 0x0F);
}
// helper:calculate frequency
uint32_t gpx2_compute_divisions_from_freq(uint32_t freq_hz)
{
    // period in ps =1e12/freq
    return (1000000000000ULL / freq_hz);
}
static void gpx2_set_refclk_by_xosc(uint8_t mode)
{
    mode &= 1;
    gpx2_config[7] &=~(1<<7);
    gpx2_config[7] |= (mode << 7);
}
static void gpx2_set_cmos_input(uint8_t mode)
{
    mode &= 1;
    gpx2_config[16]&=~(1<<2); //clear bit before setting
    gpx2_config[16] |= (mode << 2);
}
static void gpx2_input_config()
{
    int input = 0;
    uint32_t bigInput = 0;
    char ch;

    printf("\n---INPUT CONFIGURATION---\n");

    //stop pins
    printf("\nConfigure STOP pins(0=disable, 1=enable)\n");
    for(size_t i=0; i<4; i++){
        printf("Enable STOP%d? (0/1): ", i+1);
        scanf("%d", &input);
        pins[i]=(uint8_t)input;
    }
    gpx2_set_input_pins(pins);

    //REFCLK pin enable
    printf("\nEnable REFCLK pin? (0=off, 1=on): ");
    scanf("%d", &input);
    gpx2_set_refclk(input);

    //HIT_ENA pins
    printf("\nConfigure HIT_ENA pins (0=disable, 1=enable)\n");
    for (size_t i=0; i<4; i++){
        printf("Enable HIT_ENA%d? (0/1): ", i+1);
        scanf("%d", &input);
        pins[i]=(uint8_t)input;
    }
    gpx2_set_input_processing(pins);

    //channel combine 
    printf("\nChannel combine mode: \n");
    printf("N=normal\n");
    printf("D=Pulse distance\n");
    printf("W=pulse width\n");
    printf("Select mode: ");
    scanf(" %c", &ch);
    gpx2_set_channel_combine(gpx2_channel_combine_mode_converter(ch));

    //HIRES
    printf("\nHIRES mode: \n");
    printf("0=OFF\n");
    printf("2=2x\n");
    printf("4=4x\n");
    printf("Select HIRES: ");
    scanf("%d", &input);
    gpx2_set_hires(gpx2_hires_mode_converter(input));

    //FIFO modes
    printf("\nEnable COMMON FIFO? (0/1): ");
    scanf("%d", &input);
    gpx2_set_common_fifo(input);

    printf("Enable BLOCKWISE FIFO? (0/1): ");
    scanf("%d", &input);
    gpx2_set_blockwise_fifo(input);

    //REFCLK frequency
    printf("\nEnter REFCLK frequency in Hz (e.g., 5000000 for 5MHz): ");
    scanf("%d", &bigInput);
    gpx2_set_refclk_divisions(gpx2_compute_divisions_from_freq(bigInput));

    //XOSC
    printf("\nUse internal XOSC? (0=extrenal REFCLK, 1=internal crystal): ");
    scanf("%d", &input);
    gpx2_set_refclk_by_xosc(input);

    //CMOS input
    printf("\nEnable CMOS input mode? (0/1): ");
    scanf("%d", &input);
    gpx2_set_cmos_input(input);

    printf("\nInput configuration updated.\n");
    
}

/**
 * Config values used during runtime
 */
static void gpx2_pins_disable()
{
    gpx2_config[0] |= (1 << 6);
}
static void gpx2_pins_enable()
{
    gpx2_config[0] &= ~(1 << 6);
}
static void gpx2_refclk_reset_pulse()
{
    gpx2_config[0] |= (1 << 7);
}
static void gpx2_refclk_reset_unpulse()
{
    gpx2_config[0] &= ~(1 << 7);
}

/**
 * Communication with TDC
 */

// helper:control chip select (SSN)
static inline void gpx2_cs_low(void)
{
    gpio_put(PIN_SPI_CS, 0); // drive CS low->select GPX2
}
static inline void gpx2_cs_high(void)
{
    gpio_put(PIN_SPI_CS, 1); // drive CS high->deselect GPX2
}

// send one byte over spi(blocking)
static void gpx2_spi_send_byte(uint8_t value)
{
    spi_write_blocking(SPI_PORT, &value, 1);
}
// read one byte over spi (blocking)
static uint8_t gpx2_spi_read_byte(void)
{
    uint8_t rx = 0;
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1);
    return rx;
}

// write configuration registers to GPX2
static bool gpx2_write_config(const uint8_t *cfg)
{
    gpx2_cs_low();

    // opcode for write config+start address
    gpx2_spi_send_byte(OPC_WRITE_CONFIG + 0x00);

    // send all 17 config bytes
    for (int i = 0; i < 17; i++)
    {
        gpx2_spi_send_byte(cfg[i]);
    }
    gpx2_cs_high();
    return true;
}

// read back and verify config. registers
static bool gpx2_verify_config(const uint8_t *cfg)
{
    gpx2_cs_low();

    // opcode for read config. +start address (0x00)
    gpx2_spi_send_byte(OPC_READ_CONFIG + 0x00);
    // read 17 bytes and compare to local config.
    uint8_t rx;
    for (int i = 0; i < 17; i++)
    {
        rx = gpx2_spi_read_byte();
        // printf("received %x\n",rx);
        if (rx != cfg[i])
        {
            gpx2_cs_high();
            return false;
        }
    }
    gpx2_cs_high();
    return true;
}
static void gpx2_write_and_verify_config(const uint8_t *cfg)
{
    char input;
    gpx2_write_config(cfg);
    if (!gpx2_verify_config(cfg))
    {
        printf("Config write error, press Q to reset pico\n");
        while (1)
        {
            input = getchar();
            if (input == 'q' || input == 'Q')
            {
                restart();
            }
        }
    }
}

// send initialize and start measurement
static void gpx2_start_measurement(void)
{
    gpx2_cs_low();
    gpx2_spi_send_byte(OPC_INIT);
    gpx2_cs_high();
    busy_wait_us(100);
}

// read 24bit value from SPI, GPX2 send values as 3-byte big-endian
static uint32_t gpx2_read_24bit(void)
{
    uint32_t value = 0;

    value |= ((uint32_t)gpx2_spi_read_byte()) << 16;
    value |= ((uint32_t)gpx2_spi_read_byte()) << 8;
    value |= ((uint32_t)gpx2_spi_read_byte());

    return value;
}

// read measurement results for 4 channels
static void gpx2_read_results(uint32_t reference_index[4],
                              uint32_t stop_results[4])
{
    // wait until gpx2 puls interrupt pin low
    while (gpio_get(PIN_GPX_INT) != 0)
    {
        tight_loop_contents(); // small idle loop
    }
    gpx2_cs_low();
    gpx2_spi_send_byte(OPC_READ_RESULTS + 8);

    for (int ch = 0; ch < 4; ch++)
    {
        // 3 bytes reference index
        reference_index[ch] = gpx2_read_24bit();
        // 3 bytes stop results
        stop_results[ch] = gpx2_read_24bit();
    }
    gpx2_cs_high();
}
bool gpx2_validate_input(void)
{
    bool ok=true;
    
    uint8_t pin_ena=gpx2_config[0]& 0x0F; //PIN_ENA1..4
    uint8_t refclk_ena=(gpx2_config[0]>>4)&0x01;
    uint8_t hit_ena=gpx2_config[1]&0x0F; //HIT_ENA1..4
    uint8_t channel_combine=(gpx2_config[1]>>4)&0x03;
    uint8_t hires=(gpx2_config[1]>>6)&0x03;
    uint8_t common_fifo=(gpx2_config[2]>>6)&0x01;
    uint8_t blockwise_fifo=(gpx2_config[2]>>7)&0x01;

    uint32_t refclk_div=
        gpx2_config[3]|
        (gpx2_config[4]<<8)|
        ((gpx2_config[5]&0x0F)<<16);
    
    uint8_t cmos_input=(gpx2_config[16]>>2)&0x01;

    printf("\n CONFIG VALIDATION \n");

    //1. PIN_ENA vs HIT_ENA mismatch
    for (int i=0; i<4; i++){
        if ((pin_ena & (1<<i))&&!(hit_ena&(1<<i))){
            printf("ERROR: STOP%d enabled but HIT_ENA%d disabled\n", i + 1, i + 1);
            ok=false;
        }
    }
    //2. REFCLK enabled?
    if (!refclk_ena){
        printf("ERROR: REFCLK input not enabled\n");
        ok=false;
    }
    //3. CHANNEL_COMBINE valid?
    if (channel_combine>2){
        printf("ERROR: invalid CHANNEL_COMBINE value (%u)\n", channel_combine);
        ok=false;
    }
    //4. HIRES valid?
    if (hires != 4 || hires != 2 || hires != 0){
        printf("ERROR: Invalid HIRES value (%u)\n", hires);
        ok=false;
    }
    //5. FIFO consistency
    if (common_fifo && !blockwise_fifo){
        printf("ERROR: COMMON_FIFO enabled but BLOCKWISE_FIFO disabled\n");
        ok=false;
    }
    //6. REFCLK_DIVISIONS check
    if (refclk_div==0){
        printf ("ERROR: REFCLK_DIVISIONS=0\n");
        ok=false;
    }
    else if (refclk_div<10000){
        printf("WARNING: REFCLK_DIVISIONS (%u) too low, quantization artifacts possible\n", refclk_div);
    }
    else if (refclk_div>1000000){
        printf("WARNING: REFCLK_DIVISIONS (%u) unusually high\n", refclk_div);
    }
    //7. CMOS_INPUT validation
    if (cmos_input){
        if (pin_ena==0){
            printf("ERROR: CMOS input enabled but no STOP pins active\n");
            ok=false;
        }
        if (!refclk_ena){
            printf("ERROR: CMOS input enabled but REFCLK disabled\n");
            ok=false;
        }
    }
    //8. REFCLK_DIVISIONS vs actual REFCLK freq
    uint32_t refclk_freq=1000000000000ULL/refclk_div;
    if(refclk_freq<20000000){
        printf("WARNING: REFCLK frequency (%u Hz) is low, may reduce resolution\n", refclk_freq);  
    }
    //XOSC vs REFCLK pin consistency
    uint8_t xosc=(gpx2_config[7]>>7)&0x01;
    if (xosc && refclk_ena){
        printf("ERROR: XOSC enabled but REFCLK pin also enabled-disable REFCLK when using XOSC.\n");
        ok=false;
    }
    if (!xosc && !refclk_ena){
        printf("ERROR: External REFCLK selected but REFCLK pin disabled.\n");
        ok=false;
    }
    //9. missing PIN_ENA_STOP master bit
    uint8_t stop_master_enable=(gpx2_config[0]>>6)&0x01;
    if (stop_master_enable==1 && pin_ena !=0){
        printf("ERROR: STOP pins enabled but master STOP_ENA bit is off\n");
        ok=false;
    }
    //10. CHANNEL_COMBINE mode requires specifiC STOP/HIT config.
    if (channel_combine==1||channel_combine==2){
        if ((pin_ena&0x03)!=0x03||(hit_ena&0x03)!=0x03){
            printf("ERROR: Pulse distance/width mode requires STOP1 and STOP2 enabled HIT_ENA1/2\n");
            ok=false;
        }
    }
    //11. HIT_EN channels
    if ((hit_ena & 0xF0)!=0){
        printf("ERROR: HIT_ENA has bits set beyond STOP4\n");
        ok=false;
    }
    //12. LVDS redout frequency warning
    
    //13. HIRES+CHANNEL_COMBINE conflict
    if (channel_combine==GPX2_COMBINE_PULSE_DISTANCE&&hires>0){
        if (hires==GPX2_HIRES_4X){
            printf("WARNING: Pulse Width + HIRES 4x may exceed internal timing limits at high event rates\n");
        } else {
            printf("WARNING: Pulse Width + HIRES may reduce accuracy at high event rates\n");
        }
    }
    //14. HIRES 4x  with coarse REFCLK_DIVISIONS may reduce accuracy
    if (hires==2 && refclk_div>50000){
        printf("WARNING: HIRES 4x with REFCLK_DIVISIONS reduces accuracy\n");
    }
    //15. STOP pins enabled but HIT_ENA disabled for all
    if (pin_ena !=0 && hit_ena==0){
        printf("ERROR: STOP pins enabled but all HIT_ENA bits are zero\n");
        ok=false;
    }
    //16. REFCLK enabled but REFCLK_DIV not updated
    
    //17. CMOS input mode+high frequency STOP signals
    if (cmos_input && (hires>0||channel_combine>0)){
        printf("WARNING: CMOS input with HIRES or COMBINE modes may distort timing\n");
    }
    //18. FIFO modes+SPI readout speed mismatch
    if (blockwise_fifo && common_fifo && GPX2_SPI_SPEED_HZ<8000000){
        printf("WARNING: FIFO modes enabled but SPI speed (%u Hz) may be too slow\n",
        GPX2_SPI_SPEED_HZ);
    }
    //19. REFCLK disabled but HIRES enabled
    if (!refclk_ena && hires>0){
        printf("ERROR: HIRES requires REFCLK enabled\n");
        ok=false;
    }
    //20. CHANNEL_COMBINE=pulse distance but STOP3/4 enabled
    if (channel_combine==1 && (pin_ena & 0b1100)){
        printf("WARNING: Pulse distance mode ignores STOP3/4; disable them for clarity\n");
    }
    //21. CHANNEL_COMBINE=pulse width but HIRES enabled
    //if (channel_combine==2 && hires>0){
      //  printf("WARNING: Pulse Width + HIRES may exceed internal timing limits\n");
    //} -> merged with 13th
    //22. REFCLK_DIVISION too small for pulse width mode
    if (channel_combine==2 && refclk_div>60000){
        printf("WARNING: Pulse width mode with coarse REFCLK_DIVISIONS reduces base time resolution\n");
    }
    //23. STOP pins enabled but REFCLK disabled
    if (pin_ena !=0 && !refclk_ena){
        printf("ERROR: STOP pins active but REFCLK disabled\n");
        ok=false;
    }
    //24. REFCLK_DIVISIONS too large(low resolution)
    if (refclk_div>100000){
        printf("WARNING: REFCLK_DIVISIONS>100000 ps reduces timing resolution\n");
    }
    if (ok)
        printf("CONFIG VALID\n");
    else
        printf("CONFIG INVALID-fix error before applying\n");
    return ok;
}

// main
int main()
{
    stdio_init_all(); // enable usb serial output
    char userinput = getchar();
    // initialize SPI hardware
    spi_init(SPI_PORT, GPX2_SPI_SPEED_HZ); // SPI_PORT at 4MHz
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);

    // initialie chip select pin
    gpio_init(PIN_SPI_CS);
    gpio_set_function(PIN_SPI_CS, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpx2_cs_high();

    // initialize interrupt pin
    gpio_init(PIN_GPX_INT);
    gpio_set_function(PIN_GPX_INT, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_GPX_INT, GPIO_IN);

    // power-on reset command
    gpx2_cs_low();
    gpx2_spi_send_byte(OPC_POWER_RESET);
    gpx2_cs_high();
    busy_wait_us(100);

    //cli
    do
    {
        gpx2_input_config();
    } while (!gpx2_validate_input());
    
    



    // write config to GPX2
    gpx2_write_and_verify_config(gpx2_config);

    printf("Config written, starting measurement...\n");

    // start measurement
    gpx2_start_measurement();

    uint32_t reference_index[4] = {0};
    uint32_t stop_results[4] = {0};

    while (true)
    {
        if (clk_reset)
        {
            gpx2_refclk_reset_unpulse();
            gpx2_write_and_verify_config(gpx2_config);
            clk_reset = false;
            gpx2_start_measurement();
        }

        userinput = getchar_timeout_us(0);
        if (userinput == 'p' || userinput == 'P') // p pauses measurements
        {
            measure = false;
            gpx2_pins_disable();
            gpx2_write_and_verify_config(gpx2_config);
        }
        else if (userinput == 'r' || userinput == 'R') // r restarts measurements
        {
            measure = true;
            gpx2_pins_enable();
            gpx2_write_and_verify_config(gpx2_config);
            gpx2_start_measurement();
        }
        else if (userinput == 'c' || userinput == 'C')
        {
            gpx2_refclk_reset_pulse();
            gpx2_write_and_verify_config(gpx2_config);
            clk_reset = true;
        }
        else if (userinput == 'q' || userinput == 'Q') // q resets the pico
        {
            restart();
        }
        if (measure)
        {
            // read masurement results
            gpx2_read_results(reference_index, stop_results);

            // print results for all 4 channels
            for (int ch = 0; ch < 4; ch++)
            {
                if (pins[ch] != 0)
                {
                    printf("CH%d: REF=%lu   STOP=%lu\n",
                           ch + 1,
                           (unsigned long)reference_index[ch],
                           (unsigned long)stop_results[ch]);
                    // printf("%d\n",stop_results[ch]); //debug
                }
            }
        }

        
    }
    return 0;
}
