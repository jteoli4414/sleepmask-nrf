#define MAX_INT_STATUS1 0x00
#define MAX_INT_STATUS2 0x01

#define MAX_INT_EN1 0x02
#define MAX_AFULL_INT_MASK 0x80
#define MAX_AFULL_INT_EN 0x80
#define MAX_PPGRDY_INT_MASK 0x40
#define MAX_PPGRDY_INT_EN 0x40
#define MAX_ALCOVF_INT_MASK 0x20
#define MAX_ALCOVF_INT_EN 0x20

#define MAX_INT_EN2 0x03
#define MAX_DIETEMPRDY_INT_MASK 0x02
#define MAX_DIETEMPRDY_INT_EN 0x02

#define MAX_FIFO_WT_PTR 0x04
#define MAX_OVF_COUNTER 0x05
#define MAX_FIFO_RD_PTR 0x06
#define MAX_FIFO_DATA 0x07

#define MAX_FIFO_CONFIG 0x08

#define MAX_SAMPLEAVG_MASK 0xE0
#define MAX_SAMPLEAVG_1 0x00
#define MAX_SAMPLEAVG_2 0x20
#define MAX_SAMPLEAVG_4 0x40
#define MAX_SAMPLEAVG_8 0x60
#define MAX_SAMPLEAVG_16 0x80
#define MAX_SAMPLEAVG_32 0xA0


#define MAX_FIFO_ROLLOVER_MASK 0x10
#define MAX_FIFO_ROLLOVER_EN 0x10
#define MAX_FIFO_ALMOST_FULL_MASK 0x0F

#define MAX_MODE_CONFIG 0x09
#define MAX_RESET_MASK 0x40
#define MAX_RESET_EN 0x40
#define MAX_SHUTDOWN_MASK 0x80
#define MAX_SHUTDOWN_EN 0x80
#define MAX_WAKEUP_EN 0x00
#define MAX_MODE_MASK 0x07
#define MAX_MODE_REDONLY 0x02
#define MAX_MODE_REDIRONLY 0x03
#define MAX_MODE_MULTILED 0x07

#define MAX_SPO2_CONFIG 0x0D

#define MAX_ADC_RANGE_MASK 0x60
#define MAX_ADCRANGE_MASK 0x9F
#define MAX_ADCRANGE_2048 0x00
#define MAX_ADCRANGE_4096 0x20
#define MAX_ADCRANGE_8192 0x40
#define MAX_ADCRANGE_16384 0x60

#define MAX_PULSE_WIDTH_MASK 0x03
#define MAX_PULSEWIDTH_69 0x00
#define MAX_PULSEWIDTH_118 0x01
#define MAX_PULSEWIDTH_215 0x02
#define MAX_PULSEWIDTH_411 0x03

#define MAX_SAMPLE_RATE_MASK 0x1E
#define MAX_SAMPLERATE_50 0x00
#define MAX_SAMPLERATE_100 0x04
#define MAX_SAMPLERATE_200 0x08
#define MAX_SAMPLERATE_400 0x0C
#define MAX_SAMPLERATE_800 0x10
#define MAX_SAMPLERATE_1000 0x14
#define MAX_SAMPLERATE_1600 0x18
#define MAX_SAMPLERATE_3200 0x1C

#define MAX_LED1_PA 0x0C
#define MAX_LED2_PA 0x0D

#define MAX_LED_MULTI_CONTROL1 0x11
#define MAX_LED_MULTI_CONTROL2 0x12
#define MAX_SLOT1_MASK 0x07
#define MAX_SLOT2_MASK 0x70
#define MAX_SLOT3_MASK 0x07
#define MAX_SLOT4_MASK 0x70
#define MAX_SLOT_NONE 0x00
#define MAX_SLOT_RED_LED 0x01
#define MAX_SLOT_IR_LED 0x02

#define MAX_DIE_TEMP_INT 0x1F
#define MAX_DIE_TEMP_FRACTION 0x20
#define MAX_DIE_TEMP_CONFIG 0x21
#define MAX_REV_ID 0xFE
#define MAX_PART_ID 0xFF

#define MAX30102_ADDR 0x57

#define I2C_BUFFER_LENGTH 32
#define SAMPLE_BUFFER_LENGTH 32

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

typedef struct{
    const struct i2c_dt_spec * device;
    uint8_t num_active_led;
    uint32_t red[SAMPLE_BUFFER_LENGTH];
    uint32_t ir[SAMPLE_BUFFER_LENGTH];
    uint8_t head;
    uint8_t tail;
} max_ctx_t;

/* ctor and dtor */
max_ctx_t * max_create(const struct i2c_dt_spec * device);
void max_destroy(max_ctx_t * ctx);

/* Set Up Defualt Configs */
void max_setup(max_ctx_t * ctx, uint8_t pwr_lev_red, uint8_t pwr_lev_ir, uint8_t sample_avg, uint8_t led_mode, int sample_rate, int pulse_width, int adc_range);

uint32_t max_get_red(max_ctx_t * ctx);
uint32_t max_get_ir(max_ctx_t * ctx);
uint32_t max_get_fifo_red(max_ctx_t * ctx);
uint32_t max_get_fifo_ir(max_ctx_t * ctx);

/* Configuration */
void max_soft_reset(max_ctx_t * ctx);
void max_shut_down(max_ctx_t * ctx);
void max_wake_up(max_ctx_t * ctx);

void max_set_led_mode(max_ctx_t * ctx, uint8_t mode);
void max_set_adc_range(max_ctx_t * ctx, uint8_t adc_range);
void max_set_pulse_width(max_ctx_t * ctx, uint8_t pulse_width);
void max_set_sample_rate(max_ctx_t * ctx, uint8_t rate);

void max_set_pulse_amplitude_red(max_ctx_t * ctx, uint8_t value);
void max_set_pulse_amplitude_ir(max_ctx_t * ctx, uint8_t value);

void max_enable_slot(max_ctx_t * ctx, uint8_t slot_number, uint8_t device);
void max_disable_slots(max_ctx_t * ctx);

/* Interrupts */
uint8_t max_get_INT1(max_ctx_t * ctx); //Returns the main interrupt group
uint8_t max_get_INT2(max_ctx_t * ctx); //Returns the temp ready interrupt
void max_enable_int_AFULL(max_ctx_t * ctx); /* Enable or Disable individual interrupts */
void max_disable_int_AFULL(max_ctx_t * ctx);
void max_enable_int_DATARDY(max_ctx_t * ctx);
void max_disable_int_DATARDY(max_ctx_t * ctx);
void max_enable_int_ALCOVF(max_ctx_t * ctx);
void max_disable_int_ALCOVF(max_ctx_t * ctx);
void max_enable_int_PROXINT(max_ctx_t * ctx);
void max_disable_int_PROXINT(max_ctx_t * ctx);
void max_enable_int_DIETEMPRDY(max_ctx_t * ctx);
void max_disable_int_DIETEMPRDY(max_ctx_t * ctx);

/* FIFO Configuration */
void max_set_fifo_average(max_ctx_t * ctx, uint8_t samples);
void max_enable_fifo_rollover(max_ctx_t * ctx);
void max_disable_fifo_rollover(max_ctx_t * ctx);
void max_set_fifo_almost_full(max_ctx_t * ctx, uint8_t samples);

/* FIFO Reading */
uint16_t max_check(max_ctx_t * ctx); // Checks for new data and fills FIFO
uint8_t max_safe_check(max_ctx_t * ctx, int64_t wait);
uint8_t max_available(max_ctx_t * ctx); // Tells caller how many new samples are available (head - tail)
void max_next_sample(max_ctx_t * ctx); // Advances the tail of the sense array

uint8_t max_get_write_pointer(max_ctx_t * ctx);
uint8_t max_get_read_pointer(max_ctx_t * ctx);
void max_clear_fifo(max_ctx_t * ctx); //Sets the read/write pointers to zero

/* Die Temperature */
float max_read_temperature(max_ctx_t * ctx);
float max_read_temperatureF(max_ctx_t * ctx);

/* Detecting ID/Revision */ 
uint8_t max_get_revision_id(max_ctx_t * ctx);
uint8_t max_get_read_part_id(max_ctx_t * ctx);