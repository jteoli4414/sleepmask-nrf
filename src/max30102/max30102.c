#include "max30102.h"

/* I2C Protocols */
/* Write byte to MAX at addr */
void max_write_register_8(const struct i2c_dt_spec * device, uint8_t addr, uint8_t val)
{
    uint8_t buf[2];
    buf[0] = addr;
    buf[1] = val;
    int ret = i2c_write_dt(device, buf, 2);
    if(ret != 0){
        printk("Failed to write to I2C device address %x at reg. %x \n\r", device->addr, buf[0]);
    }
}
/* Read byte from MAX at addr */
uint8_t max_read_register_8(const struct i2c_dt_spec * device, uint8_t addr)
{
    uint8_t reg_addr = addr;
    uint8_t reading = 0;
    int ret = i2c_write_read_dt(device, &reg_addr, 1, &reading, 1);
    if(ret != 0){
        printk("Failed to write to I2C device address %x at reg. %x \n\r", device->addr, reg_addr);
    }
    return reading;
}
/* Write byte without clearing other bits */
void max_mask_write_register_8(const struct i2c_dt_spec * device, uint8_t reg_addr, uint8_t mask, uint8_t val)
{
    uint8_t orig = max_read_register_8(device, reg_addr);
    orig &= ~(mask); /* Bit Clear */
    orig |= val;
    max_write_register_8(device, reg_addr, orig);
}
/* ctor */
max_ctx_t * max_create(const struct i2c_dt_spec * device)
{
    max_ctx_t * inst = malloc(sizeof(max_ctx_t));
    if (inst == NULL)
    {
        return NULL;
    }
    inst->device = device;
    /* Initialize Array Blocks */
    memset(inst->ir, 0, sizeof(inst->ir));
    memset(inst->red, 0, sizeof(inst->red));
    return inst;
}
/* dtor */
void max_destroy(max_ctx_t * ctx)
{

}
/* Initialize MAX */
void max_setup(max_ctx_t * ctx, uint8_t pwr_lev_red, uint8_t pwr_lev_ir, uint8_t sample_avg, uint8_t led_mode, int sample_rate, int pulse_width, int adc_range)
{
    max_soft_reset(ctx); /* Reset All Configs to POR Values */

    /* FIFO */
    switch(sample_avg)
    {
        case 1: max_set_fifo_average(ctx, MAX_SAMPLEAVG_1); break;
        case 2: max_set_fifo_average(ctx, MAX_SAMPLEAVG_2); break;
        case 4: max_set_fifo_average(ctx, MAX_SAMPLEAVG_4); break;
        case 8: max_set_fifo_average(ctx, MAX_SAMPLEAVG_8); break;
        case 16: max_set_fifo_average(ctx, MAX_SAMPLEAVG_16); break;
        case 32: max_set_fifo_average(ctx, MAX_SAMPLEAVG_32); break;
        default: max_set_fifo_average(ctx, MAX_SAMPLEAVG_4); break;
    }

    max_enable_fifo_rollover(ctx);
    /* Mode */
    switch(led_mode)
    {
        case 1: max_set_led_mode(ctx, MAX_MODE_REDONLY); break;
        case 2: max_set_led_mode(ctx, MAX_MODE_REDIRONLY); break;
        case 3: max_set_led_mode(ctx, MAX_MODE_MULTILED); break;
        default: max_set_led_mode(ctx, MAX_MODE_REDIRONLY); break;
    }

    /* ADC */
    if(adc_range < 4096) max_set_adc_range(ctx, MAX_ADCRANGE_2048);
    else if(adc_range < 8192) max_set_adc_range(ctx, MAX_ADCRANGE_4096);
    else if(adc_range < 16384) max_set_adc_range(ctx, MAX_ADCRANGE_8192);
    else if(adc_range == 16384) max_set_adc_range(ctx, MAX_ADCRANGE_16384);
    else max_set_adc_range(ctx, MAX_ADCRANGE_2048);

    /* Sampling rate */
    if(sample_rate < 100) max_set_sample_rate(ctx, MAX_SAMPLERATE_50);
    else if(sample_rate < 200) max_set_sample_rate(ctx, MAX_SAMPLERATE_100);
    else if(sample_rate < 400) max_set_sample_rate(ctx, MAX_SAMPLERATE_200);
    else if(sample_rate < 800) max_set_sample_rate(ctx, MAX_SAMPLERATE_400);
    else if(sample_rate < 1000) max_set_sample_rate(ctx, MAX_SAMPLERATE_800);
    else if(sample_rate < 1600) max_set_sample_rate(ctx, MAX_SAMPLERATE_1000);
    else if(sample_rate < 3200) max_set_sample_rate(ctx, MAX_SAMPLERATE_1600);
    else if(sample_rate == 3200) max_set_sample_rate(ctx, MAX_SAMPLERATE_3200);
    else max_set_sample_rate(ctx, MAX_SAMPLERATE_50);

    /* Pulse width */
    if(pulse_width < 118) max_set_pulse_width(ctx, MAX_PULSEWIDTH_69);
    else if(pulse_width < 215) max_set_pulse_width(ctx, MAX_PULSEWIDTH_118);
    else if(pulse_width < 411) max_set_pulse_width(ctx, MAX_PULSEWIDTH_215);
    else if(pulse_width == 411) max_set_pulse_width(ctx, MAX_PULSEWIDTH_411);
    else max_set_pulse_width(ctx, MAX_PULSEWIDTH_69);

    /* Set amplitude */
    max_set_pulse_amplitude_red(ctx, pwr_lev_red);
    max_set_pulse_amplitude_ir(ctx, pwr_lev_ir);

    /* Set slots */
    max_enable_slot(ctx, 1, MAX_SLOT_RED_LED);
    ctx->num_active_led = 1;
    if(led_mode > 1) 
    {
        max_enable_slot(ctx, 2, MAX_SLOT_IR_LED);
        ctx->num_active_led = 2;
    }


    /* Clear FIFO */
    max_clear_fifo(ctx);
    ctx->head = 0;
    ctx->tail = 0;
}


uint32_t max_get_red(max_ctx_t * ctx)
{
    if(max_safe_check(ctx, 250000))
    {
        return(ctx->red[ctx->head]);
    }
    else
    {
        return 0;
    }
}
uint32_t max_get_ir(max_ctx_t * ctx)
{
    if(max_safe_check(ctx, 250000))
    {
        return(ctx->ir[ctx->head]);
    }
    else
    {
        return 0;
    }
}

uint32_t max_get_fifo_red(max_ctx_t * ctx)
{
    return ctx->red[ctx->tail];
}
uint32_t max_get_fifo_ir(max_ctx_t * ctx)
{
    return ctx->ir[ctx->tail];
}

/* Configuration */
void max_soft_reset(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_MODE_CONFIG, MAX_RESET_MASK, MAX_RESET_EN);


    uint32_t ms_then = k_uptime_get_32();
    while((k_uptime_get_32() - ms_then) < 100000)
    {
        uint8_t response = max_read_register_8(ctx->device, MAX_MODE_CONFIG);
        if ((response & MAX_RESET_MASK) == 0)
        {
            break;
        }
    }
}
void max_shut_down(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_MODE_CONFIG, MAX_SHUTDOWN_MASK, MAX_SHUTDOWN_EN);
}
void max_wake_up(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_MODE_CONFIG, MAX_SHUTDOWN_MASK, MAX_WAKEUP_EN);
}
void max_set_led_mode(max_ctx_t * ctx, uint8_t mode)
{
    max_mask_write_register_8(ctx->device, MAX_MODE_CONFIG, MAX_MODE_MASK, mode);
}

void max_set_adc_range(max_ctx_t * ctx, uint8_t adc_range)
{
    max_mask_write_register_8(ctx->device, MAX_SPO2_CONFIG, MAX_ADC_RANGE_MASK, adc_range);
}
void max_set_pulse_width(max_ctx_t * ctx, uint8_t pulse_width)
{
    max_mask_write_register_8(ctx->device, MAX_SPO2_CONFIG, MAX_PULSE_WIDTH_MASK, pulse_width);
}
void max_set_sample_rate(max_ctx_t * ctx, uint8_t rate)
{
    max_mask_write_register_8(ctx->device, MAX_SPO2_CONFIG, MAX_SAMPLE_RATE_MASK, rate);
}
void max_set_pulse_amplitude_red(max_ctx_t * ctx, uint8_t value)
{
    max_write_register_8(ctx->device, MAX_LED1_PA, value);
}
void max_set_pulse_amplitude_ir(max_ctx_t * ctx, uint8_t value)
{
    max_write_register_8(ctx->device, MAX_LED2_PA, value);
}

void max_enable_slot(max_ctx_t * ctx, uint8_t slot_number, uint8_t device)
{
    switch (slot_number) 
    {
        case (1):
        max_mask_write_register_8(ctx->device, MAX_LED_MULTI_CONTROL1, MAX_SLOT1_MASK, device);
        break;
        case (2):
        max_mask_write_register_8(ctx->device, MAX_LED_MULTI_CONTROL1, MAX_SLOT2_MASK, device << 4);
        break;
        case (3):
        max_mask_write_register_8(ctx->device, MAX_LED_MULTI_CONTROL2, MAX_SLOT3_MASK, device << 4);
        break;
        case (4):
        max_mask_write_register_8(ctx->device, MAX_LED_MULTI_CONTROL2, MAX_SLOT4_MASK, device << 4);
        break;
        default:
        break;
    }
}
void max_disable_slots(max_ctx_t * ctx)
{
    max_write_register_8(ctx->device, MAX_LED_MULTI_CONTROL1, 0);
    max_write_register_8(ctx->device, MAX_LED_MULTI_CONTROL2, 0);
}

/* Interrupts */
uint8_t max_get_INT1(max_ctx_t * ctx) // Returns the main interrupt group
{
    return(max_read_register_8(ctx->device, MAX_INT_EN1));
}
uint8_t max_get_INT2(max_ctx_t * ctx) // Returns the temp ready interrupt
{
    return(max_read_register_8(ctx->device, MAX_INT_EN2));
}
void max_enable_int_AFULL(max_ctx_t * ctx) /* Enable or Disable individual interrupts */
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN1, MAX_AFULL_INT_MASK, MAX_AFULL_INT_EN);
}
void max_disable_int_AFULL(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN1, MAX_AFULL_INT_MASK, 0);
}
void max_enable_int_DATARDY(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN1, MAX_PPGRDY_INT_MASK, MAX_PPGRDY_INT_EN);
}
void max_disable_int_DATARDY(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN1, MAX_PPGRDY_INT_MASK, 0);
}
void max_enable_int_ALCOVF(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN1, MAX_ALCOVF_INT_MASK, MAX_ALCOVF_INT_EN);
}
void max_disable_int_ALCOVF(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN1, MAX_ALCOVF_INT_MASK, 0);
}
void max_enable_int_DIETEMPRDY(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN2, MAX_DIETEMPRDY_INT_MASK, MAX_DIETEMPRDY_INT_EN);
}
void max_disable_int_DIETEMPRDY(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_INT_EN2, MAX_DIETEMPRDY_INT_MASK, 0);
}

/* FIFO Configuration */
void max_set_fifo_average(max_ctx_t * ctx, uint8_t sample_reg)
{
    max_mask_write_register_8(ctx->device, MAX_FIFO_CONFIG, MAX_SAMPLEAVG_MASK, sample_reg);
}
void max_enable_fifo_rollover(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_FIFO_CONFIG, MAX_FIFO_ROLLOVER_MASK, MAX_FIFO_ROLLOVER_EN);
}
void max_disable_fifo_rollover(max_ctx_t * ctx)
{
    max_mask_write_register_8(ctx->device, MAX_FIFO_CONFIG, MAX_FIFO_ROLLOVER_MASK, 0);
}
void max_set_fifo_almost_full(max_ctx_t * ctx, uint8_t sample_reg)
{
    max_mask_write_register_8(ctx->device, MAX_FIFO_CONFIG, MAX_FIFO_ALMOST_FULL_MASK, sample_reg);
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t max_check(max_ctx_t * ctx) //Checks for new data and fills FIFO
{
    //Read register FIDO_DATA in (3-byte * number of active LED) chunks
    //Until FIFO_RD_PTR = FIFO_WR_PTR
    uint8_t read_pointer = max_get_read_pointer(ctx);
    uint8_t write_pointer = max_get_write_pointer(ctx);

    int number_of_samples = 0;
    // New data?
    if(read_pointer != write_pointer)
    {
        number_of_samples = write_pointer - read_pointer;
        if(number_of_samples < 0) // Wrapping condition
        {
            number_of_samples += 32;
        }
        int bytes_left = number_of_samples * ctx->num_active_led * 3; // (3 Bytes per sample) * (number of active LEDs) * (samples to be read)
        while(bytes_left > 0)
        {
            // Trim to_get to be the amount of bytes of the multiple of samples we need
            int to_get = bytes_left;
            if(to_get > I2C_BUFFER_LENGTH) // Allows us to get more data than chips i2c buffer allows
            {
                to_get = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (ctx->num_active_led * 3)); // Make sure its multiple of how many LEDs we have on
            }
            bytes_left -= to_get; // decrement how much we need to get
            uint8_t * temp_buf = malloc(to_get * sizeof(uint8_t));
            // We know how much we need to get now get
            while(to_get > 0)
            {
                ctx->head++;
                ctx->head %= SAMPLE_BUFFER_LENGTH; // Wrap Condition
                /* RED LED */
                uint32_t temp;
                uint16_t reg_addr = (uint16_t)MAX_FIFO_DATA;
                i2c_write_read_dt(ctx->device, &reg_addr, (size_t)1, temp_buf, (size_t)to_get);
                if(ctx->num_active_led > 1)
                {
                    for(int i = 0; i < to_get; i+=6)
                    {
                        temp = 0;
                        temp += (uint32_t)temp_buf[i];
                        temp <<= 8;
                        temp += (uint32_t)temp_buf[i+1];
                        temp <<= 8;
                        temp += (uint32_t)temp_buf[i+2];
                        temp &= 0x3ffff;
                        ctx->red[ctx->head] = temp;
                        temp = 0;
                        temp += (uint32_t)temp_buf[i+3];
                        temp <<= 8;
                        temp += (uint32_t)temp_buf[i+4];
                        temp <<= 8;
                        temp += (uint32_t)temp_buf[i+5];
                        temp &= 0x3ffff;
                        ctx->ir[ctx->head] = temp;
                    }
                }
                else
                {
                    for(int i = 0; i < to_get; i+=3)
                    {
                        temp = 0;
                        temp += (uint32_t)temp_buf[i];
                        temp <<= 8;
                        temp += (uint32_t)temp_buf[i+1];
                        temp <<= 8;
                        temp += (uint32_t)temp_buf[i+2];
                        temp &= 0x3ffff;
                        ctx->red[ctx->head] = temp;
                    }
                }
                to_get -= ctx->num_active_led * 3;
            }
            free(temp_buf);
        }
    }
    return ((uint16_t)number_of_samples);
}

uint8_t max_safe_check(max_ctx_t * ctx, int64_t ms_wait)
{
    uint32_t ms_then = k_uptime_get_32();
    while((k_uptime_get_32() - ms_then) < ms_wait)
    {
        if(max_check(ctx) > 0)
        {
            return 1;
        }
    }
    return 0;
}

uint8_t max_available(max_ctx_t * ctx) //Tells caller how many new samples are available (head - tail)
{
    int8_t number_of_samples = ctx->head - ctx->tail;
    if(number_of_samples < 0)
    {
        number_of_samples += SAMPLE_BUFFER_LENGTH;
    }
    return number_of_samples;
}

void max_next_sample(max_ctx_t * ctx) //Advances the tail of the sense array
{
    if(max_available(ctx))
    {
        ctx->tail++;
        ctx->tail %= SAMPLE_BUFFER_LENGTH;
    }
}

uint8_t max_get_write_pointer(max_ctx_t * ctx)
{
    return(max_read_register_8(ctx->device, MAX_FIFO_WT_PTR));
}

uint8_t max_get_read_pointer(max_ctx_t * ctx)
{
    return(max_read_register_8(ctx->device, MAX_FIFO_RD_PTR));
}

void max_clear_fifo(max_ctx_t * ctx)
{
    max_write_register_8(ctx->device, MAX_FIFO_WT_PTR, 0);
    max_write_register_8(ctx->device, MAX_OVF_COUNTER, 0);
    max_write_register_8(ctx->device, MAX_FIFO_RD_PTR, 0);
} //Sets the read/write pointers to zero

/* Die Temperature */
float max_read_temperature(max_ctx_t * ctx)
{
    max_write_register_8(ctx->device, MAX_DIE_TEMP_CONFIG, 0x01);
    uint32_t ms_then = k_uptime_get_32();
    while((k_uptime_get_32() - ms_then) < 100000)
    {
        uint8_t response = max_read_register_8(ctx->device, MAX_DIE_TEMP_CONFIG);
        if(response & 0x01)
        {
            break;
        }
    }

    int8_t temp_int = max_read_register_8(ctx->device, MAX_DIE_TEMP_INT);
    uint8_t temp_frac = max_read_register_8(ctx->device, MAX_DIE_TEMP_FRACTION);

    return ((float)temp_int + ((float)temp_frac * 0.0625));
}
float max_read_temperatureF(max_ctx_t * ctx)
{
    float temp = max_read_temperature(ctx);
    if (temp != -999.0) temp = temp * 1.8 + 32.0;
    return (temp);
}

/* Detecting ID/Revision */ 
uint8_t max_get_revision_id(max_ctx_t * ctx)
{
    return(max_read_register_8(ctx->device, MAX_REV_ID));
}

uint8_t max_get_read_part_id(max_ctx_t * ctx)
{
    return(max_read_register_8(ctx->device, MAX_PART_ID));
}