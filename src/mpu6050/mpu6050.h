#ifndef _MPU6050_H
#define _MPU6050_H
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

// note: DMP code memory blocks defined at end of header file

#define I2C_TIMEOUT 1000
#define CALIBRATION_SAMPLES 128

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

/* State Variable */
typedef struct 
{
    /* Calibration Offsets */
    int16_t cali_ax;
    int16_t cali_ay;
    int16_t cali_az;
    int16_t cali_gx;
    int16_t cali_gy;
    int16_t cali_gz;
    const struct i2c_dt_spec * device;
    
}mpu_ctx_t;

/* ctor and dtor */


mpu_ctx_t * MPU6050_create(const struct i2c_dt_spec * device);
void MPU6050_destroy(mpu_ctx_t * ctx);

void MPU6050_initialize(mpu_ctx_t * ctx);
uint8_t MPU6050_testConnection(mpu_ctx_t * ctx);

// AUX_VDDIO register
uint8_t MPU6050_getAuxVDDIOLevel(mpu_ctx_t * ctx);
void MPU6050_setAuxVDDIOLevel(mpu_ctx_t * ctx, uint8_t level);

// SMPLRT_DIV register
uint8_t MPU6050_getRate(mpu_ctx_t * ctx);
void MPU6050_setRate(mpu_ctx_t * ctx, uint8_t rate);

// CONFIG register
uint8_t MPU6050_getExternalFrameSync(mpu_ctx_t * ctx);
void MPU6050_setExternalFrameSync(mpu_ctx_t * ctx, uint8_t sync);
uint8_t MPU6050_getDLPFMode(mpu_ctx_t * ctx);
void MPU6050_setDLPFMode(mpu_ctx_t * ctx, uint8_t bandwidth);

// GYRO_CONFIG register
uint8_t MPU6050_getFullScaleGyroRange(mpu_ctx_t * ctx);
void MPU6050_setFullScaleGyroRange(mpu_ctx_t * ctx, uint8_t range);

// ACCEL_CONFIG register
uint8_t MPU6050_getAccelXSelfTest(mpu_ctx_t * ctx);
void MPU6050_setAccelXSelfTest(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getAccelYSelfTest(mpu_ctx_t * ctx);
void MPU6050_setAccelYSelfTest(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getAccelZSelfTest(mpu_ctx_t * ctx);
void MPU6050_setAccelZSelfTest(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getFullScaleAccelRange(mpu_ctx_t * ctx);
void MPU6050_setFullScaleAccelRange(mpu_ctx_t * ctx, uint8_t range);
uint8_t MPU6050_getDHPFMode(mpu_ctx_t * ctx);
void MPU6050_setDHPFMode(mpu_ctx_t * ctx, uint8_t mode);

// FF_THR register
uint8_t MPU6050_getFreefallDetectionThreshold(mpu_ctx_t * ctx);
void MPU6050_setFreefallDetectionThreshold(mpu_ctx_t * ctx, uint8_t threshold);

// FF_DUR register
uint8_t MPU6050_getFreefallDetectionDuration(mpu_ctx_t * ctx);
void MPU6050_setFreefallDetectionDuration(mpu_ctx_t * ctx, uint8_t duration);

// MOT_THR register
uint8_t MPU6050_getMotionDetectionThreshold(mpu_ctx_t * ctx);
void MPU6050_setMotionDetectionThreshold(mpu_ctx_t * ctx, uint8_t threshold);

// MOT_DUR register
uint8_t MPU6050_getMotionDetectionDuration(mpu_ctx_t * ctx);
void MPU6050_setMotionDetectionDuration(mpu_ctx_t * ctx, uint8_t duration);

// ZRMOT_THR register
uint8_t MPU6050_getZeroMotionDetectionThreshold(mpu_ctx_t * ctx);
void MPU6050_setZeroMotionDetectionThreshold(mpu_ctx_t * ctx, uint8_t threshold);

// ZRMOT_DUR register
uint8_t MPU6050_getZeroMotionDetectionDuration(mpu_ctx_t * ctx);
void MPU6050_setZeroMotionDetectionDuration(mpu_ctx_t * ctx, uint8_t duration);

// FIFO_EN register
uint8_t MPU6050_getTempFIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setTempFIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getXGyroFIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setXGyroFIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getYGyroFIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setYGyroFIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getZGyroFIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setZGyroFIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getAccelFIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setAccelFIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlave2FIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setSlave2FIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlave1FIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setSlave1FIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlave0FIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setSlave0FIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);

// I2C_MST_CTRL register
uint8_t MPU6050_getMultiMasterEnabled(mpu_ctx_t * ctx);
void MPU6050_setMultiMasterEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getWaitForExternalSensorEnabled(mpu_ctx_t * ctx);
void MPU6050_setWaitForExternalSensorEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlave3FIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setSlave3FIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlaveReadWriteTransitionEnabled(mpu_ctx_t * ctx);
void MPU6050_setSlaveReadWriteTransitionEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getMasterClockSpeed(mpu_ctx_t * ctx);
void MPU6050_setMasterClockSpeed(mpu_ctx_t * ctx, uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t MPU6050_getSlaveAddress(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveAddress(mpu_ctx_t * ctx, uint8_t num, uint8_t address);
uint8_t MPU6050_getSlaveRegister(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveRegister(mpu_ctx_t * ctx, uint8_t num, uint8_t reg);
uint8_t MPU6050_getSlaveEnabled(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveEnabled(mpu_ctx_t * ctx, uint8_t num, uint8_t enabled);
uint8_t MPU6050_getSlaveWordByteSwap(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveWordByteSwap(mpu_ctx_t * ctx, uint8_t num, uint8_t enabled);
uint8_t MPU6050_getSlaveWriteMode(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveWriteMode(mpu_ctx_t * ctx, uint8_t num, uint8_t mode);
uint8_t MPU6050_getSlaveWordGroupOffset(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveWordGroupOffset(mpu_ctx_t * ctx, uint8_t num, uint8_t enabled);
uint8_t MPU6050_getSlaveDataLength(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveDataLength(mpu_ctx_t * ctx, uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
uint8_t MPU6050_getSlave4Address(mpu_ctx_t * ctx);
void MPU6050_setSlave4Address(mpu_ctx_t * ctx, uint8_t address);
uint8_t MPU6050_getSlave4Register(mpu_ctx_t * ctx);
void MPU6050_setSlave4Register(mpu_ctx_t * ctx, uint8_t reg);
void MPU6050_setSlave4OutputByte(mpu_ctx_t * ctx, uint8_t data);
uint8_t MPU6050_getSlave4Enabled(mpu_ctx_t * ctx);
void MPU6050_setSlave4Enabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlave4InterruptEnabled(mpu_ctx_t * ctx);
void MPU6050_setSlave4InterruptEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlave4WriteMode(mpu_ctx_t * ctx);
void MPU6050_setSlave4WriteMode(mpu_ctx_t * ctx, uint8_t mode);
uint8_t MPU6050_getSlave4MasterDelay(mpu_ctx_t * ctx);
void MPU6050_setSlave4MasterDelay(mpu_ctx_t * ctx, uint8_t delay);
uint8_t MPU6050_getSlate4InputByte(mpu_ctx_t * ctx);

// I2C_MST_STATUS register
uint8_t MPU6050_getPassthroughStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getSlave4IsDone(mpu_ctx_t * ctx);
uint8_t MPU6050_getLostArbitration(mpu_ctx_t * ctx);
uint8_t MPU6050_getSlave4Nack(mpu_ctx_t * ctx);
uint8_t MPU6050_getSlave3Nack(mpu_ctx_t * ctx);
uint8_t MPU6050_getSlave2Nack(mpu_ctx_t * ctx);
uint8_t MPU6050_getSlave1Nack(mpu_ctx_t * ctx);
uint8_t MPU6050_getSlave0Nack(mpu_ctx_t * ctx);

// INT_PIN_CFG register
uint8_t MPU6050_getInterruptMode(mpu_ctx_t * ctx);
void MPU6050_setInterruptMode(mpu_ctx_t * ctx, uint8_t mode);
uint8_t MPU6050_getInterruptDrive(mpu_ctx_t * ctx);
void MPU6050_setInterruptDrive(mpu_ctx_t * ctx, uint8_t drive);
uint8_t MPU6050_getInterruptLatch(mpu_ctx_t * ctx);
void MPU6050_setInterruptLatch(mpu_ctx_t * ctx, uint8_t latch);
uint8_t MPU6050_getInterruptLatchClear(mpu_ctx_t * ctx);
void MPU6050_setInterruptLatchClear(mpu_ctx_t * ctx, uint8_t clear);
uint8_t MPU6050_getFSyncInterruptLevel(mpu_ctx_t * ctx);
void MPU6050_setFSyncInterruptLevel(mpu_ctx_t * ctx, uint8_t level);
uint8_t MPU6050_getFSyncInterruptEnabled(mpu_ctx_t * ctx);
void MPU6050_setFSyncInterruptEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getI2CBypassEnabled(mpu_ctx_t * ctx);
void MPU6050_setI2CBypassEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getClockOutputEnabled(mpu_ctx_t * ctx);
void MPU6050_setClockOutputEnabled(mpu_ctx_t * ctx, uint8_t enabled);

// INT_ENABLE register
uint8_t MPU6050_getIntEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getIntFreefallEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntFreefallEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getIntMotionEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntMotionEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getIntZeroMotionEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntZeroMotionEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getIntFIFOBufferOverflowEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntFIFOBufferOverflowEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getIntI2CMasterEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntI2CMasterEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getIntDataReadyEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntDataReadyEnabled(mpu_ctx_t * ctx, uint8_t enabled);

// INT_STATUS register
uint8_t MPU6050_getIntStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getIntFreefallStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getIntMotionStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getIntZeroMotionStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getIntFIFOBufferOverflowStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getIntI2CMasterStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getIntDataReadyStatus(mpu_ctx_t * ctx);

// ACCEL_*OUT_* registers
void MPU6050_getMotion9(mpu_ctx_t * ctx, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
void MPU6050_getMotion6(mpu_ctx_t * ctx, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6050_getMotion6_cali(mpu_ctx_t * ctx, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6050_getAcceleration(mpu_ctx_t * ctx, int16_t* x, int16_t* y, int16_t* z);
int16_t MPU6050_getAccelerationX(mpu_ctx_t * ctx);
int16_t MPU6050_getAccelerationY(mpu_ctx_t * ctx);
int16_t MPU6050_getAccelerationZ(mpu_ctx_t * ctx);

// TEMP_OUT_* registers
int16_t MPU6050_getTemperature(mpu_ctx_t * ctx);

// GYRO_*OUT_* registers
void MPU6050_getRotation(mpu_ctx_t * ctx, int16_t* x, int16_t* y, int16_t* z);
int16_t MPU6050_getRotationX(mpu_ctx_t * ctx);
int16_t MPU6050_getRotationY(mpu_ctx_t * ctx);
int16_t MPU6050_getRotationZ(mpu_ctx_t * ctx);

// EXT_SENS_DATA_* registers
uint8_t MPU6050_getExternalSensorByte(mpu_ctx_t * ctx, int position);
uint16_t MPU6050_getExternalSensorWord(mpu_ctx_t * ctx, int position);
uint32_t MPU6050_getExternalSensorDWord(mpu_ctx_t * ctx, int position);

// MOT_DETECT_STATUS register
uint8_t MPU6050_getXNegMotionDetected(mpu_ctx_t * ctx);
uint8_t MPU6050_getXPosMotionDetected(mpu_ctx_t * ctx);
uint8_t MPU6050_getYNegMotionDetected(mpu_ctx_t * ctx);
uint8_t MPU6050_getYPosMotionDetected(mpu_ctx_t * ctx);
uint8_t MPU6050_getZNegMotionDetected(mpu_ctx_t * ctx);
uint8_t MPU6050_getZPosMotionDetected(mpu_ctx_t * ctx);
uint8_t MPU6050_getZeroMotionDetected(mpu_ctx_t * ctx);

// I2C_SLV*_DO register
void MPU6050_setSlaveOutputByte(mpu_ctx_t * ctx, uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
uint8_t MPU6050_getExternalShadowDelayEnabled(mpu_ctx_t * ctx);
void MPU6050_setExternalShadowDelayEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getSlaveDelayEnabled(mpu_ctx_t * ctx, uint8_t num);
void MPU6050_setSlaveDelayEnabled(mpu_ctx_t * ctx, uint8_t num, uint8_t enabled);

// SIGNAL_PATH_RESET register
void MPU6050_resetGyroscopePath(mpu_ctx_t * ctx);
void MPU6050_resetAccelerometerPath(mpu_ctx_t * ctx);
void MPU6050_resetTemperaturePath(mpu_ctx_t * ctx);

// MOT_DETECT_CTRL register
uint8_t MPU6050_getAccelerometerPowerOnDelay(mpu_ctx_t * ctx);
void MPU6050_setAccelerometerPowerOnDelay(mpu_ctx_t * ctx, uint8_t delay);
uint8_t MPU6050_getFreefallDetectionCounterDecrement(mpu_ctx_t * ctx);
void MPU6050_setFreefallDetectionCounterDecrement(mpu_ctx_t * ctx, uint8_t decrement);
uint8_t MPU6050_getMotionDetectionCounterDecrement(mpu_ctx_t * ctx);
void MPU6050_setMotionDetectionCounterDecrement(mpu_ctx_t * ctx, uint8_t decrement);

// USER_CTRL register
uint8_t MPU6050_getFIFOEnabled(mpu_ctx_t * ctx);
void MPU6050_setFIFOEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getI2CMasterModeEnabled(mpu_ctx_t * ctx);
void MPU6050_setI2CMasterModeEnabled(mpu_ctx_t * ctx, uint8_t enabled);
void MPU6050_switchSPIEnabled(mpu_ctx_t * ctx, uint8_t enabled);
void MPU6050_resetFIFO(mpu_ctx_t * ctx);
void MPU6050_resetI2CMaster(mpu_ctx_t * ctx);
void MPU6050_resetSensors(mpu_ctx_t * ctx);

// PWR_MGMT_1 register
void MPU6050_reset(mpu_ctx_t * ctx);
uint8_t MPU6050_getSleepEnabled(mpu_ctx_t * ctx);
void MPU6050_setSleepEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getWakeCycleEnabled(mpu_ctx_t * ctx);
void MPU6050_setWakeCycleEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getTempSensorEnabled(mpu_ctx_t * ctx);
void MPU6050_setTempSensorEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getClockSource(mpu_ctx_t * ctx);
void MPU6050_setClockSource(mpu_ctx_t * ctx, uint8_t source);

// PWR_MGMT_2 register
uint8_t MPU6050_getWakeFrequency(mpu_ctx_t * ctx);
void MPU6050_setWakeFrequency(mpu_ctx_t * ctx, uint8_t frequency);
uint8_t MPU6050_getStandbyXAccelEnabled(mpu_ctx_t * ctx);
void MPU6050_setStandbyXAccelEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getStandbyYAccelEnabled(mpu_ctx_t * ctx);
void MPU6050_setStandbyYAccelEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getStandbyZAccelEnabled(mpu_ctx_t * ctx);
void MPU6050_setStandbyZAccelEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getStandbyXGyroEnabled(mpu_ctx_t * ctx);
void MPU6050_setStandbyXGyroEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getStandbyYGyroEnabled(mpu_ctx_t * ctx);
void MPU6050_setStandbyYGyroEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getStandbyZGyroEnabled(mpu_ctx_t * ctx);
void MPU6050_setStandbyZGyroEnabled(mpu_ctx_t * ctx, uint8_t enabled);

// FIFO_COUNT_* registers
uint16_t MPU6050_getFIFOCount(mpu_ctx_t * ctx);

// FIFO_R_W register
uint8_t MPU6050_getFIFOByte(mpu_ctx_t * ctx);
void MPU6050_setFIFOByte(mpu_ctx_t * ctx, uint8_t data);
void MPU6050_getFIFOBytes(mpu_ctx_t * ctx, uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t MPU6050_getDeviceID(mpu_ctx_t * ctx);
void MPU6050_setDeviceID(mpu_ctx_t * ctx, uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

#ifdef ENABLE_DMP

// XG_OFFS_TC register
uint8_t MPU6050_getOTPBankValid(mpu_ctx_t * ctx);
void MPU6050_setOTPBankValid(mpu_ctx_t * ctx, uint8_t enabled);
int8_t MPU6050_getXGyroOffsetTC(mpu_ctx_t * ctx);
void MPU6050_setXGyroOffsetTC(mpu_ctx_t * ctx, uint8_t offset);

// YG_OFFS_TC register
int8_t MPU6050_getYGyroOffsetTC(mpu_ctx_t * ctx);
void MPU6050_setYGyroOffsetTC(mpu_ctx_t * ctx, uint8_t offset);

// ZG_OFFS_TC register
int8_t MPU6050_getZGyroOffsetTC(mpu_ctx_t * ctx);
void MPU6050_setZGyroOffsetTC(mpu_ctx_t * ctx, uint8_t offset);

// X_FINE_GAIN register
int8_t MPU6050_getXFineGain(mpu_ctx_t * ctx);
void MPU6050_setXFineGain(mpu_ctx_t * ctx, uint8_t gain);

// Y_FINE_GAIN register
int8_t MPU6050_getYFineGain(mpu_ctx_t * ctx);
void MPU6050_setYFineGain(mpu_ctx_t * ctx, uint8_t gain);

// Z_FINE_GAIN register
int8_t MPU6050_getZFineGain(mpu_ctx_t * ctx);
void MPU6050_setZFineGain(mpu_ctx_t * ctx, uint8_t gain);

// XA_OFFS_* registers
int16_t MPU6050_getXAccelOffset(mpu_ctx_t * ctx);
void MPU6050_setXAccelOffset(mpu_ctx_t * ctx, int16_t offset);

// YA_OFFS_* register
int16_t MPU6050_getYAccelOffset(mpu_ctx_t * ctx);
void MPU6050_setYAccelOffset(mpu_ctx_t * ctx, int16_t offset);

// ZA_OFFS_* register
int16_t MPU6050_getZAccelOffset(mpu_ctx_t * ctx);
void MPU6050_setZAccelOffset(mpu_ctx_t * ctx, int16_t offset);

// XG_OFFS_USR* registers
int16_t MPU6050_getXGyroOffset(mpu_ctx_t * ctx);
void MPU6050_setXGyroOffset(mpu_ctx_t * ctx, int16_t offset);

// YG_OFFS_USR* register
int16_t MPU6050_getYGyroOffset(mpu_ctx_t * ctx);
void MPU6050_setYGyroOffset(mpu_ctx_t * ctx, int16_t offset);

// ZG_OFFS_USR* register
int16_t MPU6050_getZGyroOffset(mpu_ctx_t * ctx);
void MPU6050_setZGyroOffset(mpu_ctx_t * ctx, int16_t offset);

// INT_ENABLE register (DMP functions)
uint8_t MPU6050_getIntPLLReadyEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntPLLReadyEnabled(mpu_ctx_t * ctx, uint8_t enabled);
uint8_t MPU6050_getIntDMPEnabled(mpu_ctx_t * ctx);
void MPU6050_setIntDMPEnabled(mpu_ctx_t * ctx, uint8_t enabled);

// DMP_INT_STATUS
uint8_t MPU6050_getDMPInt5Status(mpu_ctx_t * ctx);
uint8_t MPU6050_getDMPInt4Status(mpu_ctx_t * ctx);
uint8_t MPU6050_getDMPInt3Status(mpu_ctx_t * ctx);
uint8_t MPU6050_getDMPInt2Status(mpu_ctx_t * ctx);
uint8_t MPU6050_getDMPInt1Status(mpu_ctx_t * ctx);
uint8_t MPU6050_getDMPInt0Status(mpu_ctx_t * ctx);

// INT_STATUS register (DMP functions)
uint8_t MPU6050_getIntPLLReadyStatus(mpu_ctx_t * ctx);
uint8_t MPU6050_getIntDMPStatus(mpu_ctx_t * ctx);

// USER_CTRL register (DMP functions)
uint8_t MPU6050_getDMPEnabled(mpu_ctx_t * ctx);
void MPU6050_setDMPEnabled(mpu_ctx_t * ctx, uint8_t enabled);
void MPU6050_resetDMP(mpu_ctx_t * ctx);

// BANK_SEL register
void MPU6050_setMemoryBank(mpu_ctx_t * ctx, uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank);

// MEM_START_ADDR register
void MPU6050_setMemoryStartAddress(mpu_ctx_t * ctx, uint8_t address);

// MEM_R_W register
uint8_t MPU6050_readMemoryByte(mpu_ctx_t * ctx);
void MPU6050_writeMemoryByte(mpu_ctx_t * ctx, uint8_t data);
void MPU6050_readMemoryBlock(mpu_ctx_t * ctx, uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
uint8_t MPU6050_writeMemoryBlock(mpu_ctx_t * ctx, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem);
uint8_t MPU6050_writeProgMemoryBlock(mpu_ctx_t * ctx, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify);

uint8_t MPU6050_writeDMPConfigurationSet(mpu_ctx_t * ctx, const uint8_t *data, uint16_t dataSize, uint8_t useProgMem);
uint8_t MPU6050_writeProgDMPConfigurationSet(mpu_ctx_t * ctx, const uint8_t *data, uint16_t dataSize);

// DMP_CFG_1 register
uint8_t MPU6050_getDMPConfig1(mpu_ctx_t * ctx);
void MPU6050_setDMPConfig1(mpu_ctx_t * ctx, uint8_t config);

// DMP_CFG_2 register
uint8_t MPU6050_getDMPConfig2(mpu_ctx_t * ctx);
void MPU6050_setDMPConfig2(mpu_ctx_t * ctx, uint8_t config);

// special methods for MotionApps 2.0 implementation
//#ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS20
uint8_t *dmpPacketBuffer;
uint16_t dmpPacketSize;

uint8_t MPU6050_dmpInitialize(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpPacketAvailable(mpu_ctx_t * ctx);

uint8_t MPU6050_dmpSetFIFORate(mpu_ctx_t * ctx, uint8_t fifoRate);
uint8_t MPU6050_dmpGetFIFORate(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpGetSampleStepSizeMS(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpGetSampleFrequency(mpu_ctx_t * ctx);
int32_t MPU6050_dmpDecodeTemperature(mpu_ctx_t * ctx, uint8_t tempReg);

// Register callbacks after a packet of FIFO data is processed
//uint8_t MPU6050_dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t MPU6050_dmpUnregisterFIFORateProcess(inv_obj_func func);
uint8_t MPU6050_dmpRunFIFORateProcesses(mpu_ctx_t * ctx);

// Setup FIFO for various output
uint8_t MPU6050_dmpSendQuaternion(uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendPacketNumber(uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

// Get Fixed Point data from FIFO
uint8_t MPU6050_dmpGetAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
uint8_t MPU6050_dmpGet6AxisQuaternion(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGet6AxisQuaternion(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet);
uint8_t MPU6050_dmpGetRelativeQuaternion(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetRelativeQuaternion(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpSetLinearAccelFilterCoefficient(float coef);
uint8_t MPU6050_dmpGetLinearAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
uint8_t MPU6050_dmpGetLinearAccelInWorld(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccelInWorld(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t MPU6050_dmpGetGyroAndAccelSensor(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroAndAccelSensor(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroSensor(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroSensor(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetControlData(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetTemperature(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t MPU6050_dmpGetUnquantizedAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetUnquantizedAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuantizedAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuantizedAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetExternalSensorData(mpu_ctx_t * ctx, int32_t *data, uint16_t size, const uint8_t* packet);
uint8_t MPU6050_dmpGetEIS(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);

uint8_t MPU6050_dmpGetEuler(float *data, Quaternion *q);
uint8_t MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

// Get Floating Point data from FIFO
uint8_t MPU6050_dmpGetAccelFloat(float *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MPU6050_dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t MPU6050_dmpReadAndProcessFIFOPacket(mpu_ctx_t * ctx, uint8_t numPackets, uint8_t *processed=NULL);

uint8_t MPU6050_dmpSetFIFOProcessedCallback(void (*func) (void));

uint8_t MPU6050_dmpInitFIFOParam(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpCloseFIFO(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpSetGyroDataSource(mpu_ctx_t * ctx, uint8_t source);
uint8_t MPU6050_dmpDecodeQuantizedAccel(mpu_ctx_t * ctx);
uint32_t MPU6050_dmpGetGyroSumOfSquare(mpu_ctx_t * ctx);
uint32_t MPU6050_dmpGetAccelSumOfSquare(mpu_ctx_t * ctx);
void MPU6050_dmpOverrideQuaternion(long *q);
uint16_t MPU6050_dmpGetFIFOPacketSize(mpu_ctx_t * ctx);
//#endif

// special methods for MotionApps 4.1 implementation
//#ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS41
uint8_t *dmpPacketBuffer;
uint16_t dmpPacketSize;

uint8_t MPU6050_dmpInitialize(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpPacketAvailable(mpu_ctx_t * ctx);

uint8_t MPU6050_dmpSetFIFORate(mpu_ctx_t * ctx, uint8_t fifoRate);
uint8_t MPU6050_dmpGetFIFORate(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpGetSampleStepSizeMS(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpGetSampleFrequency(mpu_ctx_t * ctx);
int32_t MPU6050_dmpDecodeTemperature(mpu_ctx_t * ctx, uint8_t tempReg);

// Register callbacks after a packet of FIFO data is processed
//uint8_t MPU6050_dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t MPU6050_dmpUnregisterFIFORateProcess(inv_obj_func func);
uint8_t MPU6050_dmpRunFIFORateProcesses(mpu_ctx_t * ctx);

// Setup FIFO for various output
uint8_t MPU6050_dmpSendQuaternion(uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendPacketNumber(uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t MPU6050_dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

// Get Fixed Point data from FIFO
uint8_t MPU6050_dmpGetAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
uint8_t MPU6050_dmpGet6AxisQuaternion(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGet6AxisQuaternion(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet);
uint8_t MPU6050_dmpGetRelativeQuaternion(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetRelativeQuaternion(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetMag(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpSetLinearAccelFilterCoefficient(float coef);
uint8_t MPU6050_dmpGetLinearAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
uint8_t MPU6050_dmpGetLinearAccelInWorld(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccelInWorld(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t MPU6050_dmpGetGyroAndAccelSensor(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroAndAccelSensor(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroSensor(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroSensor(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetControlData(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetTemperature(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t MPU6050_dmpGetUnquantizedAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetUnquantizedAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuantizedAccel(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuantizedAccel(mpu_ctx_t * ctx, int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetExternalSensorData(mpu_ctx_t * ctx, int32_t *data, uint16_t size, const uint8_t* packet);
uint8_t MPU6050_dmpGetEIS(mpu_ctx_t * ctx, int32_t *data, const uint8_t* packet);

uint8_t MPU6050_dmpGetEuler(float *data, Quaternion *q);
uint8_t MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

// Get Floating Point data from FIFO
uint8_t MPU6050_dmpGetAccelFloat(float *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MPU6050_dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t MPU6050_dmpReadAndProcessFIFOPacket(mpu_ctx_t * ctx, uint8_t numPackets, uint8_t *processed=NULL);

uint8_t MPU6050_dmpSetFIFOProcessedCallback(void (*func) (void));

uint8_t MPU6050_dmpInitFIFOParam(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpCloseFIFO(mpu_ctx_t * ctx);
uint8_t MPU6050_dmpSetGyroDataSource(mpu_ctx_t * ctx, uint8_t source);
uint8_t MPU6050_dmpDecodeQuantizedAccel(mpu_ctx_t * ctx);
uint32_t MPU6050_dmpGetGyroSumOfSquare(mpu_ctx_t * ctx);
uint32_t MPU6050_dmpGetAccelSumOfSquare(mpu_ctx_t * ctx);
void MPU6050_dmpOverrideQuaternion(long *q);
uint16_t MPU6050_dmpGetFIFOPacketSize(mpu_ctx_t * ctx);
#else
#endif

#endif