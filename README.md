TestCode3
=========

Note: Make sure your GPS has a groundplate!!

Note: I have an improved driver for BMP085 Baro boards.
Replace the code in drv_bmp085.c with this. (will be in next version anyway..)

Cheers
Rob



#include "board.h"
#include "mw.h"

#define BARO_OFF                 digitalLo(BARO_GPIO, BARO_PIN)
#define BARO_ON                  digitalHi(BARO_GPIO, BARO_PIN)

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14) == SET) EXTI_ClearITPendingBit(EXTI_Line14);
}

typedef struct
{
    int16_t  ac1;
    int16_t  ac2;
    int16_t  ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t  b1;
    int16_t  b2;
    int16_t  mb;
    int16_t  mc;
    int16_t  md;
} bmp085_smd500_calibration_param_t;

typedef struct
{
    bmp085_smd500_calibration_param_t cal;
    uint8_t mode;
    uint8_t chip_id, ml_version, al_version;
    uint8_t dev_addr;
} bmp085_t;

#define BMP085_I2C_ADDR         0x77
#define BMP085_CHIP_ID          0x55
#define BOSCH_PRESSURE_BMP085   85
#define BMP085_CHIP_ID_REG      0xD0
#define BMP085_VERSION_REG      0xD1
#define E_SENSOR_NOT_DETECTED   (char) 0
#define BMP085_PROM_START__ADDR 0xaa
#define BMP085_PROM_DATA__LEN   22
#define BMP085_T_MEASURE        0x2E                                             // temperature measurent
#define BMP085_P_MEASURE        0x34                                             // pressure measurement
#define BMP085_CTRL_MEAS_REG    0xF4
#define BMP085_ADC_OUT_MSB_REG  0xF6
#define BMP085_ADC_OUT_LSB_REG  0xF7
#define BMP085_CHIP_ID__POS     0
#define BMP085_CHIP_ID__MSK     0xFF
#define BMP085_CHIP_ID__LEN     8
#define BMP085_CHIP_ID__REG     BMP085_CHIP_ID_REG
#define BMP085_ML_VERSION__POS  0
#define BMP085_ML_VERSION__LEN  4
#define BMP085_ML_VERSION__MSK  0x0F
#define BMP085_ML_VERSION__REG  BMP085_VERSION_REG
#define BMP085_AL_VERSION__POS  4
#define BMP085_AL_VERSION__LEN  4
#define BMP085_AL_VERSION__MSK  0xF0
#define BMP085_AL_VERSION__REG  BMP085_VERSION_REG
#define SMD500_PARAM_MI         3791.0f                                          //calibration parameter
#define BMP085_GET_BITSLICE(regvar, bitname) (regvar & bitname##__MSK) >> bitname##__POS
#define BMP085_SET_BITSLICE(regvar, bitname, val) (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

static bmp085_t bmp085 = { { 0, } };
static bool     bmp085InitDone = false;
static uint16_t bmp085_ut;                                                       // static result of temperature measurement
static uint32_t bmp085_up;                                                       // static result of pressure measurement
static void     bmp085_get_cal_param(void);
static void     bmp085_start_ut(void);
static void     bmp085_get_ut(void);
static void     bmp085_start_up(void);
static void     bmp085_get_up(void);
static float    bmp085_calculate(void);

bool bmp085Detect(baro_t *baro)
{
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint8_t data;

    if (hse_value == 12000000) return false;                                     // Not supported with this frequency
    if (bmp085InitDone) return true;
    gpio.pin   = Pin_13;
    gpio.speed = Speed_2MHz;
    gpio.mode  = Mode_Out_PP;
    gpioInit(GPIOC, &gpio);
    gpio.pin   = Pin_14;
    gpio.mode  = Mode_IN_FLOATING;
    gpioInit(GPIOC, &gpio);
    BARO_ON

    gpioExtiLineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;                         // Enable and set EXTI10-15 Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    delay(20);                                                                   // datasheet says 10ms, we'll be careful and do 20.
    i2cRead(BMP085_I2C_ADDR, BMP085_CHIP_ID__REG, 1, &data);                     /* read Chip Id */
    bmp085.chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);

    if (bmp085.chip_id == BMP085_CHIP_ID)                                        /* get bitslice */
    {
        i2cRead(BMP085_I2C_ADDR, BMP085_VERSION_REG, 1, &data);                  /* read Version reg */
        bmp085.ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);        /* get ML Version */
        bmp085.al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);        /* get AL Version */
        bmp085_get_cal_param();                                                  /* readout bmp085 calibparam structure */
        bmp085InitDone = true;
        baro->ut_delay  = 4500;                                                  // Crashpilot OSS 3 TIMINGS
        baro->up_delay  = 22500;                                                 // Crashpilot Too fast but BMP does that!
        baro->baro_type = 1;                                                     // 1 = BMP 2 = MS
        baro->start_ut  = bmp085_start_ut;
        baro->get_ut    = bmp085_get_ut;
        baro->start_up  = bmp085_start_up;
        baro->get_up    = bmp085_get_up;
        baro->calculate = bmp085_calculate;
        return true;
    }
    BARO_OFF
    return false;
}

static void bmp085_start_ut(void)
{
    i2cWrite(BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);
}

static void bmp085_get_ut(void)
{
    uint8_t data[2];
    i2cRead(BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 2, data);
    bmp085_ut = (data[0] << 8) | data[1];
}

static void bmp085_start_up(void)
{
    uint8_t ctrl_reg_data;

    ctrl_reg_data = BMP085_P_MEASURE + (3 << 6);
    i2cWrite(BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, ctrl_reg_data);
}

static void bmp085_get_up(void)
{
    uint8_t data[3];
    i2cRead(BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 3, data);
    bmp085_up = (((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[2]) >> 5;
}

// NOTE: oversampling_setting = 3 is hardcoded since setting to less resolution is no option in this application
static float bmp085_calculate(void)
{
    float   tmp1, tmp2, tmp3, tmp4;
    tmp1  = (float)(((int32_t)bmp085_ut - (int32_t)bmp085.cal.ac6) * (int32_t)bmp085.cal.ac5) / 32768.0f;
    tmp2  = (float)((int32_t)bmp085.cal.mc << 11) / (tmp1 + (float)bmp085.cal.md);
    tmp3  = tmp1 + tmp2;
    BaroActualTemp = tmp3 * 0.00625f;                                            // Put out Temp (in C)
    tmp3 -= 4000.0f;
    tmp4  = tmp3 * tmp3 / 4096.0f;
    tmp1  = ((tmp4 * (float)bmp085.cal.b2) + (tmp3 * (float)bmp085.cal.ac2)) / 1024.0f;
    tmp2  = (float)bmp085.cal.ac1 * 8.0f + tmp1;
    tmp1  = (tmp3 * (float)bmp085.cal.ac3 / 8192.0f) + (tmp4 * (float)bmp085.cal.b1 / 65536.0f);
    tmp4  = ((float)bmp085.cal.ac4 * (tmp1 * 0.25f + 32768.0f)) / 32768.0f;
    if(!tmp4) return 0.0f;
    tmp3  = fabs((((float)bmp085_up - tmp2) * 100.0f) / tmp4) * 125.0f;
    tmp2  = tmp3 * 0.00390625f;
    tmp1  = tmp2 * tmp2 * 0.04635620f + tmp3 * -0.11225891f;
    return tmp3 + (tmp1 + SMD500_PARAM_MI) * 0.0625f;
}

static void bmp085_get_cal_param(void)
{
    uint8_t data[22];
    i2cRead(BMP085_I2C_ADDR, BMP085_PROM_START__ADDR, BMP085_PROM_DATA__LEN, data);
    bmp085.cal.ac1 = (data[0] << 8)  | data[1];
    bmp085.cal.ac2 = (data[2] << 8)  | data[3];
    bmp085.cal.ac3 = (data[4] << 8)  | data[5];
    bmp085.cal.ac4 = (data[6] << 8)  | data[7];
    bmp085.cal.ac5 = (data[8] << 8)  | data[9];
    bmp085.cal.ac6 = (data[10] << 8) | data[11];
    bmp085.cal.b1  = (data[12] << 8) | data[13];
    bmp085.cal.b2  = (data[14] << 8) | data[15];
    bmp085.cal.mb  = (data[16] << 8) | data[17];
    bmp085.cal.mc  = (data[18] << 8) | data[19];
    bmp085.cal.md  = (data[20] << 8) | data[21];
}

