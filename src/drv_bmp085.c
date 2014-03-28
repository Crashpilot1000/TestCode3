#include "board.h"
#include "mw.h"

// BMP085, Standard address 0x77
//static bool convDone = false;
// static uint16_t convOverrun = 0;

#define BARO_OFF                 digitalLo(BARO_GPIO, BARO_PIN)
#define BARO_ON                  digitalHi(BARO_GPIO, BARO_PIN)

// EXTI14 for BMP085 End of Conversion Interrupt
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line14);
//        convDone = true;
    }
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
    bmp085_smd500_calibration_param_t cal_param;
    uint8_t mode;
    uint8_t chip_id, ml_version, al_version;
    uint8_t dev_addr;
    int16_t oversampling_setting;
} bmp085_t;

#define BMP085_I2C_ADDR         0x77
#define BMP085_CHIP_ID          0x55
#define BOSCH_PRESSURE_BMP085   85
#define BMP085_CHIP_ID_REG      0xD0
#define BMP085_VERSION_REG      0xD1
#define E_SENSOR_NOT_DETECTED   (char) 0
#define BMP085_PROM_START__ADDR 0xaa
#define BMP085_PROM_DATA__LEN   22
#define BMP085_T_MEASURE        0x2E                // temperature measurent
#define BMP085_P_MEASURE        0x34                // pressure measurement
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

#define BMP085_GET_BITSLICE(regvar, bitname) (regvar & bitname##__MSK) >> bitname##__POS
#define BMP085_SET_BITSLICE(regvar, bitname, val) (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

#define SMD500_PARAM_MI      3791.0f     //calibration parameter

static bmp085_t bmp085 = { { 0, } };
static bool     bmp085InitDone = false;
static uint16_t bmp085_ut;  // static result of temperature measurement
static uint32_t bmp085_up;  // static result of pressure measurement

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

    // Not supported with this frequency
    if (hse_value == 12000000) return false;
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

    delay(20); // datasheet says 10ms, we'll be careful and do 20. this is after ms5611 driver kills us, so longer the better.

    i2cRead(BMP085_I2C_ADDR, BMP085_CHIP_ID__REG, 1, &data);                     /* read Chip Id */
    bmp085.chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
    bmp085.oversampling_setting = 3;                                             // Crashpilot set to full res

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

    ctrl_reg_data = BMP085_P_MEASURE + (bmp085.oversampling_setting << 6);
    i2cWrite(BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, ctrl_reg_data);
}

/** read out up for pressure conversion
  depending on the oversampling ratio setting up can be 16 to 19 bit
   \return up parameter that represents the uncompensated pressure value
*/
static void bmp085_get_up(void)
{
    uint8_t data[3];
    i2cRead(BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 3, data);
    bmp085_up = (((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[2]) >> (8 - bmp085.oversampling_setting);
}

static float bmp085_calculate(void)
{
    int32_t  x1, x2, x3, b3, b5, b6;
    uint32_t b4, b7;
    float    pressure, tmp1, tmp2;
    // **** calculate B5 B6 ********   Temperature
    x1 = (((int32_t) bmp085_ut - (int32_t) bmp085.cal_param.ac6) * (int32_t) bmp085.cal_param.ac5) >> 15;
    x2 = ((int32_t) bmp085.cal_param.mc << 11) / (x1 + bmp085.cal_param.md);
    b5 = x1 + x2;
    BaroActualTemp = ((float)b5 + 8.0f) * 0.00625f; // Put out Temp (in C)
    b6 = b5 - 4000;
    // **** calculate B3 ***********
    x1 = (((b6 * b6) >> 12) * bmp085.cal_param.b2) >> 11;
    x2 = (bmp085.cal_param.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)bmp085.cal_param.ac1) * 4 + x3) << bmp085.oversampling_setting) + 2) >> 2; // the " * 4" will be optimized by compiler to bitshift
    // **** calculate B4 ***********
    x1 = (bmp085.cal_param.ac3 * b6) >> 13;
    x2 = (bmp085.cal_param.b1 * ((b6 * b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp085.cal_param.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t)bmp085_up - b3) * (50000 >> bmp085.oversampling_setting);
    if (b7 < 0x80000000) pressure = ((float)b7 * 2.0f) / (float)b4;              // Left that check just in case it will even help resolution with "floats"
    else pressure = ((float)b7 / (float)b4) * 2.0f;
    tmp1  = pressure * 0.00390625f;
    tmp1 *= tmp1 * 0.04635620f;
    tmp2  = pressure * -0.11225891f;
    return pressure + ((tmp1 + tmp2 + SMD500_PARAM_MI) * 0.0625f);
}

static void bmp085_get_cal_param(void)
{
    uint8_t data[22];
    i2cRead(BMP085_I2C_ADDR, BMP085_PROM_START__ADDR, BMP085_PROM_DATA__LEN, data);
    bmp085.cal_param.ac1 = (data[0] << 8)  | data[1];
    bmp085.cal_param.ac2 = (data[2] << 8)  | data[3];
    bmp085.cal_param.ac3 = (data[4] << 8)  | data[5];
    bmp085.cal_param.ac4 = (data[6] << 8)  | data[7];
    bmp085.cal_param.ac5 = (data[8] << 8)  | data[9];
    bmp085.cal_param.ac6 = (data[10] << 8) | data[11];
    bmp085.cal_param.b1  = (data[12] << 8) | data[13];
    bmp085.cal_param.b2  = (data[14] << 8) | data[15];
    bmp085.cal_param.mb  = (data[16] << 8) | data[17];
    bmp085.cal_param.mc  = (data[18] << 8) | data[19];
    bmp085.cal_param.md  = (data[20] << 8) | data[21];
}
