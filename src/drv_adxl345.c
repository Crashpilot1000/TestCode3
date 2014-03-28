#include "board.h"

// ADXL345, Alternative address mode 0x53
#define ADXL345_ADDRESS     0x53

// Registers
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_INT_ENABLE  0x2E
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATA_OUT    0x32
#define ADXL345_FIFO_CTL    0x38

// BW_RATE values
#define ADXL345_RATE_50     0x09
#define ADXL345_RATE_100    0x0A
#define ADXL345_RATE_200    0x0B
#define ADXL345_RATE_400    0x0C
#define ADXL345_RATE_800    0x0D
#define ADXL345_RATE_1600   0x0E
#define ADXL345_RATE_3200   0x0F

// various register values
#define ADXL345_POWER_MEAS  0x08
#define ADXL345_FULL_RANGE  0x08
#define ADXL345_RANGE_2G    0x00
#define ADXL345_RANGE_4G    0x01
#define ADXL345_RANGE_8G    0x02
#define ADXL345_RANGE_16G   0x03
#define ADXL345_FIFO_STREAM 0x80

static void adxl345Init(void);
static void adxl345Read(int16_t *accelData);
static void adxl345Align(int16_t *accelData);

bool adxl345Detect(sensor_t *acc)
{
    bool ack = false;
    uint8_t sig = 0;

    if (hse_value == 12000000) return false;    // Not supported with this frequency
    ack = i2cRead(ADXL345_ADDRESS, 0x00, 1, &sig);
    if (!ack || sig != 0xE5) return false;
    acc->init  = adxl345Init;
    acc->read  = adxl345Read;
    acc->align = adxl345Align;
    return true;
}

static void adxl345Init(void)
{
    i2cWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL,   ADXL345_POWER_MEAS);
    i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, ADXL345_FULL_RANGE | ADXL345_RANGE_8G);
    i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE,     ADXL345_RATE_100);
}

static void adxl345Read(int16_t *accelData)
{
    uint8_t buf[8];
    i2cRead(ADXL345_ADDRESS, ADXL345_DATA_OUT, 6, buf);
    accelData[0] = (((int16_t)buf[1]) << 8) | buf[0];
    accelData[1] = (((int16_t)buf[3]) << 8) | buf[2];
    accelData[2] = (((int16_t)buf[5]) << 8) | buf[4];
}

static void adxl345Align(int16_t *accData)
{
    // official direction is RPY, nothing to change here.
}
