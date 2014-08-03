#include "board.h"
#include "mw.h"

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 hardware
#define MPU6050_ADDRESS     0x68
#define MPU_RA_SMPLRT_DIV   0x19
#define MPU_RA_CONFIG       0x1A
#define MPU_RA_GYRO_CONFIG  0x1B
#define MPU_RA_ACCEL_CONFIG 0x1C
#define MPU_RA_INT_PIN_CFG  0x37
#define MPU_RA_ACCEL_XOUT_H 0x3B
#define MPU_RA_TEMP_OUT_H   0x41
#define MPU_RA_GYRO_XOUT_H  0x43
#define MPU_RA_PWR_MGMT_1   0x6B
#define MPU_RA_WHO_AM_I     0x75
#define BITS_FS_500DPS      0x08
#define BITS_FS_1000DPS     0x10
#define BITS_FS_2000DPS     0x18
#define BITS_FS_2G          0x00
#define BITS_FS_4G          0x08
#define BITS_FS_8G          0x10
#define BITS_FS_16G         0x18

static void mpu6050AccInit(void);
static void mpu6050AccRead(int16_t *accData);
static void mpu6050AccAlign(int16_t *accData);
static void mpu6050GyroInit(void);
static void mpu6050GyroRead(int16_t *gyroData);
static void mpu6050GyroAlign(int16_t *gyroData);
static void mpu6050TempRead(float *tempData);

bool mpu6050Detect(sensor_t * acc, sensor_t * gyro)
{
    bool ack;
    uint8_t sig;
    delay(35);                  // datasheet page 13 says 30ms. other stuff could have been running meanwhile. but we'll be safe
    ack = i2cRead(MPU6050_ADDRESS, MPU_RA_WHO_AM_I, 1, &sig);
    if (!ack) return false;

    // So like, MPU6xxx has a "WHO_AM_I" register, that is used to verify the identity of the device.
    // The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C address.
    // The least significant bit of the MPU-60X0’s I2C address is determined by the value of the AD0 pin. (we know that already).
    // But here's the best part: The value of the AD0 pin is not reflected in this register.
    if (sig != (MPU6050_ADDRESS & 0x7e)) return false;
    acc->init         = mpu6050AccInit;
    acc->read         = mpu6050AccRead;
    acc->align        = mpu6050AccAlign;
    gyro->init        = mpu6050GyroInit;
    gyro->read        = mpu6050GyroRead;
    gyro->align       = mpu6050GyroAlign;
    gyro->temperature = mpu6050TempRead;
    return true;
}

static void mpu6050AccInit(void) {
// ACC Init stuff. Moved into gyro init because the reset would screw up accel config.
}

static void mpu6050GyroInit(void)
{
    uint8_t DLPFCFG;
    gpio_config_t gpio;

    gpio.pin   = Pin_13;                                     // MPU_INT output on rev4/5 hardware (PB13, PC13)
    gpio.speed = Speed_2MHz;
    gpio.mode  = Mode_IN_FLOATING;
    if (hse_value == 8000000) gpioInit(GPIOB, &gpio);
    else if (hse_value == 12000000) gpioInit(GPIOC, &gpio);

    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(5);
    i2cWrite(MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    /*  DLPF_CFG 0: ACC 260Hz // GYRO 256Hz
    	  DLPF_CFG 1: ACC 184Hz // GYRO 188Hz
    	  DLPF_CFG 2: ACC 94Hz  // GYRO  98Hz
    	  DLPF_CFG 3: ACC 44Hz  // GYRO  42Hz
    	  DLPF_CFG 4: ACC 21Hz  // GYRO  20Hz
    	  DLPF_CFG 5: ACC 10Hz  // GYRO  10Hz
    	  DLPF_CFG 6: ACC  5Hz  // GYRO   5Hz*/
    switch (cfg.gy_lpf)
    {
    case 256:
        DLPFCFG = 0;
        break;
    case 188:
        DLPFCFG = 1;
        break;
    case 98:
        DLPFCFG = 2;
        break;
    default:
        cfg.gy_lpf = 42;                                     // Feedback for CLI if user typed in something like "90"
    case 42:
        DLPFCFG = 3;
        break;
    case 20:
        DLPFCFG = 4;
        break;
    case 10:
        DLPFCFG = 5;
        break;
    case 5:
        DLPFCFG = 6;
        break;
    }
    i2cWrite(MPU6050_ADDRESS, MPU_RA_CONFIG, DLPFCFG);       // CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, BITS_FS_2000DPS);
    i2cWrite(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, BITS_FS_8G);
}

static void mpu6050AccRead(int16_t *accData)
{
    uint8_t buf[6];
    i2cRead(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (((int16_t)buf[0]) << 8) | buf[1];
    accData[1] = (((int16_t)buf[2]) << 8) | buf[3];
    accData[2] = (((int16_t)buf[4]) << 8) | buf[5];
}

static void mpu6050GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    i2cRead(MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (((int16_t)buf[0]) << 8) | buf[1];         // Changed to full resolution here
    gyroData[1] = (((int16_t)buf[2]) << 8) | buf[3];
    gyroData[2] = (((int16_t)buf[4]) << 8) | buf[5];
}

static void mpu6050TempRead(float *tempData)
{
    uint8_t buf[2];
    int16_t temp;
    i2cRead(MPU6050_ADDRESS, MPU_RA_TEMP_OUT_H, 2, buf);
    temp = (((int16_t)buf[0]) << 8) | buf[1];
    *tempData = 36.53f + ((float)temp / 340.0f);             // That is what the invense doc says. Here Arduinopage: *tempData = ((float)temp + 12412.0f) / 340.0f;
}

static void mpu6050AccAlign(int16_t *accData)
{
    int16_t swap = accData[0];
    accData[0] =  accData[1];                                // official direction is RPY
    accData[1] = -swap;
}

static void mpu6050GyroAlign(int16_t *gyroData)
{
    gyroData[2] = -gyroData[2];
}

void MPU6050ReadAllShit(int16_t *accData, float *tempData, int16_t *gyroData)
{
//  0x3B acc  6 bytes
//  0x41 temp 2 bytes
//  0x43 gyro 6 bytes
    uint8_t buf[14];
    int16_t temp;
    i2cRead(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 14, buf);
    accData[0]  = (((int16_t)buf[0])  << 8) | buf[1];
    accData[1]  = (((int16_t)buf[2])  << 8) | buf[3];
    accData[2]  = (((int16_t)buf[4])  << 8) | buf[5];
    temp        = (((int16_t)buf[6])  << 8) | buf[7];
    *tempData   = 36.53f + ((float)temp / 340.0f);           // That is what the invense doc says. Here Arduinopage: *tempData = ((float)temp + 12412.0f) / 340.0f;
    gyroData[0] = (((int16_t)buf[8])  << 8) | buf[9];
    gyroData[1] = (((int16_t)buf[10]) << 8) | buf[11];
    gyroData[2] = (((int16_t)buf[12]) << 8) | buf[13];
}

//Degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
