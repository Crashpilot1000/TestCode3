#include "board.h"
#include "mw.h"
// HMC5883L, default address 0x1E
// PB12 connected to MAG_DRDY on rev4 hardware

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE  2
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

//#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
//#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
//#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Y axis level when bias current is applied.
//#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)   // Low limit when gain is 5.
//#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)   // High limit when gain is 5.
bool maggainok;

bool hmc5883lDetect(void)
{
    uint8_t  ChipID[3];
    if(i2cRead(MAG_ADDRESS, 0x0A, 3, ChipID) && ChipID[0] == 'H' && ChipID[1] == '4' && ChipID[2] == '3') return true;
    else return false;
}

void hmc5883lInit(float *calGain)                               // THE RESULT IS ALIGNED
{
    gpio_config_t gpio;
    int16_t magADC[3];
    uint8_t i, k;
    float xyz_total[3] = { 0, 0, 0 }, multiplier;
    float SELF_TEST_GAUSS[3] = {1.16f, 1.16f, 1.08f};           // HMC58X3_*_SELF_TEST_GAUSS for X Y Z

    maggainok = true;
    if (hse_value == 8000000)
    {
        gpio.pin   = Pin_12;                                    // PB12 - MAG_DRDY output on rev4 hardware
        gpio.speed = Speed_2MHz;
        gpio.mode  = Mode_IN_FLOATING;
        gpioInit(GPIOB, &gpio);
    }
    else
    {
        if (hse_value == 12000000)
        {
            gpio.pin   = Pin_14;                                // PC14 - MAG_DRDY output on rev5 hardware
            gpioInit(GPIOC, &gpio);
        }
    }
    delay(100);
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS);
    // Reg A DOR = 0x010 + MS1, MS0 set to pos bias
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.

    if (cfg.mag_gain)
    {                                                           // not zero
        i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFB, 0x60);           // Set the Gain 2,5 GAUSS
        multiplier = 13200.0f;                                  // 660 * 2 * 10
    }
    else
    {                                                           // zero
        i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);           // Set the Gain 1,9 GAUSS
        multiplier = 16400.0f;                                  // 820 * 2 * 10
    }

    for (i = 0; i < 5; i++)                                     // read some datasets and discard!
    {
        delay(100);
        hmc5883lRead(magADC);
    }

    for (i = 0; i < 10; i++)                                    // Collect 10 samples
    {
        i2cWrite(MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(100);
        hmc5883lRead(magADC);                                   // Get the raw values in case the scales have already been changed.
        for (k = 0; k < 3; k++)
        {
            if(abs(magADC[k]) >= 2047) maggainok = false;       // Detect saturation. Official range (-2048, 2047)
            xyz_total[k] += (float)magADC[k];                   // Since the measurements are noisy, they should be averaged rather than taking the max.
        }
        LED1_TOGGLE
    }

    // Apply the negative bias. (Same gain)
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
    for (i = 0; i < 10; i++)
    {
        i2cWrite(MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(100);
        hmc5883lRead(magADC);                                   // Get the raw values in case the scales have already been changed.
        for (k = 0; k < 3; k++)
        {
            if(abs(magADC[k]) >= 2047) maggainok = false;       // Detect saturation. Official range (-2048, 2047)
            xyz_total[k] -= (float)magADC[k];                   // Since the measurements are noisy, they should be averaged rather than taking the max.
        }
        LED1_TOGGLE
    }

    // leave test mode
    // i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFA, 0x70);            // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFA, 0x78);               // Configuration Register A  -- 01111000  num samples: 8 ; output rate: 75Hz ; normal measurement mode
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);               // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2cWrite(MAG_ADDRESS, HMC58X3_R_MODE,  0x00);               // Mode register             -- 000000 00    continuous Conversion Mode
    delay(100);

    for (i = 0; i < 3; i++)
    {
        if(maggainok) calGain[i] = fabs(multiplier * SELF_TEST_GAUSS[i] / xyz_total[i]);
        else calGain[i] = 1.0f;
    }
}

void hmc5883lRead(int16_t *magData)                             // Read aligned BTW: The 5883 sends in that order: X Z Y
{
    uint8_t buf[6];
    int16_t swap;
    i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    magData[0] = (((int16_t)buf[0]) << 8) | buf[1];             // X
    magData[2] = (((int16_t)buf[2]) << 8) | buf[3];             // Z
    magData[1] = (((int16_t)buf[4]) << 8) | buf[5];             // Y
    if (cfg.align[ALIGN_MAG][0])
    {
        alignSensors(ALIGN_MAG, magData);
    }
    else
    {
        swap       =  magData[0];
        magData[0] =  magData[1];
        magData[1] = -swap;
        magData[2] = -magData[2];
    }
}
