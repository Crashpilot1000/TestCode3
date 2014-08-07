#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "printf.h"
#include "drv_system.h"                                     // timers, delays, etc
#include "drv_gpio.h"
#include "drv_i2c_lcd.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif

// Precalculated is better for gcc
#define RADX        0.017453292519f                         // #define RADX            M_PI / 180.0f
#define RADX10      0.001745329252f                         // #define RADX10          M_PI / 1800.0f
#define RADX100     0.000174532925f                         // #define RADX100         M_PI / 18000.0f
#define RADtoDEG    57.29577951308f                         // #define RADtoDEG     180.0f  / M_PI
#define RADtoDEG10  572.9577951308f                         // #define RADtoDEG10  1800.0f  / M_PI
#define RADtoDEG100 5729.577951308f                         // #define RADtoDEG100 18000.0f / M_PI

#define RCconstPI   0.159154943092f                         // 0.5f / M_PI;

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// max. number of supported channels 10 = 6 Aux + 4 (roll pitch yaw thr)
#define MAX_RC_CHANNELS 10                                  // Limit to 10 Channels 4 + 6 NOTE: DONT GO BELOW 8 !!!

#define MAX_MONITORED_MOTORS 8                              // max. number of monitored Motors for percentage statistic

// EEPROM Definitions !WARNING! WATCH OUT FOR stm32_flash.ld and EEPROM SIZE there is the Config at the end !!
#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT    128
#endif
#define FLASH_PAGE_SIZE     ((uint16_t)0x400)               // 1KB
#define FLASH_PAGES_FORCONFIG 3                             // 3KB was 1Page/KB before
#define FLASH_WRITE_ADDR    (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - FLASH_PAGES_FORCONFIG)) //#define FLASH_WRITE_ADDR (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))
#define FDByteSize 2340                                     // Defines the Bytesize of the Floppydisk

typedef enum
{
    SENSOR_ACC   = 1 << 0,
    SENSOR_BARO  = 1 << 1,
    SENSOR_MAG   = 1 << 2,
    SENSOR_SONAR = 1 << 3,
    SENSOR_GPS   = 1 << 4,
} AvailableSensors;

typedef enum AccelSensors                                   // Type of accelerometer used/detected
{
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
} AccelSensors;

typedef enum                                                // sync this with featureNames in cli.c
{
    FEATURE_PPM              = 1 <<  0,
    FEATURE_VBAT             = 1 <<  1,
    FEATURE_SPEKTRUM         = 1 <<  2,
    FEATURE_GRAUPNERSUMH     = 1 <<  3,
    FEATURE_SERVO_TILT       = 1 <<  4,
    FEATURE_LED              = 1 <<  5,
    FEATURE_GPS              = 1 <<  6,
    FEATURE_FAILSAFE         = 1 <<  7,
    FEATURE_SONAR            = 1 <<  8,
    FEATURE_PASS             = 1 <<  9,
    FEATURE_LCD              = 1 << 10,
} AvailableFeatures;

typedef void     (* sensorInitFuncPtr)(void);               // sensor init prototype
typedef void     (* sensorReadFuncPtr)(int16_t *data);      // sensor read and align prototype
typedef void     (* sensorReadFuncPtrFLT)(float *data);     // sensor read and align prototype
typedef float    (* baroCalculateFuncPtr)(void);            // baro calculation (returns altitude in cm based on static data collected)
typedef void     (* uartReceiveCallbackPtr)(uint16_t data); // used by uart2 driver to return frames to app
typedef uint16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data

typedef struct sensor_t
{
    sensorInitFuncPtr    init;
    sensorReadFuncPtr    read;
    sensorReadFuncPtr    align;
    sensorReadFuncPtrFLT temperature;
} sensor_t;

typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    uint8_t  baro_type;                                     // 1 = BMP 2 = MS
    sensorInitFuncPtr start_ut;
    sensorInitFuncPtr get_ut;
    sensorInitFuncPtr start_up;
    sensorInitFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }
#define digitalIn(p, i)     (p->IDR & i)

// Hardware definitions and GPIO
#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12
#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13

#define GYRO
#define ACC
#define MAG
#define BARO
#define LEDRING
#define SONAR

#define LED0_TOGGLE  digitalToggle(LED0_GPIO, LED0_PIN)     // Helpful macros
#define LED0_OFF     digitalHi(LED0_GPIO, LED0_PIN)
#define LED0_ON      digitalLo(LED0_GPIO, LED0_PIN)
#define LED1_TOGGLE  digitalToggle(LED1_GPIO, LED1_PIN)
#define LED1_OFF     digitalHi(LED1_GPIO, LED1_PIN)
#define LED1_ON      digitalLo(LED1_GPIO, LED1_PIN)

#include "drv_adc.h"
#include "drv_adxl345.h"
#include "drv_bmp085.h"
#include "drv_ms5611.h"
#include "drv_hmc5883l.h"
#include "drv_i2c.h"
#include "drv_ledring.h"
#include "drv_mma845x.h"
#include "drv_mpu3050.h"
#include "drv_mpu6050.h"
#include "drv_l3g4200d.h"
#include "drv_pwm.h"
#include "drv_uart.h"
#include "drv_sonar.h"
#include "drv_gps.h"
#include "drv_graupnersumh.h"
#include "drv_spektrum.h"
