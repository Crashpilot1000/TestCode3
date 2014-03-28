#include "board.h"
#include "mw.h"

static uint8_t  ledIndex;                                   // Is the led GPIO channel enabled
static uint16_t ledtoggle_pin;
static uint8_t  ledDelay;                                   // make circle slower

static void ledEnable()
{
    digitalHi( GPIOA, ledtoggle_pin )
}

static void ledDisable()
{
    digitalLo( GPIOA, ledtoggle_pin )
}

void ledToggleInit()
{
    gpio_config_t gpio;
    if(cfg.LED_Pinout) ledtoggle_pin = Pin_7;               // RC6 (PB8) - 3.3v
    else               ledtoggle_pin = Pin_6;               // RC5 (PB8) - 3.3v
    gpio.pin   = ledtoggle_pin;
    gpio.speed = Speed_2MHz;
    gpio.mode  = Mode_Out_PP;
    gpioInit(GPIOA, &gpio);
    ledDisable();
}

void ledToggleUpdate(bool activated)                        // Ignore when we're not using leds
{
    uint8_t bit;
    if (activated)
    {
        ledDelay++;
        if (ledDelay == LED_Value_Delay)
        {
            ledDelay = 0;
            bit = (ledIndex++ >> 1) & 31;
            if (LED_Value & ( 1 << bit )) ledEnable();
            else ledDisable();
        }
    }
    else ledDisable();
}
