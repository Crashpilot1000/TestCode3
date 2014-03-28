#include "board.h"
#include "mw.h"

static volatile uint32_t usTicks = 0;             // cycles per microsecond
static volatile uint32_t sysTickUptime = 0;       // current uptime for 1kHz systick timer. will rollover after 49 days
void SetSysClock(void);                           // from system_stm32f10x.c

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
}

// r329 fix for micros() when not running at default 72MHz.
// Was affecting intrc-only operation at 64MHz.
// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do
    {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}


void systemInit(void)
{
    struct
    {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] =
    {
        {
            .gpio = LED0_GPIO,
            .cfg  = { LED0_PIN, Mode_Out_PP, Speed_2MHz }
        },
        {
            .gpio = LED1_GPIO,
            .cfg  = { LED1_PIN, Mode_Out_PP, Speed_2MHz }
        },
        {
            .gpio = BEEP_GPIO,
            .cfg  = { BEEP_PIN, Mode_Out_OD, Speed_2MHz }
        },
    };
    gpio_config_t gpio;
    uint32_t i;
    uint8_t gpio_count = sizeof(gpio_setup) / sizeof(gpio_setup[0]);

    // Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
    // Configure the Flash Latency cycles and enable prefetch buffer
    SetSysClock();

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    gpio.pin = Pin_All;
    gpio.mode = Mode_AIN;
    gpioInit(GPIOA, &gpio);
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);

    // Turn off JTAG port 'cause we're using the GPIO for leds
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;

    systemBeep(false);

    LED0_OFF
    LED1_OFF

    for (i = 0; i < gpio_count; i++)
    {
        if (hse_value == 12000000 && gpio_setup[i].cfg.mode == Mode_Out_OD) gpio_setup[i].cfg.mode = Mode_Out_PP;
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // Configure the rest of the stuff
    delay(200);                                       // Daddy.W & Crashpilot sleep for 200ms
    i2cInit(I2C2);
    initI2cLCD(false);                                // if connected, display startup message, Johannes
//  spiInit();
    delay(100);                                       // sleep for 100ms
}

#if 1
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}
#else
void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;)
    {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

void delay(uint32_t ms)
{
    while (ms--) delayMicroseconds(1000);
}

void failureMode(uint8_t mode)
{
    LED1_ON
    LED0_OFF
    while (1)
    {
        LED1_TOGGLE
        LED0_TOGGLE
        delay(475 * mode - 2);
        systemBeep(true);
        delay(25);
        systemBeep(false);
    }
}

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    delay(1000);
    if (toBootloader)
    {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC
        *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
    }
    NVIC_SystemReset();                         //    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

static void beepRev4(bool onoff)
{
    if (onoff) digitalLo(BEEP_GPIO, BEEP_PIN)
    else digitalHi(BEEP_GPIO, BEEP_PIN)
}

static void beepRev5(bool onoff)                // rev5 needs inverted beeper. oops.
{
    if (onoff) digitalHi(BEEP_GPIO, BEEP_PIN)
    else digitalLo(BEEP_GPIO, BEEP_PIN)
}

void systemBeep(bool onoff)
{
    if (hse_value == 12000000) beepRev5(onoff);
    else beepRev4(onoff);
}
