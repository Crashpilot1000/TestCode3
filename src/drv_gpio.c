#include "board.h"

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config)
{
    uint32_t pinpos, currentmode, shift, tmp;
    for (pinpos = 0; pinpos < 16; pinpos++)
    {
        if (config->pin & (0x1 << pinpos))                                  // are we doing this pin?
        {
            __IO uint32_t *cr = &gpio->CRL + (pinpos / 8);                  // reference CRL or CRH, depending whether pin number is 0..7 or 8..15
            currentmode = config->mode & 0x0F;                              // mask out extra bits from pinmode, leaving just CNF+MODE
            shift = (pinpos % 8) * 4;                                       // offset to CNF and MODE portions of CRx register
            tmp   = *cr;                                                    // Read out current CRx value
            if (config->mode & 0x10) currentmode |= config->speed;          // if we're in output mode, add speed too.
            tmp &= ~(0xF << shift);                                         // Mask out 4 bits
            tmp |= currentmode << shift;                                    // apply current pinmode
            *cr  = tmp;
            if (config->mode == Mode_IPD) gpio->ODR &= ~(1U << pinpos);     // Special handling for IPD/IPU
            else if (config->mode == Mode_IPU) gpio->ODR |= (1U << pinpos);
        }
    }
}

void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
    uint32_t tmp = 0x00;

    tmp = ((uint32_t)0x0F) << (0x04 * (pinsrc & (uint8_t)0x03));
    AFIO->EXTICR[pinsrc >> 0x02] &= ~tmp;
    AFIO->EXTICR[pinsrc >> 0x02] |= (((uint32_t)portsrc) << (0x04 * (pinsrc & (uint8_t)0x03)));
}
