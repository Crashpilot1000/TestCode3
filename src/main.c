#include "board.h"
#include "mw.h"

extern uint8_t useServo;
extern rcReadRawDataPtr rcReadRawFunc;
extern uint16_t pwmReadRawRC(uint8_t chan);
bool   SerialRCRX = false;

static void _putc(void *p, char c)
{
    uartWrite(c);
}

int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;

#if 0
    // PC12, PA15 using this to write asm for bootloader :)
    RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO; // GPIOA/C+AFIO only
    AFIO->MAPR &= 0xF0FFFFFF;
    AFIO->MAPR  = 0x02000000;
    GPIOA->CRH  = 0x34444444;                           // PIN 15 Output 50MHz
    GPIOA->BRR  = 0x8000;                               // set low 15
    GPIOC->CRH  = 0x44434444;                           // PIN 12 Output 50MHz
    GPIOC->BRR  = 0x1000;                               // set low 12
#endif

#if 0
    // using this to write asm for bootloader :)
    RCC->APB2ENR |= RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO; // GPIOB + AFIO
    AFIO->MAPR &= 0xF0FFFFFF;
    AFIO->MAPR  = 0x02000000;
    GPIOB->BRR  = 0x18;                                 // set low 4 & 3
    GPIOB->CRL  = 0x44433444;                           // PIN 4 & 3 Output 50MHz
#endif

    systemInit();
    init_printf(NULL, _putc);

    checkFirstTime(false);
    readEEPROM();

    // configure power ADC
    if (cfg.power_adc_channel && (cfg.power_adc_channel == 1 || cfg.power_adc_channel == 9)) adc_params.powerAdcChannel = cfg.power_adc_channel;
    else
    {
        adc_params.powerAdcChannel = 0;
        cfg.power_adc_channel = 0;
    }
    adcInit(&adc_params);
    serialInit(cfg.serial_baudrate);
    mixerInit();                                        // this will set useServo var depending on mixer type and NumberOfMotors

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (cfg.mixerConfiguration == MULTITYPE_AIRPLANE || cfg.mixerConfiguration == MULTITYPE_FLYING_WING) pwm_params.airplane = true;
    else pwm_params.airplane = false;
    pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SPEKTRUM) || feature(FEATURE_GRAUPNERSUMH); // spektrum/Graupner Sumh support uses UART too
    pwm_params.usePPM  = feature(FEATURE_PPM);
    
    if (feature(FEATURE_SPEKTRUM) || feature(FEATURE_GRAUPNERSUMH))  // disable RC PWM inputs if using spektrum/Graupner Sumh
    {
        cfg.devorssi = 0;
        pwm_params.enablePWMInput = false;
    }
    else pwm_params.enablePWMInput = true;

    pwm_params.useServos    = useServo;
    pwm_params.extraServos  = (cfg.gbl_flg & GIMBAL_FORWARDAUX) || (feature(FEATURE_LED) && cfg.LED_Type == 1);
    pwm_params.motorPwmRate = cfg.esc_pwm;
    pwm_params.servoPwmRate = cfg.srv_pwm;
    pwm_params.useRC5       = false;
    pwm_params.useRC6       = false;
    pwm_params.useRC78      = false;
    pwm_params.usePWM56     = false;

#ifdef SONAR
    if (feature(FEATURE_SONAR))                         // Set Sonar PWM Channels depending on Rc configuration. I2C Sonar needs no further attention here
    {
        switch(cfg.snr_type)                            // 0 = PWM56, 1 = RC78, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4 = MBRC78
        {
        case 0:
        case 3:
            if (NumberOfMotors < 5 && !feature(FEATURE_SERVO_TILT)) pwm_params.usePWM56 = true; // Only do this with 4 Motors or less
            break;
        case 1:
        case 4:
            if (feature(FEATURE_PPM)) pwm_params.useRC78  = true; // ToDo: We should check for max motornumbers as well here
        default:
            break;
        }
    }
#endif

    if (feature(FEATURE_PPM) && feature(FEATURE_LED) && (cfg.LED_Type == 2))
    {
        if (!cfg.LED_Pinout) pwm_params.useRC5 = true;
        else pwm_params.useRC6 = true;
    }

    switch (cfg.power_adc_channel)
    {
    case 1:
        pwm_params.adcChannel = PWM2;
        break;
    case 9:
        pwm_params.adcChannel = PWM8;
        break;
    default:
        pwm_params.adcChannel = 0;
        break;
    }

    pwmInit(&pwm_params);

    if (feature(FEATURE_SPEKTRUM))
    {
        SerialRCRX = true;
        spektrumInit();
        rcReadRawFunc = spektrumReadRawRC;
    }
	  else if (feature(FEATURE_GRAUPNERSUMH))
    {
        SerialRCRX = true;
        graupnersumhInit();
        rcReadRawFunc = graupnersumhReadRawRC;
    }
    else rcReadRawFunc = pwmReadRawRC;                  // configure PWM/CPPM read function.
    
    if (feature(FEATURE_GPS) && !SerialRCRX && !feature(FEATURE_PPM)) cfg.rc_auxch = min(cfg.rc_auxch, 2); // limit to 2 Aux if normal ppm rx & gps

    if (!feature(FEATURE_PASS))
    {
        if (feature(FEATURE_PPM) && feature(FEATURE_LED) && (cfg.LED_Type == 2)) ledToggleInit();
        LD1_ON();
        LD0_OFF();
        for (i = 0; i < 5; i++)
        {
            LED1_TOGGLE
            LED0_TOGGLE
            delay(10);
            systemBeep(true);
            delay(10);
            systemBeep(false);
        }
        LD0_OFF();
        LD1_OFF();
        SensorDetectAndINI();                           // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
        imuInit();                                      // Mag is initialized inside imuInit
        if (feature(FEATURE_VBAT)) batteryInit();       // Check battery type/voltage
        AvgCyclTime  = (float)cfg.looptime;             // Reduce runup on Inirun
        f.SMALL_ANGLES_25 = 1;
        if(cfg.stat_clear) ClearStats();
        baseflight_mavlink_init();                      // Always precalculate some Mavlink stuff, maybe needed
        while(1) loop();                                // Do Harakiri        
    }
    else                                                // We want feature pass, do the minimal wash - program :)
    {
        SensorDetectAndINI();
        writeAllMotors(cfg.esc_moff);                   // All Motors off
        LD0_ON();
        LD1_OFF();
        while(1) pass();                                // Do feature pass
    }
}

void HardFault_Handler(void)
{
    writeAllMotors(cfg.esc_moff);                       // fall out of the sky
    while (1);
}
