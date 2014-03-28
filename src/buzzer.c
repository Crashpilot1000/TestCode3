#include "board.h"
#include "mw.h"

static uint8_t buzzerIsOn = 0, beepDone = 0, toggleBeep = 0;
static uint32_t buzzerLastToggleTime;

static void beep(uint16_t pulse)
{
    if (!buzzerIsOn && (currentTimeMS >= (buzzerLastToggleTime + 50)))              // Buzzer is off and long pause time is up -> turn it on
    {
        buzzerIsOn = 1;
        systemBeep(true);
        buzzerLastToggleTime = currentTimeMS;                                       // save the time the buzer turned on
    }
    else if (buzzerIsOn && (currentTimeMS >= buzzerLastToggleTime + pulse))         // Buzzer is on and time is up -> turn it off
    {
        buzzerIsOn = 0;
        systemBeep(false);
        buzzerLastToggleTime = currentTimeMS;
        if (toggleBeep) toggleBeep--;
        beepDone = 1;
    }
}

static void beep_code(char first, char second, char third, char pause)
{
    char patternChar[4];
    uint16_t Duration = 50;                                                         // Vanilla default value
    static uint8_t icnt = 0;

    patternChar[0] = first;
    patternChar[1] = second;
    patternChar[2] = third;
    patternChar[3] = pause;
    switch(patternChar[icnt])
    {
    case 'M':
        Duration = 100;
        break;
    case 'L':
        Duration = 200;
        break;
    case 'D':
        Duration = 2000;
        break;
    case 'N':
        Duration = 0;
        break;
    }

    if (icnt < 3 && Duration) beep(Duration);
    if (icnt >= 3 && (buzzerLastToggleTime < currentTimeMS - Duration))
    {
        icnt = 0;
        toggleBeep = 0;
    }
    if (beepDone || !Duration)
    {
        if (icnt < 3) icnt++;
        beepDone = 0;
        buzzerIsOn = 0;
        systemBeep(false);
    }
}

void buzzer(uint8_t warn_vbat)
{
    uint8_t warn_noGPSfix = 0, warn_failsafe = 0;
    if (feature(FEATURE_FAILSAFE))
    {
        if (failsafeCnt > (5 * cfg.fs_delay) && f.ARMED)
        {
            if (failsafeCnt > 5 * (cfg.fs_delay + cfg.fs_ofdel)) warn_failsafe = 2; // start "find me" signal after landing
            else warn_failsafe = 1;                                                 // set failsafe warning level to 1 while landing
        }
        if (failsafeCnt > (5 * cfg.fs_delay) && !f.ARMED) warn_failsafe = 2;        // tx turned off while motors are off: start "find me" signal
    }

    if (sensors(SENSOR_GPS) && !GPS_FIX && (rcOptions[BOXGPSHOME] || rcOptions[BOXGPSHOLD] || rcOptions[BOXGPSAUTO]))
        warn_noGPSfix = 1;

    //===================== Priority driven Handling =====================
    // beepcode(length1,length2,length3,pause)
    // D: Double, L: Long, M: Middle, S: Short, N: None
    if      (warn_failsafe == 2)      beep_code('L','N','N','D');                   // failsafe "find me" signal
    else if (warn_failsafe)           beep_code('S','M','L','M');                   // failsafe landing active
    else if (warn_noGPSfix)           beep_code('S','S','N','M');
    else if (rcOptions[BOXBEEPERON])  beep_code('S','S','S','M');                   // beeperon
    else if (warn_vbat == 4)          beep_code('S','M','M','D');
    else if (warn_vbat == 2)          beep_code('S','S','M','D');
    else if (warn_vbat)               beep_code('S','M','N','D');
    else if (toggleBeep)              beep(50);                                     // fast confirmation beep
    else
    {
        buzzerIsOn = 0;
        systemBeep(false);
    }
}
