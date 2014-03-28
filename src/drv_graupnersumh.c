//  graupnersumh.c
//
//  by Copterwerkstatt
//  http://fpv-treff.de/viewtopic.php?f=18&t=1368&start=3020#p44535
//

#include "board.h"
#include "mw.h"

// driver for graupner sumh receiver using UART2 (freeing up more motor outputs for stuff)
#define GSUM_MAX_CHANNEL 8
#define GSUM_FRAME_SIZE 21

static bool rcFrameComplete = false;
volatile uint8_t gsumFrame[GSUM_FRAME_SIZE];
static void graupnersumhDataReceive(uint16_t c);

void graupnersumhInit(void)
{
    uart2Init(115200, graupnersumhDataReceive, true);
}


static void graupnersumhDataReceive(uint16_t c)
{
    uint32_t gsumTime;
    static uint32_t gsumTimeLast, gsumTimeInterval;
    static uint8_t  gsumFramePosition;

    gsumTime = micros();
    gsumTimeInterval = gsumTime - gsumTimeLast;
    gsumTimeLast = gsumTime;
    if (gsumTimeInterval > 5000) gsumFramePosition = 0;
    gsumFrame[gsumFramePosition] = (uint8_t)c;
    if (gsumFramePosition == GSUM_FRAME_SIZE - 1)
    {
        rcFrameComplete = true;
        failsafeCnt = 0;   // clear FailSafe counter
    }
    else
    {
        gsumFramePosition++;
    }
}

bool graupnersumhFrameComplete(void)
{
    return rcFrameComplete;
}

uint16_t graupnersumhReadRawRC(uint8_t chan)
{
    uint16_t data;
    static uint32_t gsumChannelData[GSUM_MAX_CHANNEL];
    static uint8_t gsumRcChannelMap[GSUM_MAX_CHANNEL] = {1,2,3,0,4,5,6,7};
    uint8_t b;
    if (rcFrameComplete)
    {
        if ((gsumFrame[0] == 0xA8) && (gsumFrame[GSUM_FRAME_SIZE - 2] == 0))
        {
            for (b = 0; b < GSUM_MAX_CHANNEL; b++)
            {
                gsumChannelData[b] = (((uint32_t)(gsumFrame[(b << 1) + 3]) << 8) + gsumFrame[(b << 1) + 4]) / 6.4 - 375;
            }
        }
        rcFrameComplete = false;
    }

    if (chan >= GSUM_MAX_CHANNEL) data = cfg.rc_mid;
    else data = gsumChannelData[gsumRcChannelMap[chan]];
    return data;
}
