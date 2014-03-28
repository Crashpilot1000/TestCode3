#include "board.h"
#include "mw.h"

/*
    uint16_t MaxWPDatasets;
    uint16_t FDUsedDatasets;                // Number of valid datasets of current type
    uint8_t  FloppyDisk[FDByteSize];        // Reserve ca. 2200 general purpose Bytes

==============
FD_MODE_WPLIST
==============

That's how mwii wants it.
 serialize8(wp_no);
 serialize32(lat);
 serialize32(lon);
 serialize32(AltHold);           //altitude (cm) will come here -- temporary implementation to test feature with apps
 serialize16(0);                 //heading  will come here (deg)
 serialize16(0);                 //time to stay (ms) will come here 
 serialize8(0);                  //nav flag will come here
= 18 Bytes that means 20 Bytes would be needed.

That's how we do it.
wp_no not necessary, is defined by memoryposition

typedef union
{
    struct
    {
        int32_t WPGPS[2];
        int16_t WPHight;  // In meters
        int16_t WPHead;   // +-180 Degree
        uint8_t WPTime;   // MAV_CMD_NAV_LOITER_TIME Time in Seconds; MAV_CMD_NAV_WAYPOINT 1 means slow corner, 0 fast corner;
        uint8_t WPPara1;  // MAV_CMD_NAV_WAYPOINT Hitradius in m
        int8_t  WPPara2;  // MAV_CMD_NAV_LOITER_TURNS +cw -ccw, 0 not possible
        uint8_t WPCMD;    // What action
    };
    uint8_t bytes[16];    // Sizeof reports 16 Bytes.
} wp_t;

typedef enum MavlnkCommand
{
    ML_NAV_WAYPOINT = 0,
    ML_NAV_LOITER_UNLIM,
    ML_NAV_LOITER_TURNS,
    ML_NAV_LOITER_TIME,
    ML_NAV_RETURN_TO_LAUNCH,
    ML_NAV_LAND,
    ML_NAV_TAKEOFF
} MavlnkCommand;
*/

static uint16_t MaxWPDatasets = 0;

void FloppyClear(void)
{
    MaxWPDatasets      = FDByteSize / sizeof(wp_t);
    cfg.FDUsedDatasets = 0;                                                       // Declare empty buffer
}

bool FloppyWriteWP (uint16_t *Nr, wp_t *wppnt)
{
    uint16_t i, ByteOffset;
    if (!MaxWPDatasets) FloppyClear();                                            // Check 1st run
    if ((cfg.FDUsedDatasets >= MaxWPDatasets) || (*Nr > (MaxWPDatasets - 1)) || !MaxWPDatasets) return false;
    ByteOffset = *Nr * sizeof(wp_t);                                              // Get Dataset Offset
    for(i = 0; i < sizeof(wp_t); i++) cfg.FloppyDisk[ByteOffset + i] = (*wppnt).bytes[i];  // Write Dataset
    cfg.FDUsedDatasets ++;
    ScheduleEEPROMwriteMS = millis() + 1000;                                       // Schedule write after last Dataset written.
    return true;
}

bool FloppyReadWP (uint16_t *Nr, wp_t *wppnt)
{
    uint16_t i, ByteOffset;
    if ((*Nr > (cfg.FDUsedDatasets - 1)) || !cfg.FDUsedDatasets) return false;
    ByteOffset = *Nr * sizeof(wp_t);                                              // Get Dataset Offset
    for(i = 0; i < sizeof(wp_t); i++) (*wppnt).bytes[i] = cfg.FloppyDisk[ByteOffset + i]; // Read Dataset
    return true;
}
