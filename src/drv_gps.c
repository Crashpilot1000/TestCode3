#include "board.h"
#include "mw.h"

#define MTK_BAUD_RATE_57600			"$PMTK251,57600*2C\r\n"
#define MTK_SBAS_INTEGRITYMODE	"$PMTK319,1*24\r\n"
#define MTK_OUTPUT_5HZ					"$PMTK220,200*2C\r\n"
#define MTK_NAVTHRES_OFF      	"$PMTK397,0*23\r\n"
#define MTK_SBAS_ON							"$PMTK313,1*2E\r\n"
#define MTK_WAAS_ON           	"$PMTK301,2*2E\r\n"
#define MTK_SET_BINARY					"$PGCMD,16,0,0,0,0,0*6A\r\n"
/*
 Crashpilot: Think about the dynamic model of ublox !
 Here is the chart. Interesting for us is PEDESTRIAN and PORTABLE (Default)
 http://www.diydrones.com/forum/topics/ac-2-9-1-vs-2-8-1-gps-accuracy?commentId=705844%3AComment%3A1131867

Ublox dynModel     Velocity m/s    Vertical Velocity m/s    Altitude m    Position Deviation
PORTABLE    = 0         310                50                 12000          Medium
STATIONARY  = 2          10                 6                  9000          Small
PEDESTRIAN  = 3          30                20                  9000          Small
AUTOMOTIVE  = 4          84                15                  6000          Medium
SEA         = 5          25                 5                   500          Medium
AIRBORNE_1G = 6         100               100                 50000          Large
AIRBORNE_2G = 7         250               100                 50000          Large
AIRBORNE_4G = 8         500               100                 50000          Large
*/

#define Pedestrian
const  uint32_t init_speed[5] = { 9600, 19200, 38400, 57600, 115200 };
static const uint8_t ubloxInit[] =
{
#ifdef Pedestrian
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x82,
#endif
     // NMEA - Sucks for the most part so will be disabling it.  These lines are only from original code for ref only
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,               // VGS: Course over ground and Ground speed
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,               // GSV: GNSS Satellites in View
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,               // GLL: Latitude and longitude, with time of position fix and status
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,               // GGA: Global positioning system fix data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,               // GSA: GNSS DOP and Active Satellites
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,               // RMC: Recommended Minimum data
    // now lets disable NMEA
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x06, 0x00, 0x01, 0x1E,                                   // CFG-NMEA disable all default NMEA messages - Now above wont matter. mr-fiero
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B,     // Disable NMEA GLL - Just makin sure
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32,     // Disable NMEA GSA - Just makin sure
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39,     // Disable NMEA GSV - Just makin sure
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40,     // Disable NMEA RMC - Just makin sure
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46,     // Disable NMEA VTG - Just makin sure
    // 
    // Ublox
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x00, 0x1D, 0x66,                                  // Disable ALL UBX MSG's first, clean house. also good indicator. anything wrong then no output!
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x01, 0x10, 0x4B,                                  // set DOP MSG
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,                                  // set POSLLH MSG
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,                                  // set STATUS MSG
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,                                  // set SOL MSG
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,                                  // set VELNED MSG
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3,                                  // set SVINFO MSG - Some people like to see this
    //
    // 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41, // set WAAS to EGNOS - For EU - original code, whats the point?
    // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,             // set rate to 5Hz
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xA8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xBE, 0xAA,                // set rate to 5.95Hz - Ublox 6 engines MAX NAV rate
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x02, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x89, 0x39,                                     // disable SBAS
    // Now for NAV5 Settings and Tweaks for better PH in Pedistrian mode
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x04, 0x00, 0xC8, 0x00, 0xC8, 0x00, 0x50, 0x00, 0xC8, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x80,
    0xB5, 0x62, 0x06, 0x23, 0x28, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x03, 0x02, 0x03, 0x10, 0x0A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x43, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x64, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBA, 0x1A,
    //
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x00, 0x19, 0x81,                                                                       // set Max performance mode - Just incase.
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x07, 0x9F, 0xA2,     //  I think this would save config to eeprom...not sure
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76,                                                           // Hotstart GPS - All done, restarting GPS
};

static const uint8_t svinfo[] =
{
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3                // set SVINFO MSG rate
};

// UBX SPECIFIC DATASETS
typedef struct
{
    uint32_t time;                                                                  // GPS msToW
    int32_t  longitude;
    int32_t  latitude;
    int32_t  altitude_ellipsoid;
    int32_t  altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct
{
    uint32_t time;                                                                  // GPS msToW
    uint8_t  fix_type;
    uint8_t  fix_status;
    uint8_t  differential_status;
    uint8_t  res;
    uint32_t time_to_first_fix;
    uint32_t uptime;                                                                // milliseconds
} ubx_nav_status;

typedef struct
{
    uint32_t time;
    int32_t  time_nsec;
    int16_t  week;
    uint8_t  fix_type;
    uint8_t  fix_status;
    int32_t  ecef_x;
    int32_t  ecef_y;
    int32_t  ecef_z;
    uint32_t position_accuracy_3d;
    int32_t  ecef_x_velocity;
    int32_t  ecef_y_velocity;
    int32_t  ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t  res;
    uint8_t  satellites;
    uint32_t res2;
} ubx_nav_solution;

typedef struct
{
    uint32_t time;                                                                  // GPS msToW
    int32_t  ned_north;
    int32_t  ned_east;
    int32_t  ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t  heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;

enum
{
    PREAMBLE1            = 0xb5,
    PREAMBLE2            = 0x62,
    CLASS_NAV            = 0x01,
    CLASS_ACK            = 0x05,
    CLASS_CFG            = 0x06,
    MSG_ACK_NACK         = 0x00,
    MSG_ACK_ACK          = 0x01,
    MSG_POSLLH           = 0x2,
    MSG_STATUS           = 0x3,
    MSG_SOL              = 0x6,
    MSG_VELNED           = 0x12,
    MSG_SVINFO           = 0x30,
    MSG_CFG_PRT          = 0x00,
    MSG_CFG_RATE         = 0x08,
    MSG_CFG_SET_RATE     = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} ubs_protocol_bytes;

enum
{
    FIX_NONE               = 0,
    FIX_DEAD_RECKONING     = 1,
    FIX_2D                 = 2,
    FIX_3D                 = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME               = 5
} ubs_nav_fix_type;

enum
{
    NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;


static volatile uint8_t GPS_Present = 0;

static void gpsPrint(const char *str);
static bool GPS_MTK_newFrame(uint8_t data);
static bool GPS_NMEA_newFrame(char c);
static bool GPS_UBLOX_newFrame(uint8_t data);
static bool GPS_newFrame(char c);
static void FiveElementSpikeFilterINT32(int32_t newval, int32_t *array);

void DoChkGPSDeadin50HzLoop(void)                                                   // Check this in a 50Hz loop in mainprogram
{
    static uint8_t  cnt = 0;
    static uint32_t LastGPSTS = 0;
    if (!LastGPSTS) LastGPSTS = TimestampNewGPSdata;                                // if TimestampNewGPSdata is 0 we will be here again soon
    else
    {
        if (TimestampNewGPSdata == LastGPSTS) cnt++;
        else
        {
            cnt = 0;
            LastGPSTS = TimestampNewGPSdata;
        }
        if (cnt == 75) sensorsClear(SENSOR_GPS);                                    // No Data for 1.5 secs?
    }
}

static void GPS_NewData(uint16_t c)                                                 // Called by uart2Init interrupt
{
    static int32_t  LatSpikeTab[5], LonSpikeTab[5];
    static bool     FilterCleared = false;
    uint8_t         i;

    if (GPS_newFrame(c))
    {
        if(!GPS_FIX)                                                                // Don't fill spikefilter with pure shit
        {
            if(!FilterCleared)
            {
                for (i = 0; i < 5; i++)
                {
                    LatSpikeTab[i] = 0;
                    LonSpikeTab[i] = 0;
                }
                FilterCleared = true;
            }
        }
        else                                                                        // We have a fix. Can and shall we use Spikefiltervalues?
        {
            FilterCleared = false;
            FiveElementSpikeFilterINT32(IRQGPS_coord[LAT], LatSpikeTab);            // Feed filter to have data when needed
            FiveElementSpikeFilterINT32(IRQGPS_coord[LON], LonSpikeTab);
            if (LatSpikeTab[2] && LonSpikeTab[2] && GPS_numSat < 6)                 // Filter when doubtful satnumber
            {
                IRQGPS_coord[LAT] = LatSpikeTab[2];
                IRQGPS_coord[LON] = LonSpikeTab[2];
            }
        }
        TimestampNewGPSdata = millis();                                             // Set timestamp of Data arrival in MS
    }
}

static bool GPS_newFrame(char c)                                                    // Crashpilot
{
    switch (cfg.gps_type) 	                                                        // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
    {
    case 0:                                                                         // NMEA
        return GPS_NMEA_newFrame(c);
    case 1:                                                                         // UBX
    case 4:
        return GPS_UBLOX_newFrame(c);
    case 2:                                                                         // Dealing with old, faulty and new, correct binary protocol
    case 3:
        return GPS_MTK_newFrame(c);                                                 // GPS_MTK_newFrame handles both 1.6 and 1.9 3drobotics nomenclature
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////////
// ***   GPS INIT   ***
////////////////////////////////////////////////////////////////////////////////////
void gpsInit(uint32_t baudrate)                                                     // Called in Main
{
    uint8_t i;
    uint32_t timeout;

    GPS_Present = 0;
    delay(2000);                                                                    // let it init
    timeout = millis() + 12000; 					                                          // 12 sec timeout
    while (!GPS_Present && millis() < timeout)                                      // Repeat while no GPS Data
    {
        uart2Init(baudrate, GPS_NewData, false);                                    // Set up Interrupthandler
        switch (cfg.gps_type)  	                                                    // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
        {
        case 0:                                                                     // GPS_NMEA
            break;
        case 1:                                                                     // GPS_UBLOX
            UbloxForceBaud(baudrate);
            for (i = 0; i < sizeof(ubloxInit); i++)
            {
                delay(7);
                uart2Write(ubloxInit[i]);                                           // send ubx init binary
            }
            break;
        case 2:                                                                     // GPS_MTK16
        case 3:                                                                     // GPS_MTK19
            for (i = 0; i < 5; i++)
            {
                uart2ChangeBaud(init_speed[i]);
                delay(200);
                gpsPrint(MTK_BAUD_RATE_57600);
            }
            uart2ChangeBaud(57600);
            delay(200);
            gpsPrint(MTK_SET_BINARY);
            delay(200);
            gpsPrint(MTK_OUTPUT_5HZ);
            delay(200);
            gpsPrint(MTK_SBAS_INTEGRITYMODE);
            delay(200);
            gpsPrint(MTK_NAVTHRES_OFF);
            delay(200);
            gpsPrint(MTK_SBAS_ON);
            delay(200);
            gpsPrint(MTK_WAAS_ON);
            break;
        case 4:                                                                     // GPS_UBLOX_DUMB = 4
            break;
        }
        delay(1000);
    }
    if (GPS_Present) sensorsSet(SENSOR_GPS);                                        // Do we get Data? Is GPS present?
}

void UblxSignalStrength(void)
{
    uint8_t i;
    for (i = 0; i < sizeof(svinfo); i++)
    {
        delay(7);
        uart2Write(svinfo[i]);
    }
}

void UbloxForceBaud(uint32_t baud)
{
    uint8_t i;
    for (i = 0; i < 5; i++)
    {
        delay(50);
        uart2ChangeBaud(init_speed[i]);
        delay(250);
        switch(baud)
        {
        case 19200:
            gpsPrint("$PUBX,41,1,0003,0001,19200,0*23\r\n");
            break;
        case 38400:
            gpsPrint("$PUBX,41,1,0003,0001,38400,0*26\r\n");
            break;
        case 57600:
            gpsPrint("$PUBX,41,1,0003,0001,57600,0*2D\r\n");
            break;
        case 115200:
            gpsPrint("$PUBX,41,1,0003,0001,115200,0*1E\r\n");
            break;
        }
    }
    uart2ChangeBaud(baud);
    delay(200);
}

static void gpsPrint(const char *str)
{
    while (*str)
    {
        if (cfg.gps_type == 1) delay(7);                                            // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3,
        uart2Write(*str);
        str++;
    }
    while (!uart2TransmitEmpty());                                                  // wait to send all
    delay(30);
}

static bool GPS_MTK_newFrame(uint8_t data)                                          // Crashpilot: This code is stupid but works
{
    static  uint8_t  pstep, lastbyte, LSLshifter,chkA, count, satn, fixtype;
    static  uint32_t lat, lon, alt, grspeed, grcourse;                              // MTK Dataset use unsigned for shiftoperation here
    int32_t tmp32     = 0;
    uint8_t startbyte = 0xd1;                                                       // Set default for 1.9 FW
    bool    parsed    = false;

    if(!pstep)
    {
        if (cfg.gps_type == 2) startbyte = 0xd0;                                    // 3drobotics 1.6 FW and clones have $d0 preamblebyte not d1
        if (data == 0xdd && lastbyte == startbyte) pstep = 100;                     // Detect Sync "0xD1,0xDD" Only search for Sync when not already decoding
    }
    lastbyte = data;
    switch(pstep)
    {
    case 0:                                                                         // Special Case: Do Nothing
        break;
    case 100:                                                                       // Special Case: Prepare next decoding run
        pstep = 1;                                                                  // Jump into decoding on next run
        chkA  = count = 0;
        break;
    case 1:                                                                         // Payload Byte is always $20! (This is the first Byte after sync preamble)
        if (data == 0x20) pstep++;                                                  // Since it is always $20 we take it as extended, undocumented syncword "preamble3"
        else pstep = 0;                                                             // Error! Wait for sync
        chkA += data;
        count++;
        break;
    case 2:                                                                         // Read Dataset Latitude
        lat        = data;
        chkA      += data;
        LSLshifter = 0;
        pstep++;
        count++;
        break;
    case 3:
        LSLshifter += 8;
        tmp32       = data;
        chkA       += data;
        lat        |= (tmp32 << LSLshifter);
        if (LSLshifter == 24) pstep++;
        count++;
        break;
    case 4:                                                                         // Read Dataset Longitude
        lon        = data;
        LSLshifter = 0;
        chkA      += data;
        pstep++;
        count++;
        break;
    case 5:
        LSLshifter += 8;
        tmp32       = data;
        chkA       += data;
        lon        |= (tmp32 << LSLshifter);
        if (LSLshifter == 24) pstep++;
        count++;
        break;
    case 6:                                                                         // Read Dataset MSL Altitude
        alt        = data;
        chkA      += data;
        LSLshifter = 0;
        pstep++;
        count ++;
        break;
    case 7:
        LSLshifter += 8;
        tmp32       = data;
        chkA       += data;
        alt        |= (tmp32 << LSLshifter);
        if (LSLshifter == 24)
        {
            alt = alt / 100;                                                        // GPS altitude in meter
            pstep++;
        }
        count++;
        break;
    case 8:                                                                         // Read Dataset Ground Speed
        grspeed    = data;
        chkA      += data;
        LSLshifter = 0;
        pstep++;
        count++;
        break;
    case 9:
        LSLshifter += 8;
        tmp32       = data;
        chkA       += data;
        grspeed    |= (tmp32 << LSLshifter);
        if (LSLshifter == 24) pstep++;
        count++;
        break;
    case 10:                                                                        // Read Dataset Heading
        grcourse   = data;
        chkA      += data;
        LSLshifter = 0;
        pstep++;
        count++;
        break;
    case 11:
        LSLshifter += 8;
        tmp32       = data;
        chkA       += data;
        grcourse   |= (tmp32 << LSLshifter);
        if (LSLshifter == 24) pstep++;
        count++;
        break;
    case 12:                                                                        // Read number of satellites in view
        satn  = data;
        chkA += data;
        pstep++;
        count++;
        break;
    case 13:                                                                        // Read Fix Type
        fixtype = data;                                                             // FIX_NONE = 1, FIX_2D = 2, FIX_3D = 3, FIX_2D_SBAS = 6, FIX_3D_SBAS = 7
        chkA   += data;
        pstep++;
        count++;
        break;
    case 14:                                                                        // Wait for cheksum A
        if (count == 33)                                                            // 33 = 0x21
        {
            if (chkA == data) pstep++;                                              // ChecksumA reached. Correct? than go on
            else pstep = 0;                                                         // Error?
        }
        else
        {
            chkA += data;
            count++;
        }
        break;
    case 15:                                                                        // Dataset RDY !! Cheksum B omitted, ChkA was OK
        if (fixtype > 1) GPS_FIX = true;
         else GPS_FIX = false;
        if (startbyte == 0xd0)                                                      // We are dealing with old binary protocol here (*10 Error LAT/LON)
        {
            lat *= 10;                                                              // so we have to multiply by 10 lat and lon
            lon *= 10;
        }
        IRQGPS_coord[LAT] = (int32_t)lat;
        IRQGPS_coord[LON] = (int32_t)lon;
        GPS_altitude      = alt;
        IRQGPS_speed      = grspeed;
        IRQGPS_grcrs      = grcourse / 10;                                          // /10 to get deg * 10 according docu
        GPS_numSat        = satn;
        GPS_Present       = 1;                                                      // Show GPS is working
        parsed            = true;                                                   // RDY
        pstep             = 0;                                                      // Do nothing / Scan for sync
        break;
    }
    return parsed;
}

#define FRAME_GGA  1
#define FRAME_RMC  2
#define DIGIT_TO_VAL(_x)    (_x - '0')                                              // This code is used for parsing NMEA data
static uint32_t GPS_coord_to_degrees(char* s)
{
    char *p, *q;
    uint32_t deg = 0, min = 0, frac_min = 0, i;
    for (p = s; isdigit((unsigned char)*p); p++);                                   // scan for decimal point or end of field
    q = s;
    while ((p - q) > 2)                                                             // convert degrees
    {
        if (deg) deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    while (p > q)                                                                   // convert minutes
    {
        if (min) min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    if (*p == '.')                                                                  // convert fractional minutes expect up to four digits, result is in ten-thousandths of a minute
    {
        q = p + 1;
        for (i = 0; i < 4; i++)
        {
            frac_min *= 10;
            if (isdigit((unsigned char)*q)) frac_min += DIGIT_TO_VAL(*q++);
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

static uint32_t grab_fields(char *src, uint8_t mult)                                // convert string to uint32
{
    uint8_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++)
    {
        if (src[i] == '.')
        {
            i++;
            if (!mult) break;
            else src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9') tmp += src[i] - '0';
    }
    return tmp;
}

static uint8_t hex_c(uint8_t n)                                                     // convert '0'..'9','A'..'F' to 0..15
{
    n -= '0';
    if (n > 9) n -= 7;
    n &= 0x0F;
    return n;
}

static bool GPS_NMEA_newFrame(char c)
{
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, frame = 0;

    if (c == '$') param = offset = parity = 0;
    else if (c == ',' || c == '*')
    {
        string[offset] = 0;
        if (!param)                                                                 // frame identification
        {
            frame = 0;
            if (string[0] == 'G' && string[1] == 'P')
            {
                if (string[2] == 'G' && string[3] == 'G' && string[4] == 'A')      frame = FRAME_GGA;
                else if (string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
            }
        }
        else if (frame == FRAME_GGA)
        {
            if (param == 2)
            {
                IRQGPS_coord[LAT] = GPS_coord_to_degrees(string);
            }
            else if (param == 3 && string[0] == 'S') IRQGPS_coord[LAT] = -IRQGPS_coord[LAT];
            else if (param == 4)
            {
                IRQGPS_coord[LON] = GPS_coord_to_degrees(string);
            }
            else if (param == 5 && string[0] == 'W') IRQGPS_coord[LON] = -IRQGPS_coord[LON];
            else if (param == 6)
            {
                GPS_FIX = (string[0]  > '0');
            }
            else if (param == 7)
            {
                GPS_numSat = grab_fields(string, 0);
            }
            else if (param == 9)
            {
                GPS_altitude = grab_fields(string, 0);                              // altitude in meters added by Mis
            }
        }
        else if (frame == FRAME_RMC)
        {
            if (param == 7)
            {
                IRQGPS_speed = ((uint32_t)grab_fields(string, 1) * 5144L) / 1000L;  // gps speed in cm/s will be used for navigation
            }
            else if (param == 8)
            {
                IRQGPS_grcrs = grab_fields(string, 1);                              // ground course deg*10
            }
        }
        param++;
        offset = 0;
        if (c == '*') checksum_param = 1;
        else parity ^= c;
    }
    else if (c == '\r' || c == '\n')
    {
        if (checksum_param)                                                         // parity checksum
        {
            uint8_t checksum = hex_c(string[0]);
            checksum <<= 4;
            checksum += hex_c(string[1]);
            if (checksum == parity) frameOK = 1;
        }
        checksum_param = 0;
    }
    else
    {
        if (offset < 15) string[offset++] = c;
        if (!checksum_param) parity ^= c;
    }
    if (frame) GPS_Present = 1;
    return frameOK && (frame == FRAME_GGA);
}

static bool GPS_UBLOX_newFrame(uint8_t data)
{
#define Bufbytes 52                                                                 // Sizeof reports 52 Bytes no need to waste more
    static union                                                                    // UBLOX Receive buffer
    {
        ubx_nav_posllh   posllh;
        ubx_nav_status   status;
        ubx_nav_solution solution;
        ubx_nav_velned   velned;
        uint8_t          bytes[Bufbytes];
    } buffer;
    static uint16_t payloadlength, payloadcounter;
    static uint8_t  ck_a, ck_b, step = 0, UBXmsgid, UBXclass;
    static bool     newpos = false, newspd = false, nextfx = false;
    bool parsed = false;

reset:
    switch (step)
    {
    case 1:
        if (data == PREAMBLE2)
        {
            step++;
            break;
        }
        step = 0;
    case 0:
        if (data == PREAMBLE1) step++;
        payloadcounter = 0;
        break;
    case 2:                                                                         // Read Class
        step++;
        ck_b     = data;
        ck_a     = data;
        UBXclass = data;
        break;
    case 3:                                                                         // Read msgid
        step++;
        ck_b    += (ck_a += data);
        UBXmsgid = data;
        break;
    case 4:                                                                         // Read payload low byte
        step++;
        ck_b += (ck_a += data);
        payloadlength  = (uint16_t)data;
        break;
    case 5:                                                                         // Read payload high byte
        step++;
        ck_b += (ck_a += data);
        payloadlength |= (uint16_t)((uint16_t)data << 8);
        if (payloadlength > 512)
        {
            step = 0;
            goto reset;
        }
        break;
    case 6:
        ck_b += (ck_a += data);                                                     // checksum byte
        if (payloadcounter < Bufbytes) buffer.bytes[payloadcounter] = data;         // Copy over data until buffer full
        if (++payloadcounter == payloadlength) step++;                              // Read out everything offered and proceed
        break;
    case 7:
        step++;
        if (ck_a != data)
        {
            step = 0;
            goto reset;
        }
        break;
    case 8:                                                                         // Piece stuff together if chb is correct
        step = 0;
        if (ck_b != data || UBXclass != CLASS_NAV) break;                           // bad checksum or irrelevant for us
        switch (UBXmsgid)
        {
        case MSG_POSLLH:
            IRQGPS_coord[LAT] = buffer.posllh.latitude;
            IRQGPS_coord[LON] = buffer.posllh.longitude;
            GPS_altitude = buffer.posllh.altitude_msl / 1000;                       // alt in m we don't buffer GPS_altitude since it not of importance
            GPS_FIX = nextfx;
            newpos    = true;
            break;
        case MSG_STATUS:
            nextfx = (buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (buffer.status.fix_type == FIX_3D || buffer.status.fix_type == FIX_2D);
            if (!nextfx) GPS_FIX = false;
            break;
        case MSG_SOL:
            nextfx = (buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (buffer.solution.fix_type == FIX_3D || buffer.solution.fix_type == FIX_2D);
            if (!nextfx) GPS_FIX = false;
            GPS_numSat = buffer.solution.satellites;                                // GPS_hdop = _buffer.solution.position_DOP;
            break;
        case MSG_VELNED:
            IRQGPS_speed = buffer.velned.speed_2d;                                  // cm/s speed_3d = _buffer.velned.speed_3d;  // cm/s
            IRQGPS_grcrs = (uint16_t)(buffer.velned.heading_2d / 10000);            // Heading 2D deg * 100000 rescaled to deg * 10
            newspd       = true;
            break;
        default:
            break;
        }
        if (newpos && newspd)
        {
            newspd = false;
            newpos = false;
            parsed = true;
        }
        GPS_Present = 1;
    }
    return parsed;
}

// Also gets rid of "glitches" though ublox KF doesn't do that
static void FiveElementSpikeFilterINT32(int32_t newval, int32_t *array)
{
    uint8_t sortidx, maxsortidx = 4;
    int32_t extmp;
    bool    rdy = false;
    array[0] = newval;
    array[4] = newval;
    while(!rdy)
    {
        rdy = true;
        for (sortidx = 0; sortidx < maxsortidx; sortidx++)
        {
            extmp = array[sortidx];
            if (extmp > array[sortidx + 1])
            {
                array[sortidx]     = array[sortidx + 1];
                array[sortidx + 1] = extmp;
                rdy = false;
            }
        }
        maxsortidx --;
    }
}
