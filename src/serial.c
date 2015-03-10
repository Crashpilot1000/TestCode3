#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"

// Multiwii Serial Protocol 0
#define MSP_VERSION              0
#define PLATFORM_32BIT           0x80000000

#define MSP_IDENT                100    //out message         multitype + version
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         1 altitude
#define MSP_BAT                  110    //out message         vbat, powermetersum, RSSI
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113    //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114    //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203    //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_WP_SET               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUGMSG             253    //out message         debug string buffer
#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

#define INBUF_SIZE 64

static uint8_t  checksum, indRX, inBuf[INBUF_SIZE];
static uint8_t  cmdMSP;
static uint32_t LastValidProtocolTimestampMS;
bool   AllowProtocolAutosense;
uint8_t  Currentprotocol;               // 0=Dont Know 1=Mwii 2=Mavlink

// static bool guiConnected = false;
void serialize32(uint32_t a)
{
    static uint8_t t;
    t = a;
    uartWrite(t);
    checksum ^= t;
    t = a >> 8;
    uartWrite(t);
    checksum ^= t;
    t = a >> 16;
    uartWrite(t);
    checksum ^= t;
    t = a >> 24;
    uartWrite(t);
    checksum ^= t;
}

void serialize16(int16_t a)
{
    static uint8_t t;
    t = a;
    uartWrite(t);
    checksum ^= t;
    t = a >> 8 & 0xff;
    uartWrite(t);
    checksum ^= t;
}

void serialize8(uint8_t a)
{
    uartWrite(a);
    checksum ^= a;
}

uint8_t read8(void)
{
    return inBuf[indRX++] & 0xff;
}

uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t) read8() << 8;
    return t;
}

uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t) read16() << 16;
    return t;
}

void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(cmdMSP);
}

void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void tailSerialReply(void)
{
    serialize8(checksum);
}

void serializeNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(*c);
}

void serialInit(uint32_t baudrate)
{
    uartInit(baudrate);
}

static void evaluateCommand(void)
{
    uint32_t i, tmpu32 = 0;
    uint8_t wp_no;

    switch (cmdMSP)
    {
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++)
            rcDataSAVE[i] = read16();
        headSerialReply(0);
        break;
    case MSP_SET_RAW_GPS:
        GPS_FIX = read8();
        GPS_numSat = read8();
        Real_GPS_coord[LAT] = read32();
        Real_GPS_coord[LON] = read32();
        GPS_altitude = read16();
        GPS_speed = read16();
        GPS_update |= 2;                              // New data signalisation to GPS functions
        headSerialReply(0);
        break;
    case MSP_SET_PID:
        for (i = 0; i < PIDITEMS; i++)
        {
            cfg.P8[i] = read8();
            cfg.I8[i] = read8();
            cfg.D8[i] = read8();
        }
        cfg.P8[PIDROLL]  = max(cfg.P8[PIDROLL], 1);   // Prevent div by zero in pid controller
        cfg.P8[PIDPITCH] = max(cfg.P8[PIDPITCH], 1);  // Prevent div by zero in pid controller
        headSerialReply(0);
        break;
    case MSP_SET_BOX:
        for (i = 0; i < CHECKBOXITEMS; i++)
        {
            tmpu32 = 0;
            if (cfg.rc_auxch > 4) tmpu32 = read32();
            else tmpu32 = read16();
            cfg.activate[i] = tmpu32;
        }
        headSerialReply(0);
        break;
    case MSP_SET_RC_TUNING:
        cfg.rcRate8 = read8();
        cfg.rcExpo8 = read8();
        cfg.rollPitchRate = read8();
        cfg.yawRate = read8();
        cfg.dynThrPID = read8();
        cfg.thrMid8 = read8();
        cfg.thrExpo8 = read8();
        headSerialReply(0);
        break;
    case MSP_SET_MISC:
        headSerialReply(0);
        break;
    case MSP_IDENT:
        headSerialReply(7);
        serialize8(VERSION);                // multiwii version
        serialize8(cfg.mixerConfiguration); // type of multicopter
        serialize8(MSP_VERSION);            // MultiWii Serial Protocol Version
        serialize32(PLATFORM_32BIT);        // "capability"
        break;
    case MSP_STATUS:
        headSerialReply(11);
        serialize16((int16_t)AvgCyclTime);
        serialize16(i2cGetErrorCounter());
        serialize16(sensors(SENSOR_ACC)                   |
                    sensors(SENSOR_BARO)   << 1           |
                    sensors(SENSOR_MAG)    << 2           |
                    sensors(SENSOR_GPS)    << 3           |
                    sensors(SENSOR_SONAR)  << 4);

        serialize32(f.ANGLE_MODE           << BOXANGLE    |
                    f.HORIZON_MODE         << BOXHORIZON  |
                    f.BARO_MODE            << BOXBARO     |
                    f.MAG_MODE             << BOXMAG      |
                    f.ARMED                << BOXARM      |
                    rcOptions[BOXCAMSTAB]  << BOXCAMSTAB  |
                    f.GPS_HOME_MODE        << BOXGPSHOME  |
                    f.GPS_HOLD_MODE        << BOXGPSHOLD  |
                    f.GPS_AUTO_MODE        << BOXGPSAUTO  |
                    f.HEADFREE_MODE        << BOXHEADFREE |
                    f.PASSTHRU_MODE        << BOXPASSTHRU |
                    rcOptions[BOXBEEPERON] << BOXBEEPERON |
                    rcOptions[BOXHEADADJ]  << BOXHEADADJ  |
                    rcOptions[BOXOSD]      << BOXOSD      |
                    f.GTUNE                << BOXGTUNE);
        serialize8(0);
        break;
    case MSP_RAW_IMU:
        headSerialReply(18);
        if (!feature(FEATURE_PASS))                               // Just Do the normal stuff
        {
            for (i = 0; i < 3; i++) serialize16((int16_t)(((int32_t)accSmooth[i] * 512) / cfg.sens_1G));
            for (i = 0; i < 3; i++) serialize16(((int16_t)gyroData[i]) >> 2);
            for (i = 0; i < 3; i++) serialize16((int16_t)magADCfloat[i]);
        }
        else                                                      // Just serialize unfiltered AccZ for Balancing
        {
            for (i = 0; i < 2; i++) serialize16(0);
            serialize16((int16_t)((((int32_t)accADC[YAW] * 512) / cfg.sens_1G) - 512)); // Put accz into the middle
            for (i = 0; i < 6; i++) serialize16(0);
        }
        break;
    case MSP_SERVO:
        headSerialReply(16);
        for (i = 0; i < 8; i++) serialize16(servo[i]);
        break;
    case MSP_MOTOR:
        headSerialReply(16);
        for (i = 0; i < 8; i++) serialize16(motor[i]);
        break;
    case MSP_RC:
        headSerialReply((cfg.rc_auxch + 4) * 2);
        for (i = 0; i < cfg.rc_auxch + 4; i++) serialize16(rcDataSAVE[i]); // Put out raw values
        break;
    case MSP_RAW_GPS:
        headSerialReply(14);
        serialize8(GPS_FIX);
        serialize8(GPS_numSat);
        serialize32(Real_GPS_coord[LAT]);
        serialize32(Real_GPS_coord[LON]);
        serialize16(GPS_altitude);
        serialize16(GPS_speed);
        break;
    case MSP_COMP_GPS:
        headSerialReply(5);
        serialize16(GPS_distanceToHome);
        serialize16(GPS_directionToHome);
        serialize8(GPS_update & 1);
        break;
    case MSP_ATTITUDE:
        headSerialReply(8);
        for (i = 0; i < 2; i++) serialize16((int16_t)angle[i]);
        serialize16((int16_t)heading);
        serialize16((int16_t)headFreeModeHold);
        break;
    case MSP_ALTITUDE:
        headSerialReply(6);
        if (feature(FEATURE_PASS))
        {
            serialize32(0);
            serialize16(0);
        }
        else
        {
            serialize32((int32_t)EstAlt);
            serialize16((int16_t)vario);
        }
        break;
    case MSP_BAT:
        headSerialReply(5);
        serialize8(vbat);
        serialize16(0);                               // power meter stuff
        serialize16((uint16_t)rssi << 2);             // Upscale 255 to 1020, so 100% (1023) is hard to achieve, who cares?
        break;
    case MSP_RC_TUNING:
        headSerialReply(7);
        serialize8(cfg.rcRate8);
        serialize8(cfg.rcExpo8);
        serialize8(cfg.rollPitchRate);
        serialize8(cfg.yawRate);
        serialize8(cfg.dynThrPID);
        serialize8(cfg.thrMid8);
        serialize8(cfg.thrExpo8);
        break;
    case MSP_PID:
        headSerialReply(3 * PIDITEMS);
        for (i = 0; i < PIDITEMS; i++)
        {
            serialize8(cfg.P8[i]);
            serialize8(cfg.I8[i]);
            serialize8(cfg.D8[i]);
        }
        break;
    case MSP_BOX:
        if (cfg.rc_auxch > 4) headSerialReply(4 * CHECKBOXITEMS);
        else headSerialReply(2 * CHECKBOXITEMS);
        for (i = 0; i < CHECKBOXITEMS; i++)
        {
            tmpu32 = cfg.activate[i];
            if (cfg.rc_auxch > 4) serialize32(tmpu32);
            else serialize16((int16_t)tmpu32);
        }
        break;
    case MSP_BOXNAMES:
        headSerialReply(sizeof(boxnames) - 1);
        serializeNames(boxnames);
        break;
    case MSP_PIDNAMES:
        headSerialReply(sizeof(pidnames) - 1);
        serializeNames(pidnames);
        break;
    case MSP_MISC:
        headSerialReply(2);
        serialize16(0); // intPowerTrigger1
        break;
    case MSP_MOTOR_PINS:
        headSerialReply(8);
        for (i = 0; i < 8; i++) serialize8(i + 1);
        break;
    case MSP_WP:
        wp_no = read8();    // get the wp number
        headSerialReply(12);
        if (wp_no == 0)
        {
            serialize8(0);                   // wp0
            serialize32(GPS_home[LAT]);
            serialize32(GPS_home[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        }
        else if (wp_no == 16)
        {
            serialize8(16);                  // wp16
            serialize32(GPS_WP[LAT]);
            serialize32(GPS_WP[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        }
        break;
    case MSP_RESET_CONF:
        checkFirstTime(true);
        headSerialReply(0);
        break;
    case MSP_ACC_CALIBRATION:
        calibratingA = true;
        headSerialReply(0);
        break;
    case MSP_MAG_CALIBRATION:
        calibratingM = true;
        headSerialReply(0);
        break;
    case MSP_EEPROM_WRITE:
        writeParams(0);
        headSerialReply(0);
        break;
    case MSP_DEBUG:
        headSerialReply(8);
        for (i = 0; i < 4; i++) serialize16(debug[i]);      // 4 variables are here for general monitoring purpose
        break;
    default:                                                // we do not know how to handle the (valid) message, indicate error MSP $M!
        headSerialError(0);
        break;
    }
    tailSerialReply();
}

bool mwii_receive(char c)
{
    static uint8_t offset, dataSize, c_state = 0;

    switch(c_state)
    {
    case 0:
        if (c == '$') c_state++;
        else c_state = 0;
        break;
    case 1:
        if (c == 'M') c_state++;
        else c_state = 0;
        break;
    case 2:
        if (c == '<') c_state++;
        else c_state = 0;
        break;
    case 3:
        if (c > INBUF_SIZE) c_state = 0;
        else c_state++;
        dataSize  = c;
        offset    = checksum = indRX = 0;
        checksum ^= c;
        break;
    case 4:
        cmdMSP    = c;
        checksum ^= c;
        c_state++;
        break;
    case 5:
        if (offset < dataSize)
        {
            checksum       ^= c;
            inBuf[offset++] = c;
        }
        else
        {
            c_state = 0;
            if (checksum == c)          // compare calculated and transferred checksum
            {
                evaluateCommand();      // we got a valid packet, evaluate it
                return true;
            }
        }
        break;
    }
    return false;
}

void serialCom(bool singlestep)                                                     // Singlestep means you can restrict to decoding just one dataset per run
{                                                                                   // Reason: Reduce priority when other tasks need more.
    uint8_t c;
    bool HaveSomePacket = false, Skipnow = false;
    static uint8_t NumberSignCNT = 0, RcharCNT = 0, RetCNT = 0, SingleStepCnt;

    if (singlestep) SingleStepCnt = min(SingleStepCnt+1, 4);
    else SingleStepCnt = 0;
    if (SingleStepCnt > 3) singlestep = false;                                      // Limit singlestep to max 3. in a row
// START Common Stuff for serial protocols
    if (cfg.rssicut && rssi <= cfg.rssicut) rssi = 0;
// END Common Stuff for serial protocols
    if (f.ARMED)                                                                    // Change Protocol according to armed state
    {
        switch(cfg.tele_prot)                                                       // 0=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)
        {
        case 0:
            Currentprotocol = PROTOCOL_MWII21;
            break;
        case 1:                                                                     // Do Frsky here
            sendFRSKYTelemetry();
            break;
        case 2:
        case 3:
            Currentprotocol = PROTOCOL_MAVLINK;
            break;
        }
    }
    else                                                                            // Not armed engage autosensing here
    {
        if (AllowProtocolAutosense && ((currentTimeMS - LastValidProtocolTimestampMS) > 4000))
        Currentprotocol = PROTOCOL_AUTOSENSE;                                       // Set Protocol to unknown after 4 Sek of garbage or no inputdata
    }

    if((Currentprotocol == PROTOCOL_MAVLINK) || (Currentprotocol == PROTOCOL_AUTOSENSE))
    {
        baseflight_mavlink_send_updates();                                          // It will Heartbeat and mavlink around some data
    }

    while (uartAvailable() && !Skipnow)
    {
        c = uartRead();
        if (Currentprotocol == PROTOCOL_AUTOSENSE)                                  // Check for extra Data when no protocol active (that is only in disarmed state possible)
        {
            if (c == '#')
            {
                NumberSignCNT++;
                if (NumberSignCNT > 2) cliProcess();                                // This is a one way - reset - street
            }else NumberSignCNT = 0;

            if (c == 'R')
            {
                RcharCNT++;
                if (RcharCNT > 2) systemReset(true);                                // This is a one way - reset - street
            }else RcharCNT = 0;
        }
        if (!f.ARMED && c == '\r')                                                  // Check for 3 Times Return in a row
        {
            RetCNT++;
            if (RetCNT > 2) cliProcess();
        }else RetCNT = 0;

        switch(Currentprotocol)
        {
        case PROTOCOL_AUTOSENSE:                                                    // Current is Unsure
            if(baseflight_mavlink_receive(c))Currentprotocol = PROTOCOL_MAVLINK;
            else if(mwii_receive(c)) Currentprotocol = PROTOCOL_MWII21;
            if(Currentprotocol != PROTOCOL_AUTOSENSE) HaveSomePacket = true;        // Some protocol found
            break;
        case PROTOCOL_MWII21:                                                       // Current is Mwii
            HaveSomePacket = mwii_receive(c);
            break;
        case PROTOCOL_MAVLINK:                                                      // Current is Mavlink
            HaveSomePacket = baseflight_mavlink_receive(c);
            break;
        }
        if(HaveSomePacket)
        {
            LastValidProtocolTimestampMS = currentTimeMS;
            if (singlestep) Skipnow = true;
        }
    }
}
