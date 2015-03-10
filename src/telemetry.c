/*
 * FrSky Telemetry implementation by silpstream @ rcgroups
 * GPS modification Crashpilot
 */
#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"

#define CYCLETIME             125

#define PROTOCOL_HEADER       0x5E
#define PROTOCOL_TAIL         0x5E

// Data Ids  (bp = before decimal point; af = after decimal point)
// Official data IDs
#define ID_GPS_ALTIDUTE_BP    0x01
#define ID_GPS_ALTIDUTE_AP    0x09
#define ID_TEMPRATURE1        0x02
#define ID_RPM                0x03
#define ID_FUEL_LEVEL         0x04
#define ID_TEMPRATURE2        0x05
#define ID_VOLT               0x06
#define ID_ALTITUDE_BP        0x10
#define ID_ALTITUDE_AP        0x21
#define ID_GPS_SPEED_BP       0x11
#define ID_GPS_SPEED_AP       0x19
#define ID_LONGITUDE_BP       0x12
#define ID_LONGITUDE_AP       0x1A
#define ID_E_W                0x22
#define ID_LATITUDE_BP        0x13
#define ID_LATITUDE_AP        0x1B
#define ID_N_S                0x23
#define ID_COURSE_BP          0x14
#define ID_COURSE_AP          0x1C
#define ID_DATE_MONTH         0x15
#define ID_YEAR               0x16
#define ID_HOUR_MINUTE        0x17
#define ID_SECOND             0x18
#define ID_ACC_X              0x24
#define ID_ACC_Y              0x25
#define ID_ACC_Z              0x26
#define ID_VOLTAGE_AMP_BP     0x3A
#define ID_VOLTAGE_AMP_AP     0x3B
#define ID_CURRENT            0x28
// User defined data IDs
#define ID_GYRO_X             0x40
#define ID_GYRO_Y             0x41
#define ID_GYRO_Z             0x42

// from sensors.c
extern uint8_t batteryCellCount;
static bool telemetryEnabled = false;

static void sendDataHead(uint8_t id)
{
    uartWrite(PROTOCOL_HEADER);
    uartWrite(id);
}

static void sendTelemetryTail(void)
{
    uartWrite(PROTOCOL_TAIL);
}

static void serializeFrsky(uint8_t data)            // take care of byte stuffing
{
    if (data == 0x5e)
    {
        uartWrite(0x5d);
        uartWrite(0x3e);
    }
    else if (data == 0x5d)
    {
        uartWrite(0x5d);
        uartWrite(0x3d);
    }
    else uartWrite(data);
}

static void serialize16(int16_t a)
{
    uint8_t t;
    t = a;
    serializeFrsky(t);
    t = a >> 8 & 0xff;
    serializeFrsky(t);
}

static void sendAccel(void)
{
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        sendDataHead(ID_ACC_X + i);
        serialize16((int16_t)(((int32_t)accSmooth[i] * 1000) / cfg.sens_1G));
    }
}

static void sendBaro(void)
{
    int32_t tmp = max((int32_t)EstAlt, 0);
    sendDataHead(ID_ALTITUDE_BP);
    serialize16(tmp / 100);
    sendDataHead(ID_ALTITUDE_AP);
    serialize16(tmp % 100);
}

static void sendTemperature1(void)
{
    sendDataHead(ID_TEMPRATURE1);
    serialize16((int16_t)telemTemperature1 / 10);
}

static void sendTime(void)
{
    uint32_t seconds = currentTimeMS / 1000;    
    uint8_t minutes = (seconds / 60) % 60;
    sendDataHead(ID_HOUR_MINUTE);                   // if we fly for more than an hour, something's wrong anyway
    serialize16(minutes << 8);
    sendDataHead(ID_SECOND);
    serialize16(seconds % 60);
}

// Mwii: dddddddddd
// Frsky pdf: dddmm.mmmm
static void GPStoDDDMM_MMMM(int32_t mwiigps, int16_t *dddmm, int16_t *mmmm)
{
    int32_t absgps, deg, min;
    if(!mwiigps)
    {
        *dddmm = 0;
        *mmmm  = 0;
    }
    else
    {
        absgps = abs(mwiigps);
        deg    = absgps / 10000000;
        absgps = (absgps - deg * 10000000) * 60;    // absgps = Minutes left * 10^7
        min    = absgps / 10000000;                 // minutes left
        *dddmm = deg * 100 + min;
        *mmmm  = (absgps - min * 10000000) / 1000;
    }
}

static void sendGPS(void)
{
    int16_t tmp1, tmp2;
    GPStoDDDMM_MMMM(Real_GPS_coord[LAT], &tmp1, &tmp2);
    sendDataHead(ID_LATITUDE_BP);
    serialize16(tmp1);
    sendDataHead(ID_LATITUDE_AP);
    serialize16(tmp2);
    sendDataHead(ID_N_S);
    serialize16(Real_GPS_coord[LAT] < 0 ? 'S' : 'N');

    GPStoDDDMM_MMMM(Real_GPS_coord[LON], &tmp1, &tmp2);
    sendDataHead(ID_LONGITUDE_BP);
    serialize16(tmp1);
    sendDataHead(ID_LONGITUDE_AP);
    serialize16(tmp2);
    sendDataHead(ID_E_W);
    serialize16(Real_GPS_coord[LON] < 0 ? 'W' : 'E');
}

/*
 * Send voltage via ID_VOLT
 *
 * NOTE: This sends voltage divided by batteryCellCount. To get the real
 * battery voltage, you need to multiply the value by batteryCellCount.
 * Note: Fuck the pdf. Format for Voltage Data for single cells is like this:
 *  llll llll cccc hhhh
 *  l: Low voltage bits
 *  h: High voltage bits
 *  c: Cell number (starting at 0)
 */
static void sendVoltage(void)
{
    static uint16_t currentCell = 0;
    uint16_t cellNumber;
    uint32_t cellVoltage;
    uint16_t payload;
 
    cellVoltage = vbat / batteryCellCount;
    cellVoltage = (cellVoltage * 2100) / 42;        // Map to 12 bit range
    cellNumber  = currentCell % batteryCellCount;
    payload     = (cellNumber << 4);                // Cell number is at bit 9-12
    payload    |= ((cellVoltage & 0x0ff) << 8);     // Lower voltage bits are at bit 0-8
    payload    |= ((cellVoltage & 0xf00) >> 8);     // Higher voltage bits are at bits 13-15
    sendDataHead(ID_VOLT);
    serialize16(payload);
    currentCell++;
    currentCell %= batteryCellCount;
}

static void sendVoltageAmp()                        // Send voltage with ID_VOLTAGE_AMP
{
    uint16_t voltage = (vbat * 110) / 21;
    sendDataHead(ID_VOLTAGE_AMP_BP);
    serialize16(voltage / 100);
    sendDataHead(ID_VOLTAGE_AMP_AP);
    serialize16(((voltage % 100) + 5) / 10);
}

static void sendHeading(void)
{
    int16_t tmp1 = heading;
    if (tmp1 < 0) tmp1 += 360;                      // heading in degrees, in compass units (0..360, 0=north)
    sendDataHead(ID_COURSE_BP);
    serialize16(tmp1);
    sendDataHead(ID_COURSE_AP);
    serialize16(0);
}

void initFRSKYTelemetry(bool State)
{
    if (State != telemetryEnabled)
    {
        if (State) serialInit(9600);
        else serialInit(cfg.serial_baudrate);
        telemetryEnabled = State;
        AllowProtocolAutosense = true;
        reset_mavlink();
    }
}

void initMinimOSDTelemetry(bool State)
{
    if (State != telemetryEnabled)
    {
        if (State) serialInit(57600);
        else serialInit(cfg.serial_baudrate);
        telemetryEnabled = State;
        AllowProtocolAutosense = true;
        reset_mavlink();
    }
}

void sendFRSKYTelemetry(void)
{
    static uint32_t lastCycleTime = 0;
    static uint8_t  cycleNum = 0;

    if (currentTimeMS - lastCycleTime >= CYCLETIME)
    {
        lastCycleTime = currentTimeMS;
        cycleNum++;

        sendAccel();                                // Sent every 125ms
        sendTelemetryTail();

        if (!(cycleNum % 4))                        // Sent every 500ms
        {
            sendBaro();
            sendHeading();
            sendTelemetryTail();
        }

        if (!(cycleNum % 8))                        // Sent every 1s
        {
            sendTemperature1();
            if (feature(FEATURE_VBAT))
            {
                sendVoltage();
                sendVoltageAmp();
            }
            if (sensors(SENSOR_GPS)) sendGPS();
            sendTelemetryTail();
        }

        if (cycleNum == 40)                         //Frame 3: Sent every 5s
        {
            cycleNum = 0;
            sendTime();
            sendTelemetryTail();
        }
    }
}
