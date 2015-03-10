#include "board.h"
#include "mw.h"

bool   calibratingA = false;                                      // ACC cal
bool   calibratingM = false;                                      // Magnetometer

float  MainDptCut;
bool   MpuSpecial;
extern uint16_t batteryWarningVoltage;
extern uint8_t  batteryCellCount;
float  magCal[3];                                                 // Gain for each axis, populated at sensor init
float  gyroZero[3];                                               // Populated upon initialization
bool   havel3g4200d = false;

sensor_t acc;                                                     // acc access functions
sensor_t gyro;                                                    // gyro access functions
baro_t   baro;                                                    // barometer access functions
uint8_t  accHardware = ACC_DEFAULT;                               // which accel chip is used/detected
static   bool  GyroCalCompromised = false;                        // Normal Gyro calibration could not be done, so wacky or presaved offsets are used
static   float GyroScale16;

static void GYRO_Common(void);
static void Mag_Calibration(void);
static void ACC_getRawRot(void);
static void Gyro_getRawRot(void);
static void Gyro_Calibrate(void);

void SensorDetectAndINI(void)                                     // "enabledSensors" is "0" in config.c so all sensors disabled per default
{
    int16_t deg, min;
    uint8_t sig          = 0;
    bool    ack          = false;
    bool    haveMpu6k    = false;

    GyroScale16 = (16.0f / 16.4f) * RADX;                         // GYRO part. RAD per SECOND, take into account that gyrodata are div by X
    if (mpu6050Detect(&acc, &gyro))                               // Autodetect gyro hardware. We have MPU3050 or MPU6050.
    {
        haveMpu6k = true;                                         // this filled up  acc.* struct with init values
    }
    else if (l3g4200dDetect(&gyro))
    {
        havel3g4200d = true;
        GyroScale16 = (16.0f / 14.2857f) * RADX;                  // GYRO part. RAD per SECOND, take into account that gyrodata are div by X
    }
    else if (!mpu3050Detect(&gyro))
    {
        failureMode(3);                                           // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
    }

    sensorsSet(SENSOR_ACC);                                       // ACC part. Will be cleared if not available
retry:
    switch (cfg.acc_hdw)
    {
    case 0:                                                       // autodetect
    case 1:                                                       // ADXL345
        if (adxl345Detect(&acc)) accHardware = ACC_ADXL345;
        if (cfg.acc_hdw == ACC_ADXL345) break;
    case 2:                                                       // MPU6050
        if (haveMpu6k)
        {
            mpu6050Detect(&acc, &gyro);                           // yes, i'm rerunning it again.  re-fill acc struct
            accHardware = ACC_MPU6050;
            if (cfg.acc_hdw == ACC_MPU6050) break;
        }
    case 3:                                                       // MMA8452
        if (mma8452Detect(&acc))
        {
            accHardware = ACC_MMA8452;
            if (cfg.acc_hdw == ACC_MMA8452) break;
        }
    }

    if (accHardware == ACC_DEFAULT)                               // Found anything? Check if user fucked up or ACC is really missing.
    {
        if (cfg.acc_hdw > ACC_DEFAULT)
        {
            cfg.acc_hdw = ACC_DEFAULT;                            // Nothing was found and we have a forced sensor type. User probably chose a sensor that isn't present.
            goto retry;
        }
        else sensorsClear(SENSOR_ACC);                            // We're really screwed
    }

    if (sensors(SENSOR_ACC)) acc.init();
    if (haveMpu6k && accHardware == ACC_MPU6050) MpuSpecial = true;
    else MpuSpecial = false;

    if (feature(FEATURE_PASS)) return;                            // Stop here we need just ACC for Vibrationmonitoring if present
    if (feature(FEATURE_GPS) && !SerialRCRX) gpsInit(cfg.gps_baudrate);// SerialRX and GPS can not coexist.
    gyro.init();                                                  // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    if (havel3g4200d) l3g4200dConfig();
    else if (!haveMpu6k) mpu3050Config();
    Gyro_Calibrate();                                             // Do Gyrocalibration here (is blocking), provides nice Warmuptime for the following rest!
#ifdef MAG
    if (hmc5883lDetect())
    {
        sensorsSet(SENSOR_MAG);
        hmc5883lInit(magCal);                                     // Crashpilot: Calculate Gains / Scale
        deg = cfg.mag_dec / 100;                                  // calculate magnetic declination
        min = cfg.mag_dec % 100;
        magneticDeclination = ((float)deg + ((float)min / 60.0f));// heading is in decimaldeg units NO 0.1 deg shit here
    }
#endif
#ifdef BARO                                                       // No delay necessary since Gyrocal blocked a lot already
    ack = i2cRead(0x77, 0x77, 1, &sig);                           // Check Baroadr.(MS & BMP) BMP will say hello here, MS not
    if ( ack) ack = bmp085Detect(&baro);                          // Are we really dealing with BMP?
    if (!ack) ack = ms5611Detect(&baro);                          // No, Check for MS Baro
    if (ack) sensorsSet(SENSOR_BARO);
    if(cfg.esc_nfly) ESCnoFlyThrottle = constrain_int(cfg.esc_nfly, cfg.esc_min, cfg.esc_max); // Set the ESC PWM signal threshold for not flyable RPM
    else ESCnoFlyThrottle = cfg.esc_min + (((cfg.esc_max - cfg.esc_min) * 5) / 100); // If not configured, take 5% above esc_min
#endif
#ifdef SONAR
    if (feature(FEATURE_SONAR)) Sonar_init();                     // Initialize Sonars here depending on Rc configuration.
    SonarLandWanted = cfg.snr_land;                               // Variable may be overwritten by failsave
#endif
    MainDptCut = RCconstPI / (float)cfg.maincuthz;                // Initialize Cut off frequencies for mainpid D
}

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 4095) * cfg.vbatscale;
}

void batteryInit(void)
{
    uint32_t i;
    uint32_t voltage = 0;
    for (i = 0; i < 32; i++)                                      // average up some voltage readings
    {
        voltage += adcGetChannel(ADC_BATTERY);
        delay(10);
    }
    voltage = batteryAdcToVoltage((uint16_t)(voltage / 32));
    for (i = 2; i < 6; i++)                                       // autodetect cell count, going from 2S..6S
    {
        if (voltage < i * cfg.vbatmaxcellvoltage) break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * cfg.vbatmincellvoltage;           // 3.3V per cell minimum, configurable in CLI
}

static void alignBoardyaw(float *rpy)
{
    float tmp;
    switch(cfg.align_board_yaw)                                   // 0 = 0 Deg. 1 = 90 Deg. 2 = 180 Deg. 3 = 270 Deg Clockwise
    {
    case 0:
        break;
    case 1:                                                       // 1 = 90 Deg CW
        tmp        =  rpy[ROLL];
        rpy[ROLL]  =  rpy[PITCH];
        rpy[PITCH] = -tmp;
        break;
    case 2:                                                       // 2 = 180 Deg CW
        rpy[ROLL]  = -rpy[ROLL];
        rpy[PITCH] = -rpy[PITCH];
        break;
    case 3:                                                       // 3 = 270 Deg CW
        tmp        =  rpy[ROLL];
        rpy[ROLL]  = -rpy[PITCH];
        rpy[PITCH] =  tmp;
        break;
    }
}

// ALIGN_GYRO = 0,
// ALIGN_ACCEL = 1,
// ALIGN_MAG = 2
void alignSensors(uint8_t type, int16_t *data)
{
    uint8_t i;
    int16_t tmp[3], axis;
    for (i = 0; i < 3; i++) tmp[i] = data[i];                     // make a copy
    for (i = 0; i < 3; i++)
    {
        axis = cfg.align[type][i];
        if (axis > 0)data[axis - 1] = tmp[i];
        else data[-axis - 1] = -tmp[i];
    }
}

static void Gyro_500Hz_AVG(float *xyztmp, uint16_t count)
{
    uint16_t i, axis;
    for (axis = 0; axis < 3; axis++) xyztmp[axis] = 0.0f;
    for (i = 0; i < count; i++)
    {
        delay(2);
        Gyro_getRawRot();
        for (axis = 0; axis < 3; axis++) xyztmp[axis] += gyroADC[axis] / (float)count;
    }
}

#define Gyromaxcount   300                                        // Processed Samples = Gyromaxcount * Gyroavgcount
#define Gyrodiscardcnt 100
#define Gyroavgcount     5
#define Timeoutrun       5                                        // One Run takes 2 seconds with 1000 samples @ 500Hz
static void Gyro_Calibrate(void)                                  // Total Samples = Gyromaxcount * Gyroavgcount + Gyrodiscardcnt
{
    float    Temp[3];
    uint16_t i, axis;
    uint8_t  breakout = 0;
    stdev_t  var[3];

    Gyro_500Hz_AVG(Temp, Gyrodiscardcnt);                         // Discard some values here to let gyro settle
    do                                                            // Shaky Hands Loop NOTE: Removed Sphere stuff here because results are equal!
    {
        breakout++;                                               // Increase Breakout Counter
        for (axis = 0; axis < 3; axis++)                          // Clear for Run
        {
            devClear(&var[axis]);
            gyroZero[axis] = 0.0f;
        }
        for (i = 0; i < Gyromaxcount; i++)                        // Outer loop for StdDev
        {
            Gyro_500Hz_AVG(Temp, Gyroavgcount);                   // Average some
            for (axis = 0; axis < 3; axis++)                      // Add StdDev, save values for next loopturn
            {
                devPush(&var[axis], Temp[axis]);
                gyroZero[axis] += Temp[axis];
            }
        }
        Temp[0] = 0.0f;
        for (axis = 0; axis < 3; axis++)
        {
            gyroZero[axis] /= (float)Gyromaxcount;                // Calculate gyrozero no matter what. Not timecritical anyway
            Temp[0] += devStandardDeviation(&var[axis]);
        }
    }
    while (Temp[0] > cfg.gy_stdev && breakout < Timeoutrun);      // Breakout prevents endlessloop after time
    GyroCalCompromised = (breakout == Timeoutrun);                // We timed out so gyro is problematic
    if(GyroCalCompromised && cfg.ShakyDataAvail)                  // Problem? Use backupdata, if available (normally they are..)
    {
        for (axis = 0; axis < 3; axis++) gyroZero[axis] = cfg.ShakyGyroZero[axis];
        blinkLED(15, 20, 10);                                     // Warnblink Flight may be degraded but possible.
    }
}

// Note: That mwii div by 4 stuff (done there on gyro readout) isn't bad at all to get rid of noise / jitter the easy way.
// The idea is preserved but accuracy is improved without introducing jitter.
// gyroData is seen by main pid controller. gyroADC is used in IMU for vector-rotation.
static void GYRO_Common(void)
{
    int16_t i, tmp;                                               // uint8 for "i" is shorter on 8Bit
    for (i = 0; i < 3; i++)
    {
        gyroADC[i]  = gyroADC[i] - gyroZero[i];
        tmp         = (int16_t)(gyroADC[i] * 0.3125f);
        gyroData[i] = (float)tmp * 3.2f;
        gyroADC[i] *= GyroScale16;                                // gyroADC delivered in 16 * rad/s
    }
    alignBoardyaw(gyroData);
    alignBoardyaw(gyroADC);
}

static void Acc_500Hz_AVG(float *xyztmp, uint16_t count)          // Do just 500 HZ - not all ACC do 1KHZ
{
    uint16_t i, axis;
    for (axis = 0; axis < 3; axis++) xyztmp[axis] = 0.0f;
    for (i = 0; i < count; i++)
    {
        delay(2);
        ACC_getRawRot();
        for (axis = 0; axis < 3; axis++) xyztmp[axis] += accADC[axis] / (float)count;
    }
}

#define ACCavgcount   3000
#define ACCdiscardcnt  100
static void Acc_Calibrate(void)                                   // Removed Sphere Algo, wasn't really better, sorry.
{
    uint16_t i;
    Acc_500Hz_AVG(cfg.accZero, ACCdiscardcnt);                    // Discard some values for warmup abuse accZero[3]
    Acc_500Hz_AVG(cfg.accZero, ACCavgcount);
    cfg.sens_1G = 16384;                                          // preset 2^14 that is the 16G Scale of MPU
    for (i = 0; i < 9; i++)                                       // Eval Bitresolution of ACC. BitScale is recognized here
    {
        if((float)abs((int16_t)cfg.accZero[2]) > ((float)cfg.sens_1G * 0.85f)) break;
        else cfg.sens_1G >>= 1;
    }
    cfg.accZero[2] -= cfg.sens_1G;
    for (i = 0; i < 3; i++) cfg.ShakyGyroZero[i] = gyroZero[i];
    for (i = 0; i < 2; i++) cfg.angleTrim[i] = 0.0f;
    cfg.ShakyDataAvail = 1;
    cfg.acc_calibrated = 1;
    writeParams(1);
    systemReset(false);
}

static void ACC_Common(void)
{
    uint8_t axis;
    if (calibratingA)
    {
        cfg.acc_calibrated = 0;                                   // Mark ACC not calibrated
        Gyro_Calibrate();                                         // Recal Gyro as well to have fresh data (warm chip) to save along acc offsets
        if (!GyroCalCompromised) Acc_Calibrate();                 // Proceed if copter is still. Dead end (Freeze, Calibrate, Save, Reset)
        else calibratingA = false;                                // Shaky copter during ACC Calibration is not tolerated
    }
    else
    {
        for (axis = 0; axis < 3; axis++)
        {
            if(cfg.acc_calibrated) accADC[axis] -= cfg.accZero[axis];
            else accADC[axis] = 1.0f;
        }
        alignBoardyaw(accADC);
    }
}

void GETMPU6050(void)
{
    int16_t accADC16[3];  
    int16_t gyroADC16[3];
    uint8_t i;
    MPU6050ReadAllShit(accADC16, &telemTemperature1, gyroADC16);
    if (cfg.align[ALIGN_ACCEL][0]) alignSensors(ALIGN_ACCEL, accADC16); else acc.align(accADC16);
    if (cfg.align[ALIGN_GYRO][0])  alignSensors(ALIGN_GYRO, gyroADC16); else gyro.align(gyroADC16);  
    for (i = 0; i < 3; i++)
    {
        accADC[i]  = (float)accADC16[i];
        gyroADC[i] = (float)gyroADC16[i];
    }
    ACC_Common();
    GYRO_Common();
}

static void ACC_getRawRot(void)
{
    int16_t accADC16[3], i;
    acc.read(accADC16);
    if (cfg.align[ALIGN_ACCEL][0]) alignSensors(ALIGN_ACCEL, accADC16); else acc.align(accADC16);
    for (i = 0; i < 3; i++) accADC[i] = (float)accADC16[i];
}

static void Gyro_getRawRot(void)
{
    int16_t gyroADC16[3], i;
    gyro.read(gyroADC16);                                         // range: +/- 8192; +/- 2000 deg/sec
    if (cfg.align[ALIGN_GYRO][0]) alignSensors(ALIGN_GYRO, gyroADC16); else gyro.align(gyroADC16);
    for (i = 0; i < 3; i++) gyroADC[i] = (float)gyroADC16[i];
}

void ACC_getADC(void)
{
    ACC_getRawRot();
    ACC_Common();
}

void Gyro_getADC(void)
{
    Gyro_getRawRot();
    GYRO_Common();
}

#ifdef BARO
void Baro_update(void)                                            // Note Pressure is now global for telemetry 1hPa = 1mBar
{
    static float    BaroTab[5], BaroSpikeTab[5];                  // Note: We don't care about runup bufferstate since first 50 runs are discarded anyway
    static uint32_t LastGeneraltime;
    static uint16_t baroDeadline = 0;
    static uint8_t  state = 0, Bidx = 0, SkipCnt = 0;
    float           temp;
    bool            rdy = false;
    uint8_t         i, maxsortidx = 4;

    newbaroalt = false;                                           // Reset Newbarovalue since it's iterative and not interrupt driven it's OK
    if (micros() - LastGeneraltime < baroDeadline) return;        // Make it rollover friendly

    switch (state)
    {
    case 0:
        baro.start_ut();                                          // Temperature Conversion start
        LastGeneraltime = micros();
        baroDeadline    = baro.ut_delay - 1;
        SkipCnt = 0;                                              // Reset Skipcounter, reduces 27ms delay to average 20ms delay for ms baro (37Hz to 50Hz)
        state++;
        break;
    case 1:
        baro.get_ut();                                            // Readout Temp fall through case
        state++;
    case 2:
        baro.start_up();                                          // Pressure Conversion start
        LastGeneraltime = micros();
        baroDeadline    = baro.up_delay - 1;
        state++;
        break;
    case 3:
        baro.get_up();                                            // Readout Pressure
        baroDeadline    = 0;                                      // Don't use delay between read. Cycletime is enough. Before: TimeNowMicros + baro.repeat_delay - 1;
        ActualPressure  = baro.calculate();                       // ActualPressure needed by mavlink
        BaroSpikeTab[0] = (1.0f - pow(ActualPressure / 101325.0f, 0.190295f)) * 4433000.0f; // I stick to the "slower", method - gives better results.
        BaroSpikeTab[4] = BaroSpikeTab[0];
        while(!rdy)                                               // Spikefilter now
        {
            rdy = true;
            for (i = 0; i < maxsortidx; i++)
            {
                temp = BaroSpikeTab[i];
                if (temp > BaroSpikeTab[i + 1])
                {
                    BaroSpikeTab[i]     = BaroSpikeTab[i + 1];
                    BaroSpikeTab[i + 1] = temp;
                    rdy = false;
                }
            }
            maxsortidx --;
        }
        BaroTab[Bidx] = BaroSpikeTab[2]; Bidx++;
        if (Bidx == 5) Bidx = 0;
        BaroAlt = 0;
        for (i = 0; i < 5; i++) BaroAlt += BaroTab[i];
        BaroAlt *= 0.2f;
        SkipCnt++;
        if (SkipCnt == 2 || baro.baro_type == 1) state = 0;       // Read new Temp every 2nd run gives us little more speed without loosing resolution. However it worsens BMP - so not done there // baro_type: 1 = BMP 2 = MS
        else state = 2;
        newbaroalt = true;
        break;
    }
}
#endif

#ifdef SONAR
#define SonarErrorLimit 5                                         // We will bridge 5 consecutive faulty reads, HC-SR04 = 300ms Maxbotix = 500ms
void Sonar_update(void)
{
    static  int16_t  LastGoodSonarAlt = -1;                       // Initialize with errorvalue
    static  uint32_t AcceptTimer = 0;
    static  uint8_t  Errorcnt = 0;                                // This is compared to SonarErrorLimit
    int16_t LastSonarAlt = sonarAlt;                              // Save Last Alt here for comparison
    uint8_t tilt;
    int32_t newdata = GetSnr();                                   // Keep it running for disconnect detection
    if (newdata)                                                  // 100 ms with Maxbotix, 60ms with HC-SR04
    {
        sonarAlt = newdata;
        tilt = 100 - constrain(TiltValue * 100.0f, 0, 100.0f);    // We don't care for upsidedownstuff, because althold is disabled than anyway
        if (cfg.snr_dbg) { debug[1] = tilt; debug[2] = sonarAlt; }  // Give raw tilt & sonar in debugmode
        if (sonarAlt >= cfg.snr_min && sonarAlt <= cfg.snr_max && tilt < cfg.snr_tilt)
        {
            LastGoodSonarAlt = sonarAlt;
            Errorcnt = SonarBreach = 0;                           // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
        }
        else
        {                                                         // So sonarvalues are not sane here
            Errorcnt = min(Errorcnt + 1, SonarErrorLimit);        // Increase Errorcount within upper limit            
            if (tilt < cfg.snr_tilt && sonarAlt != -1)            // Determine Limit breach type independent of tilt
            {
                if (sonarAlt <= cfg.snr_min) SonarBreach = 1;     // We breached lower limit
                if (sonarAlt >= cfg.snr_max) SonarBreach = 2;     // We breached upper limit
            }
            else SonarBreach = 0;                                 // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
            if (Errorcnt != SonarErrorLimit) sonarAlt = LastGoodSonarAlt; // Bridge error with last value, when it's -1 we take care later
            else sonarAlt = -1;
        }

        if (LastSonarAlt != -1 && sonarAlt != -1 && cfg.snr_diff && abs(sonarAlt - LastSonarAlt) > cfg.snr_diff) // Too much Difference between reads?
            sonarAlt = -1;
        
        if (sonarAlt < 0)                                         // Handle error here separately
        {
            LastGoodSonarAlt = -1;
            SonarStatus = 0;
        }
        else
        {
            switch(SonarStatus)
            {
            case 0:
                SonarStatus++;                                    // Definition of "SonarStatus" 0 = no contact, 1 = Made first contact, 2 = Steady contact
                AcceptTimer = currentTimeMS + 700;                // Set 700 ms timeout before signalizing "steady contact" this exceeds our "bridging" from above
                break;
            case 1:
                if (currentTimeMS >= AcceptTimer) SonarStatus++;  // 2 = Steady contact // imu/getEstimatedAltitude will be happy to know
            default:
                break;
            }
        }
    }
    if (cfg.snr_dbg) debug[0] = sonarAlt;                         // Display Sonaralt like seen by althold
}
#endif

#ifdef MAG
// Rearranged & Extended by Crashpilot
static void Mag_getRawADC_With_Gain(void)                         // Read aligned
{
    int16_t magADC[3];
    uint8_t i;
    hmc5883lRead(magADC);                                         // Does that now: alignSensors(ALIGN_MAG, magADC);
    for (i = 0; i < 3; i++) magADCfloat[i] = (float)magADC[i] * magCal[i];
}

void Mag_getADC(void)
{
    static uint32_t Lasttime = 0;
    uint8_t  i;
    uint32_t TimeNow = micros();
    if (calibratingM) Mag_Calibration();                          // Calibrates saves and resets
    if ((TimeNow - Lasttime) < 14000) return;                     // Do 70Hz in normal operation
    Lasttime = TimeNow;
    Mag_getRawADC_With_Gain();                                    // Read mag sensor with orientation correction do nothing more for now
    for (i = 0; i < 3; i++) magADCfloat[i] -= cfg.magZero[i];     // Adjust by BIAS (gathered by user calibration)
    alignBoardyaw(magADCfloat);
    HaveNewMag = true;
}

static void Mag_42Hz_AVG(float *xyztmp, uint16_t count)
{
    uint16_t i, axis;
    for (axis = 0; axis < 3; axis++) xyztmp[axis] = 0.0f;
    for (i = 0; i < count; i++)
    {
        delay(24);                                                // Do 42 Hz // Math 24ms * 500 * 5 = 60000ms = 1 Minute
        Mag_getRawADC_With_Gain();                                // Read mag sensor with correct orientation + gain
        for (axis = 0; axis < 3; axis++) xyztmp[axis] += magADCfloat[axis] / (float)count;
    }
}

static void Mag_Calibration(void)                                 // Called from XHz loop normally....
{
#define MAGmaxcount      500                                      // Take 500 samples at 10Hz rate i.e 50Sec
#define MAGerror       10000
#define MAGdiscardcnt     50
#define sflsdelta         0.0f
#define maxiterations    100
    float xyz[3], x_sumplain = 0.0f, x_sumsq = 0.0f, x_sumcube = 0.0f, y_sumplain = 0.0f, y_sumsq = 0.0f;
    float y_sumcube = 0.0f, z_sumplain = 0.0f, z_sumsq = 0.0f, z_sumcube = 0.0f, xy_sum = 0.0f;
    float xz_sum = 0.0f, yz_sum = 0.0f, x2y_sum = 0.0f, x2z_sum = 0.0f, y2x_sum = 0.0f, y2z_sum = 0.0f;
    float z2x_sum = 0.0f, z2y_sum = 0.0f, x2, y2, z2, x_sum, x_sum2, x_sum3, y_sum, y_sum2, y_sum3, z_sum;
    float z_sum2, z_sum3, XY, XZ, YZ, X2Y, X2Z, Y2X, Y2Z, Z2X, Z2Y, F0, F1, F2, F3, F4, A, B, C, A2, B2;
    float C2, QS, QB, Rsq, Q0, Q1, Q2, aA, aB, aC, nA, nB, nC, dA, dB, dC, fltsize = (float)MAGmaxcount;
    uint16_t i, gathercnt = (uint16_t)cfg.mag_time * 5;
    LD0_OFF();                                                    // Green LED OFF
    LD1_ON();                                                     // Red LED ON
    Mag_42Hz_AVG(xyz, MAGdiscardcnt);                             // Discard some
    for (i = 0; i < MAGmaxcount; i++)                             // Gather up Mag Data. Freeze FC. Adjust Mag readout JUST by GAIN/SCALE
    {
        Mag_42Hz_AVG(xyz, gathercnt);
        x2          = xyz[0] * xyz[0];                            // http://imaginaryz.blogspot.de/2011/04/least-squares-fit-sphere-to-3d-data.html
        y2          = xyz[1] * xyz[1];
        z2          = xyz[2] * xyz[2];
        x_sumplain += xyz[0];
        x_sumsq    += x2;
        x_sumcube  += x2     * xyz[0];
        y_sumplain += xyz[1];
        y_sumsq    += y2;
        y_sumcube  += y2     * xyz[1];
        z_sumplain += xyz[2];
        z_sumsq    += z2;
        z_sumcube  += z2     * xyz[2];
        xy_sum     += xyz[0] * xyz[1];
        xz_sum     += xyz[0] * xyz[2];
        yz_sum     += xyz[1] * xyz[2];
        x2y_sum    += x2     * xyz[1];
        x2z_sum    += x2     * xyz[2];
        y2x_sum    += y2     * xyz[0];
        y2z_sum    += y2     * xyz[2];
        z2x_sum    += z2     * xyz[0];
        z2y_sum    += z2     * xyz[1];
        LED0_TOGGLE
        LED1_TOGGLE
    }
    x_sum  = x_sumplain / fltsize;
    x_sum2 = x_sumsq    / fltsize;
    x_sum3 = x_sumcube  / fltsize;
    y_sum  = y_sumplain / fltsize;
    y_sum2 = y_sumsq    / fltsize;
    y_sum3 = y_sumcube  / fltsize;
    z_sum  = z_sumplain / fltsize;
    z_sum2 = z_sumsq    / fltsize;
    z_sum3 = z_sumcube  / fltsize;
    XY     = xy_sum     / fltsize;
    XZ     = xz_sum     / fltsize;
    YZ     = yz_sum     / fltsize;
    X2Y    = x2y_sum    / fltsize;
    X2Z    = x2z_sum    / fltsize;
    Y2X    = y2x_sum    / fltsize;
    Y2Z    = y2z_sum    / fltsize;
    Z2X    = z2x_sum    / fltsize;
    Z2Y    = z2y_sum    / fltsize;
    F0     =  x_sum2 + y_sum2 + z_sum2;
    F1     =  0.5f * F0;
    F2     = -8.0f * (x_sum3 + Y2X + Z2X);
    F3     = -8.0f * (X2Y + y_sum3 + Z2Y);
    F4     = -8.0f * (X2Z + Y2Z + z_sum3);
    A      = x_sum;
    B      = y_sum;
    C      = z_sum;
    A2     = A * A;
    B2     = B * B;
    C2     = C * C;
    QS     = A2 + B2 + C2;
    QB     = -2.0f * QS;
    Rsq    = F0 + QB + QS;
    Q0     = 0.5f * (QS - Rsq);
    Q1     = F1 + Q0;
    Q2     = 8.0f * (QS - Rsq + QB + F0);
    i      = 0;
    while (i < maxiterations)
    {
        i++;
        aA  = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
        aB  = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
        aC  = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
        aA  = (aA == 0.0f) ? 1.0f : aA;
        aB  = (aB == 0.0f) ? 1.0f : aB;
        aC  = (aC == 0.0f) ? 1.0f : aC;
        nA  = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
        nB  = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
        nC  = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);
        dA  = (nA - A);
        dB  = (nB - B);
        dC  = (nC - C);
        if ((dA * dA + dB * dB + dC * dC) <= sflsdelta) break;
        A   = nA;
        B   = nB;
        C   = nC;
        A2  = A * A;
        B2  = B * B;
        C2  = C * C;
        QS  = A2 + B2 + C2;
        QB  = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
        Rsq = F0 + QB + QS;
        Q0  = 0.5f * (QS - Rsq);
        Q1  = F1 + Q0;
        Q2  = 8.0f * (QS - Rsq + QB + F0);
    }
    cfg.magZero[0] = A;
    cfg.magZero[1] = B;
    cfg.magZero[2] = C;
    cfg.mag_calibrated = 1;
    for (i = 0; i < 3; i++)
    {
        if (fabs(cfg.magZero[i]) > MAGerror)
        {
            cfg.mag_calibrated = 0;                               // Supress GPS functions & Guicrazymag
            cfg.magZero[i] = 0;          
        }
    }
    writeParams(1);                                               // Calibration done, save whatever result
    systemReset(false);
}
#endif
