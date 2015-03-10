#include "board.h"
#include "mw.h"

// 2014 Crashpilot 1000
// Based on the Timecop / Mwii Port
flags_t  f;
int16_t  debug[4];
uint32_t currentTime   = 0;
uint32_t currentTimeMS = 0;
float    FLOATcycleTime = 0;
uint8_t  vbat;                                                       // battery voltage in 0.1V steps
float    telemTemperature1;                                          // gyro sensor temperature
volatile uint16_t failsafeCnt   = 0;
float    TiltValue;                                                  // 1.0 is horizontal 0 is vertical, minus is upsidedown
int16_t  rcData[MAX_RC_CHANNELS];                                    // RC RAWDATA int16_t rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // interval [1000;2000]
int16_t  rcDataSAVE [MAX_RC_CHANNELS];
int16_t  rcCommand[4];                                               // interval [esc min;esc max] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t  rssi;                                                       // 0 - 255 = 0%-100%
rcReadRawDataPtr rcReadRawFunc = NULL;                               // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)
uint8_t  rcOptions[CHECKBOXITEMS];
float    axisPID[3];
float    AvgCyclTime = 1;
static   float dynP8[2], dynD8[2];
static   float errorGyroI[2] = { 0, 0 }, errorAngleI[2] = { 0, 0 };
static   int32_t errorGyroI_YW = 0;

// **********************
// IMU & SENSORS
// **********************
float    headFreeModeHold;
float    heading = 0.0f;
float    magHold;
float    magneticDeclination;
bool     UpsideDown = false;
float    ACCDeltaTimeINS = 0;
bool     HaveNewMag = false;

// ***********************************************************
// SONAR / BARO / Autoland / Autostart / Failsave / Air - Trim
// ***********************************************************
uint8_t  SonarStatus = 0;                                            // 0 = no contact, 1 = made contact, 2 = steady contact
uint8_t  SonarBreach = 0;                                            // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
uint8_t  SonarLandWanted = 0;                                        // This is done to virtualize cfg.snr_land because failsafe can disable it and it could be saved in the worst case
float    ActualPressure;
float    GroundAlt;
float    BaroActualTemp;
int16_t  ESCnoFlyThrottle;
static   uint8_t GoodRCcnt;                                          // Number of good consecutive Signals before arming
static   int16_t FSBaroThrottle = 0;
static   bool CopterFlying = false;
static   uint8_t AirTrimstate = 0;
bool     ForceRCExpInit = true;

// **********************
// GPS
// **********************
volatile int32_t  IRQGPS_coord[2];                                   // They occure serial IRQ is done, and they are fed to Real_GPS_coord synchronized
volatile uint16_t IRQGPS_speed;
volatile uint16_t IRQGPS_grcrs;
volatile bool  GPS_FIX = false;
int32_t  Real_GPS_coord[2];                                          // RAW GPS Coords
int32_t  GPS_home[2];
int32_t  GPS_WP[2];                                                  // Currently used WP
volatile uint8_t  GPS_numSat;
uint32_t GPS_distanceToHome;                                         // distance to home
int32_t  GPS_directionToHome;                                        // direction to home or hol point in degrees
uint16_t GPS_speed;                                                  // speed in cm/s
volatile uint16_t GPS_altitude;                                      // altitude in m
uint8_t  GPS_update = 0;                                             // it's a binary toogle to distinct a GPS position update
float    GPS_angle[2] = {0, 0};                                      // it's the angles that must be applied for GPS correction
float    Last_GPS_angle[2] = {0, 0};
uint16_t GPS_ground_course = 0;                                      // DEG * 10
float    nav[2];                                                     // DEG * 100
int16_t  maxbank10  = 1;                                             // Maximum GPS Tiltangle in degree * 10 // preset to 1 for safety to prevent div 0
int16_t  maxbank100 = 1;                                             // Maximum GPS Tiltangle in degree * 100
float    GPSEXPO;
int8_t   nav_mode = NAV_MODE_NONE;                                   // Navigation mode
int8_t   wp_status = WP_STATUS_NONE;                                 // Waypoint status
int8_t   ph_status;
int32_t  WP_Target_Alt;
int16_t  WP_Desired_Climbrate;                                       // Climbrate in cm/s
bool     WP_Fastcorner;                                              // Dont decrease Speed at Target
float    sin_yaw_y;
float    cos_yaw_x;
static   uint8_t  PHminSat;

// **********************
// Overrides
// **********************
bool     BlockGPSAngles;
bool     BlockThrottle;                                              // Sets cfg.rc_mid
bool     BlockPitch;                                                 // Sets cfg.rc_mid
bool     BlockRoll;                                                  // Sets cfg.rc_mid

// **********************
// Battery monitoring
// **********************
uint8_t  batteryCellCount = 3;                                       // cell count
uint16_t batteryWarningVoltage;                                      // annoying buzzer after this one, battery ready to be dead

// **********************
// Motor monitoring
// **********************
uint8_t  motorpercent[MAX_MONITORED_MOTORS]; // Is set in board.h to 8
uint16_t motorabspwm[MAX_MONITORED_MOTORS];
uint8_t  NumberOfMotors;                                             // Store number of Motors used, initialized in main in mixerinit

// **********************
// LED & MWCRGB
// **********************
uint32_t LED_Value = 1500;
uint8_t  LED_Value_Delay = 8;

// **********************
// EEPROM
// **********************
uint32_t ScheduleEEPROMwriteMS = 0;

static int16_t RCDeadband(int16_t rcvalue, uint8_t rcdead);
static void    DoThrcmmd_DynPid(void);
static void    DoRcHeadfree(void);
static bool    DeadPilot(void);
static int16_t DoMotorStats(bool JustDoRcThrStat);
static void    DoLEDandBUZZER(void);
static void    DisArmCopter(void);
static void    MWCRGBMONO_LED(uint8_t buzz);
static void    ChkFailSafe(void);
static void    DoKillswitch(void);
static void    DoAirTrim(void);
static void    GetClimbrateTorcDataTHROTTLE(int16_t cr);
static void    DoRcArmingAndBasicStuff(void);
static void    AckTrimCheckTrimLimits(void);
static void    computeRC(void);
static void    GetRCandAuxfromBuf(void);
static void    ZeroErrorAngleI(void);
static void    calculate_Gtune(bool inirun, uint8_t ax);

void pass(void)                                                             // Feature pass
{
    static uint32_t blinktime;
    static uint8_t  lastpassmotor;
    uint32_t timetmp;
    computeIMU();
    timetmp = micros();

    if (DoGetRc50HzTimer())                                                 // 50Hz
    {
        if (failsafeCnt > 2)
        {
            rcData[THROTTLE] = cfg.esc_moff;                                // Motor off
            writeAllMotors(rcData[THROTTLE]);                               // Set all motors to zero just to be sure if user is messing in cli without saving
        }
        if (lastpassmotor != cfg.pass_mot) writeAllMotors(cfg.esc_moff);    // Motonumber was changed in cli without saving
        lastpassmotor = cfg.pass_mot;
        if (!cfg.pass_mot) writeAllMotors(rcData[THROTTLE]);                // All Motors?
        else pwmWriteMotor(cfg.pass_mot - 1, rcData[THROTTLE]);             // Specific Motor?
        failsafeCnt++;
    }
    f.ARMED = 0;                                                            // Always set this as a dummy so serial com accepts "#" and "R"
    serialCom(false);
    if ((int32_t)(timetmp - blinktime) >= 0)
    {
        blinktime = timetmp + 150000;
        LED1_TOGGLE
        LED0_TOGGLE
    }
}

/*
****************************************************************************
***                    MAIN LOOP                                         ***
****************************************************************************
*/

void loop(void)
{
    static uint32_t RTLGeneralTimer, AltRCTimer0 = 0, BaroAutoTimer, loopTime;
    float           delta, RCfactor, rcCommandAxis;
    float           PTerm = 0, ITerm = 0, DTerm = 0, PTermACC = 0, ITermACC = 0, ITermGYRO = 0, error = 0, prop = 0;
    static float    lastGyro[2] = {0, 0}, lastDTerm[2] = {0, 0};
    static uint8_t  ThrFstTimeCenter = 0, AutolandState = 0, AutostartState = 0, HoverThrcnt, RTLstate, ReduceBaroI = 0;
    static int8_t   Althightchange;
    static uint16_t HoverThrottle;
    static int16_t  BaroLandThrlimiter, SnrLandThrlimiter, initialThrottleHold, LastAltThrottle = 0;
    static int32_t  DistanceToHomeMetersOnRTLstart;
    static int16_t  AutostartClimbrate;
    static int32_t  FilterVario, AutostartTargetHight, AutostartFilterAlt;
    static stdev_t  variovariance;
    float           tmp0flt;
    int32_t         tmp0, PTermYW;
    int16_t         thrdiff;
    uint8_t         axis;

    if (DoGetRc50HzTimer())
    {
        DoRcArmingAndBasicStuff();
        DoThrcmmd_DynPid();                                                 // Populates rcCommand[THROTTLE] from rcData[THROTTLE] and does empiric pid attenuation
        if(sensors(SENSOR_GPS)) DoChkGPSDeadin50HzLoop();                   // Is GPS alive?
        PHminSat = cfg.gps_ph_minsat;                                       // Don't forget to set PH Minsats here!!
        if (feature(FEATURE_FAILSAFE))
        {
            ChkFailSafe();                                                  // Only check Failsafe if copter is armed
        }
        else
        {
            failsafeCnt = 0;
            FSBaroThrottle = 0;                                             // Set to invalid
        }

//      SPECIAL RTL Crashpilot
#define RTLsettleTime 2000                                                  // 2 sec
        if (sensors(SENSOR_GPS) && sensors(SENSOR_BARO) && f.GPS_FIX_HOME && rcOptions[BOXGPSHOME])
        {
            rcOptions[BOXBARO]     = 1;                                     // Baro On
            rcOptions[BOXMAG]      = 1;                                     // MAG ON
            rcOptions[BOXGPSHOLD]  = 1;                                     // GPS hold
            rcOptions[BOXPASSTHRU] = 0;                                     // Passthru off
            rcOptions[BOXHEADFREE] = 0;                                     // HeadFree off
            rcOptions[BOXGPSHOME]  = 0;                                     // RTL OFF
            rcData[THROTTLE] = cfg.rc_mid;                                  // Put throttlestick to middle: Althold
            PHminSat = 5;                                                   // Sloppy PH is sufficient
            if (!RTLstate) RTLstate = 1;                                    // Start RTL Sequence if it isn't already running
            if (GPS_numSat < 5) RTLstate = 0;                               // Error!
            if (cfg.rtl_mnd && RTLstate == 1 && GPS_distanceToHome < cfg.rtl_mnd)
                RTLstate = 0;                                               // Dont Do RTL if too close and RTL not already running

            switch (RTLstate)
            {
            case 0:                                                         // Error!! Do landing
                rcData[THROTTLE] = cfg.rc_minchk - 10;                      // Put throttlestick to lowest-10
                break;
            case 1:                                                         // prepare timer
                RTLGeneralTimer = currentTimeMS + RTLsettleTime;
                RTLstate++;
                break;
            case 2:                                                         // Hover certain time and wait for solid PH
                if (currentTimeMS > RTLGeneralTimer && ph_status == PH_STATUS_DONE) RTLstate++;
                break;
            case 3:                                                         // Check hight and climb if neccessary
                if (cfg.rtl_mnh)
                {
                    if (EstAlt < ((uint16_t)cfg.rtl_mnh * 100)) GetClimbrateTorcDataTHROTTLE((int16_t)cfg.rtl_cr);
                    else RTLstate++;
                }
                else RTLstate++;                                            // For safety, skip if turned off
                break;
            case 4:                                                         // Wait for Tailstuff before RTL
                if (cfg.nav_controls_heading)                               // Tail control
                {
                    if (cfg.nav_tail_first) magHold = wrap_180((float)(GPS_directionToHome - 180));
                    else magHold = GPS_directionToHome;
                    if(fabs(wrap_180(heading - magHold)) < 6.0f) RTLstate++; // Turns true, when in range of +-6 degrees
                }
                else RTLstate++;
                break;
            case 5:                                                         // Prepare RTL
                DistanceToHomeMetersOnRTLstart = GPS_distanceToHome;        // Set actual distance to Home in meters
                rcOptions[BOXGPSHOLD] = 0;                                  // GPS hold OFF
                rcOptions[BOXGPSHOME] = 1;                                  // Engage RTL
                RTLstate++;
                break;
            case 6:                                                         // OMG Do the f** RTL now
                rcOptions[BOXGPSHOLD] = 0;                                  // GPS hold OFF
                rcOptions[BOXGPSHOME] = 1;                                  // RTL
                tmp0 = (int32_t)GPS_distanceToHome - DistanceToHomeMetersOnRTLstart; // tmp0 contains flyawayvalue
                if ((cfg.gps_rtl_flyaway && tmp0 > (int32_t)cfg.gps_rtl_flyaway) ||
                        (wp_status == WP_STATUS_DONE && ph_status == PH_STATUS_DONE)) RTLstate++;
                break;
            case 7:                                                         // Do Autoland
                rcData[THROTTLE] = cfg.rc_minchk - 10;                      // Put throttlestick to lowest-10
                break;                                                      // Repeat forever because Autoland will disarm the thing
            }
        }
        else RTLstate = 0;                                                  // No BOXGPSHOME request? Reset Variable
//      SPECIAL RTL Crashpilot END


        if (rcOptions[BOXGPSAUTO])
        {
// Insert award winning stuff here
        }
        else f.GPS_AUTO_MODE = 0;

#ifdef BARO
        if (sensors(SENSOR_BARO))
        {
            if (rcOptions[BOXBARO] && GroundAltInitialized && f.ARMED)
            {
                if (!f.BARO_MODE)                                               // Initialize Baromode here if it isn't already
                {
                    Althightchange   = 0;
                    AutolandState    = 0;
                    AutostartState   = 0;
                    ThrFstTimeCenter = 0;
                    ReduceBaroI      = 0;
                    AltHold          = EstAlt;
                    if (FSBaroThrottle)                                         // Use Baro failsafethrottle here
                    {
                        LastAltThrottle     = FSBaroThrottle;
                        initialThrottleHold = FSBaroThrottle;
                    }
                    else
                    {
                        LastAltThrottle     = rcCommand[THROTTLE];
                        initialThrottleHold = rcCommand[THROTTLE];
                    }
                    f.BARO_MODE = 1;                                            // Finally set baromode to initialized
                }

                if(CopterFlying)                                                // Are we somehow airborne?
                {
                    if(AutostartState)
                    {
                        if ((abs(rcData[THROTTLE] - cfg.rc_mid) > cfg.rc_dbah)) // Autostartus interruptus
                        {
                            AutostartState      = 0;
                            ThrFstTimeCenter    = 0;
                            Althightchange      = 0;
                            ReduceBaroI         = 0;
                            AltHold             = EstAlt;
                            initialThrottleHold = LastAltThrottle;
                        }
                    }
                    else
                    {
                        if (rcData[THROTTLE] < cfg.rc_minchk && !AutolandState) AutolandState = 1; // Start Autoland
                        if (rcData[THROTTLE] > cfg.rc_minchk && AutolandState)  // Autolandus interruptus on Userinput reset some stuff
                        {
                            AutolandState       = 0;
                            ThrFstTimeCenter    = 0;
                            Althightchange      = 0;
                            ReduceBaroI         = 0;
                            AltHold             = EstAlt;
                            initialThrottleHold = LastAltThrottle;
                        }
                    }
                }
                else
                {
                    if (cfg.as_trgt && !AutostartState && ThrFstTimeCenter == 3) AutostartState = 1; // Start Autostart
                }
            }
            else
            {
                f.BARO_MODE     = 0;                                            // No Baroswitch, no Autoland/start
                AutolandState   = 0;
                AutostartState  = 0;
                LastAltThrottle = 0;
                ReduceBaroI     = 0;
            }
            if (AutolandState || AutostartState)                                // Switch to Angle mode when AutoBarofunctions anyway
            {
                rcOptions[BOXHORIZON]  = 0;
                rcOptions[BOXANGLE]    = 1;
            }
        }
#endif

#ifdef MAG
        if (sensors(SENSOR_MAG) &&  cfg.mag_calibrated)
        {
            if (rcOptions[BOXMAG])
            {
                if (!f.MAG_MODE)
                {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            }
            else f.MAG_MODE = 0;

            f.HEADFREE_MODE = rcOptions[BOXHEADFREE];
            if (rcOptions[BOXHEADADJ]) headFreeModeHold = heading;              // acquire new heading
        }
#endif

        if (sensors(SENSOR_GPS) && GPS_FIX && GPS_numSat >= 5)
        {
            if (rcOptions[BOXGPSHOME] && f.GPS_FIX_HOME)                        // Crashpilot RTH is possible with 5 Sats for emergency and if homepos is set!
            {
                if (!f.GPS_HOME_MODE)
                {
                    f.GPS_HOME_MODE = 1;
                    nav_mode = NAV_MODE_RTL;                                    // Set nav_mode before so GPS_set_next_wp can init it.
                    GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                }
            }
            else f.GPS_HOME_MODE = 0;

            if (rcOptions[BOXGPSHOLD] && GPS_numSat >= PHminSat)                // Crashpilot Only do poshold with specified Satnr or more
            {
                if (!f.GPS_HOLD_MODE)
                {
                    f.GPS_HOLD_MODE = 1;
                    nav_mode = NAV_MODE_POSHOLD;                                // Will set ph_status = PH_STATUS_NONE;
                    GPS_set_next_wp(&Real_GPS_coord[LAT], &Real_GPS_coord[LON]);
                }
            }
            else
            {
                f.GPS_HOLD_MODE = 0;
                ph_status = PH_STATUS_NONE;
            }

            if (DoingGPS())
            {
                if (cfg.rc_dbgps)                                               // Do some additional deadband for GPS, if needed
                {
                    rcCommand[PITCH] = RCDeadband(rcCommand[PITCH], cfg.rc_dbgps);
                    rcCommand[ROLL]  = RCDeadband(rcCommand[ROLL],  cfg.rc_dbgps);
                }
                rcOptions[BOXHORIZON]  = 0;
                rcOptions[BOXANGLE]    = 1;
                rcOptions[BOXMAG]      = 1;
            }
        }                                                                       // END of sensors SENSOR_GPS
        else
        {
            f.GPS_HOME_MODE = f.GPS_HOLD_MODE = 0;
            nav_mode  = NAV_MODE_NONE;
            ph_status = PH_STATUS_NONE;
        }

        f.PASSTHRU_MODE = rcOptions[BOXPASSTHRU];
        if (cfg.mixerConfiguration == MULTITYPE_FLYING_WING || cfg.mixerConfiguration == MULTITYPE_AIRPLANE) f.HEADFREE_MODE = 0;
        if (sensors(SENSOR_ACC))
        {
            if (rcOptions[BOXANGLE])                                            // Prevent simultan Angle and Horizon mode
            {                                                                   // In that case Angle mode will win.
                if (!f.ANGLE_MODE)
                {
                    ZeroErrorAngleI();
                    f.ANGLE_MODE   = 1;
                    f.HORIZON_MODE = 0;
                }
            }
            else
            {
                f.ANGLE_MODE = 0;
                if (rcOptions[BOXHORIZON])
                {
                    if (!f.HORIZON_MODE)
                    {
                        ZeroErrorAngleI();
                        f.HORIZON_MODE = 1;
                    }
                }
                else f.HORIZON_MODE = 0;
            }
        }
        else
        {
            f.ANGLE_MODE   = 0;
            f.HORIZON_MODE = 0;
        }

        if (rcOptions[BOXGTUNE])
        {
            if (!f.GTUNE)
            {
                f.GTUNE = 1;
                calculate_Gtune(true, 0);
            }
        }
        else f.GTUNE = 0;
        
        DoRcHeadfree();                                                         // Rotates Rc commands according mag and homeheading in headfreemode
        DoKillswitch();
        DoAirTrim();

// *********** END OF 50Hz RC LOOP ***********
    }
    else
    {
        DoLEDandBUZZER();                                                       // Do that, if not doing RC stuff
    }

#ifdef SONAR
    if (sensors(SENSOR_SONAR)) Sonar_update();                                  // Update "SonarStatus". Do it here because damn HC-SR04 will block a little
#endif

#ifdef MAG
    if (sensors(SENSOR_MAG)) Mag_getADC();
#endif

    currentTime   = micros();
    currentTimeMS = millis();
    if ((int32_t)(currentTime - loopTime) >= 0)
    {
        loopTime = currentTime + cfg.looptime;
        computeIMU();                                                           // looptime Timeloop starts here on predefined basis

#ifdef BARO
        if (sensors(SENSOR_BARO))                                               // The normal stuff to keep it simple
        {
            Baro_update();
            getEstimatedAltitude();                                             // Combine with sonar if possible
        }

#define HoverTimeBeforeLand     2000                                            // Wait 2 sec in the air for VirtualThrottle to catch up
        if (sensors(SENSOR_BARO) && f.BARO_MODE && f.ARMED)                     // GroundAltInitialized must not be checked but armed, in case of dumb user -> see above
        {
            getAltitudePID();                                                   // !Do not forget to calculate the Baropids...

            if(AutostartState > 1)
            {
                AutostartFilterAlt = ((AutostartFilterAlt << 1) + AutostartFilterAlt + (int32_t)EstAlt) >> 2;
                FilterVario        = ((FilterVario << 1)        + FilterVario        + (int32_t)vario) >> 2;
            }
            switch(AutostartState)
            {
            case 0:
                break;
            case 1:                                                             // Initialize Autostart with relative targethight
                AutostartTargetHight = (int32_t)EstAlt + (int32_t)cfg.as_trgt * 100;
                AutostartFilterAlt   = (int32_t)EstAlt;
                FilterVario          = 0;                                       // We are standing
                AutostartClimbrate   = (int16_t)cfg.as_lnchr << 1;
                initialThrottleHold  = ESCnoFlyThrottle;                        // Set higher baselinethrottle than esc min.
                BlockGPSAngles       = true;                                    // Block GPS on the Ground
                BaroAutoTimer        = currentTimeMS + 800;                     // prepare timer to settle data
                AutostartState++;
                break;
            case 2:                                                             // Let things settle
                if (currentTimeMS > BaroAutoTimer)
                {
                    devClear(&variovariance);
                    GetClimbrateTorcDataTHROTTLE(AutostartClimbrate);           // Ignition...
                    AutostartState++;
                }
                BlockGPSAngles = true;                                          // Block GPS on the Ground
                break;
            case 3:                                                             // Wait for liftoff. It is assumed when the std dev of the vario exceeds a limit or if the climbrate is beyond 50cm/s
                devPush(&variovariance, (float)FilterVario);
                if (FilterVario > 50 ||
                   ((uint8_t)devStandardDeviation(&variovariance) > cfg.as_stdev) ||
                   (AutostartFilterAlt > AutostartTargetHight + 50))
                {
                    CopterFlying       = true;                                  // Liftoff! Force alhold to reset virtual targethight
                    rcData[THROTTLE]   = cfg.rc_mid;                            // We have a liftoff, force althold (reset internal altholdtarget)
                    ThrFstTimeCenter   = 0;                                     // Force ini
                    AutostartClimbrate = (int16_t)cfg.as_clmbr;
                    AutostartState++;
                }
                else 
                {
                    GetClimbrateTorcDataTHROTTLE(AutostartClimbrate);           // No, liftoff increase virtual targethight (like a rubberband to pull the copter off the ground)
                    BlockGPSAngles = true;                                      // Block GPS on the Ground
                }
                break;
            case 4:                                                             // Now climb with desired rate to targethight.
                tmp0 = (int32_t)((float)AutostartFilterAlt + (float)FilterVario * cfg.bar_lag);// Actual predicted hight
                AutostartClimbrate = constrain_int((abs(AutostartTargetHight - tmp0) / 3), 10, (int16_t)cfg.as_clmbr);// Slow down when getting closer to target
                if (AutostartFilterAlt < AutostartTargetHight)
                {
                    GetClimbrateTorcDataTHROTTLE(AutostartClimbrate);           // Climb
                }
                else
                {
                    rcData[THROTTLE] = cfg.rc_mid;                              // Hover
                    ThrFstTimeCenter = 0;                                       // Forc ini
                    AutostartState   = 0;
                }
                break;
            }

            switch (AutolandState)
            {
            case 0:                                                             // No Autoland Do nothing
                SnrLandThrlimiter = BaroLandThrlimiter = 0;                     // Reset it here! BaroAutoTimer = 0;
                break;
            case 1:                                                             // Start Althold
                rcData[THROTTLE] = cfg.rc_mid;                                  // Put throttlestick to middle
                BaroAutoTimer    = currentTimeMS + HoverTimeBeforeLand;         // prepare timer
                HoverThrcnt      = 1;                                           // Initialize Hoverthrottlestuff here
                HoverThrottle    = DoMotorStats(true);                          // Try to get average here from flight. Returns 0 if not possible.
                if (HoverThrottle < LastAltThrottle) HoverThrottle = LastAltThrottle; // Take the bigger one as base
                AutolandState++;
                break;
            case 2:                                                             // We hover here and gather the Hoverthrottle
                rcData[THROTTLE] = cfg.rc_mid;                                  // Put throttlestick to middle: Hover some time to gather Hoverthr
                HoverThrottle   += LastAltThrottle;
                HoverThrcnt++;
                if (HoverThrcnt == 20)
                {
                    HoverThrottle = HoverThrottle / 20;                         // Average of 20 Values
                    if (currentTimeMS > BaroAutoTimer) AutolandState++;
                    else HoverThrcnt = 1;
                }
                break;
            case 3:                                                             // Start descent initialize Variables
                if (cfg.al_debounce)                                            // Set BaroLandThrlimiter now, if wanted
                {
                    tmp0 = (int32_t)HoverThrottle - (int32_t)cfg.esc_min;       // tmp0 contains absolute absolute hoverthrottle
                    if (tmp0 > 0) BaroLandThrlimiter = HoverThrottle + ((float)tmp0 * (float)cfg.al_debounce * 0.01f);// Check here to be on the safer side. Don't set BaroLandThrlimiter if something is wrong
                }
                GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_barolr);
                if (sensors(SENSOR_SONAR) && SonarStatus == 2) GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_snrlr);// Set al_snrlr on steady sonar contact
                BaroAutoTimer = 0;
                AutolandState++;
                break;
            case 4:                                                             // Keep descending and check for landing
                GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_barolr);
                if (sensors(SENSOR_SONAR))                                      // Adjust Landing
                {
                    if (SonarStatus == 2) GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_snrlr);// SolidSonarContact use maybe different Landrate
                    if (SonarLandWanted && SonarBreach == 1 && !SnrLandThrlimiter)             // Sonarlanding if SonarBreach = 1 (Proximity breach) fix the upper throttlevalue
                        SnrLandThrlimiter = cfg.esc_max;                        // Set maximal thr as upper limit, will be adjusted below
                }
                if (LastAltThrottle <= ESCnoFlyThrottle && !BaroAutoTimer)      // ESCnoFlyThrottle is set upon Baro initialization in sensors/sensorsAutodetect
                {
                    BaroAutoTimer = currentTimeMS;
                    if (SnrLandThrlimiter) BaroAutoTimer += (uint32_t)cfg.al_tosnr;// Aided Sonar landing is wanted and limiter set, do perhaps shorter timeout then
                    else BaroAutoTimer += (uint32_t)cfg.al_tobaro;              // No Sonar limiter, do the normal timeout
                }
                if (LastAltThrottle > ESCnoFlyThrottle) BaroAutoTimer = 0;      // Reset Timer
                if ((BaroAutoTimer && currentTimeMS > BaroAutoTimer) || UpsideDown) AutolandState++; // Proceed to disarm if timeup or copter upside down
                break;
            case 5:                                                             // Shut down Copter forever....
                DisArmCopter();
                break;
            }

            thrdiff = rcData[THROTTLE] - cfg.rc_mid;
            tmp0    = abs(thrdiff);
            
            if (!ThrFstTimeCenter)                                              // Initialize "have passed center" check
            {                                                                   // Note: thrdiff = 0 is checked below
                if (thrdiff > 0) ThrFstTimeCenter = 1;                          // Initial Throttlestick above middle
                if (thrdiff < 0) ThrFstTimeCenter = 2;                          // Initial Throttlestick below middle
            }
  
            if (ThrFstTimeCenter != 3)
            {
                if (tmp0 <= cfg.rc_dbah) ThrFstTimeCenter = 3;                  // We are inside Deadband
                else                                                            // We are outside Deadband, check if me missed a "passing middle" situation
                {
                    if (ThrFstTimeCenter == 1 && thrdiff < 0)
                    {
                        ThrFstTimeCenter = 3;
                    }
                    else
                    {
                        if (ThrFstTimeCenter == 2 && thrdiff > 0) ThrFstTimeCenter = 3;
                    }
                }
                if (ThrFstTimeCenter == 3)
                {
                    AltRCTimer0 = 0;                                            // Force first Run
                    ReduceBaroI = 0;
                }
            }

            if (currentTimeMS >= AltRCTimer0)                                   // X Hz Loop
            {
                AltRCTimer0 = currentTimeMS + 100;
                if (ThrFstTimeCenter == 3)
                {
                    if (tmp0 > cfg.rc_dbah)
                    {
                        initialThrottleHold += BaroP / 100;                     // Adjust Baselinethr by 1% of BaroP
                        if(thrdiff > 0)                                         // Note: thrdiff can not be zero here
                        {
                            ReduceBaroI    = 1;
                            Althightchange = 1;
                            if(LastAltThrottle < cfg.esc_max) AltHold += (float)(thrdiff - cfg.rc_dbah) * 0.125f;
                        }
                        else
                        {
                            ReduceBaroI    =  0;
                            Althightchange = -1;
                            if (LastAltThrottle > cfg.esc_min)
                            {
                                tmp0flt = (float)(thrdiff + cfg.rc_dbah) * 0.125f;
                                if (!AutolandState) AltHold += tmp0flt * cfg.bar_dscl;// Descent with less rate in manual mode
                                else AltHold += tmp0flt;
                            }
                        }
                    }
                    else                                                        // Stick is to center here
                    {
                        if (Althightchange)                                     // Are we coming from a hight change? Project stoppingpoint.
                        {
                            tmp0flt = vario * cfg.bar_lag;                      // tmp0flt = projected cm.
                            if(Althightchange > 0)
                            {
                                if (vario < 0.0f) tmp0flt = 0.0f;
                            }
                            else
                            {
                                if (vario < -0.5f) tmp0flt *= 0.5f;
                                else tmp0flt = 0.0f;
                            }
                            AltHold = EstAlt + tmp0flt;
                            initialThrottleHold = LastAltThrottle;              // This is for starting in althold otherwise the initialthr would be idle throttle
                            ReduceBaroI    = 0;
                            Althightchange = 0;
                        }
                    }
                }
            }                                                                   // End of X Hz Loop
            if (AutolandState || AutostartState || ph_status != PH_STATUS_NONE) BaroD = 0;// Don't do Throttle angle correction when autolanding/starting or during PH
            if (ReduceBaroI) BaroI *= 1.0f - constrain(fabs(AltHold - EstAlt) * 0.003f, 0.0f, 0.5f);// Reduce Variobrake
            tmp0flt = BaroP + BaroD - BaroI;
            if(tmp0flt < 0.0f) tmp0flt *= 0.9f;                                 // Reduce downpid to 90%
            rcCommand[THROTTLE] = constrain((int32_t)tmp0flt + initialThrottleHold, cfg.esc_min, cfg.esc_max);
            if (AutolandState)                                                  // We are Autolanding and
            {
                if (SnrLandThrlimiter)                                          // Check sonarlimiter first
                {                                                               // Sonar has given proximity alert and sonar land support is wanted
                    BlockGPSAngles = true;                                      // Block GPS on the Ground
                    if (LastAltThrottle < SnrLandThrlimiter) SnrLandThrlimiter = LastAltThrottle; // Adjust limiter here
                    rcCommand[THROTTLE] = min(rcCommand[THROTTLE], SnrLandThrlimiter);
                }
                else if (BaroLandThrlimiter) rcCommand[THROTTLE] = min(rcCommand[THROTTLE], BaroLandThrlimiter);      // Only do Barolimiter on landing, if we have no Sonarlimiter
            }
            LastAltThrottle = rcCommand[THROTTLE];
        }

// Baro STATS LOGGING
        if (sensors(SENSOR_BARO) && f.ARMED)
        {
            tmp0 = (int32_t)EstAlt / 100;
            if (tmp0 > cfg.MaxAltMeter) cfg.MaxAltMeter = tmp0;
            if (tmp0 < cfg.MinAltMeter) cfg.MinAltMeter = tmp0;
        }
// Baro STATS LOGGING
#endif

#ifdef MAG
        if (sensors(SENSOR_MAG))
        {
            float magP;
            if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE)
            {
                if (DoingGPS()) magP = (float)cfg.gps_yaw;                      // Do slower mag P in gps mode
                else magP = (float)cfg.P8[PIDMAG];
                if (f.SMALL_ANGLES_25) rcCommand[YAW] -= wrap_180(heading - magHold) * magP / 30;// 18 deg
            }
            else magHold = heading;
        }
#endif

        if (sensors(SENSOR_GPS) && sensors(SENSOR_MAG))                         // Only do GPS stuff if the Mag is available
        {
            if (!GPS_alltime() || !DoingGPS() || !f.GPS_FIX_HOME || !cfg.mag_calibrated || !f.ARMED) // Only do further GPS stuff, if needed and possible
            {
                GPS_reset_nav();
                nav_mode  = NAV_MODE_NONE;
                ph_status = PH_STATUS_NONE;
                wp_status = WP_STATUS_NONE;
            }
            else                                                                // proceed here if mag calibrated and the rest is fine as well
            {
                GPS_angle[ROLL]  = constrain((nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) * 0.1f,  -maxbank10, maxbank10);// Nav angles are in DEG * 100
                GPS_angle[PITCH] = constrain((nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) * 0.1f,  -maxbank10, maxbank10);// GPS_angle are expected to be in DEG * 10, I keep that for now

                for (axis = 0; axis < 2; axis++)
                {
                    if(!BlockGPSAngles && CopterFlying)
                    {
                        GPS_angle[axis]      = (Last_GPS_angle[axis] + GPS_angle[axis]) * 0.5f;// No wrapping needed they can not exceed 90DEG
                        Last_GPS_angle[axis] = GPS_angle[axis];
                        tmp0flt              = GPS_angle[axis] / (float)maxbank10;
                        tmp0flt              = constrain(tmp0flt, -1.0f, 1.0f); // Put in range of -1 +1
                        GPS_angle[axis]      = SpecialIntegerRoundUp((tmp0flt * (1.0f - GPSEXPO) + tmp0flt * tmp0flt * tmp0flt * GPSEXPO) * (float)maxbank10); // Do expo here, and some rounding and jitter cutoff
                    }
                    else
                    {
                        GPS_angle[axis]      = 0;
                        Last_GPS_angle[axis] = 0;
                    }
                }
            }
        }
        else GPS_angle[0] = GPS_angle[1] = 0;                                   // Zero GPS influence on the ground
        BlockGPSAngles = false;

        RCfactor  = ACCDeltaTimeINS / (MainDptCut + ACCDeltaTimeINS);           // used for pt1 element

        //VERY DIRTY! IS ALREADY FIXED BUT NOT IN THIS UPLOAD BECAUSE DONE IN IMU PART THERE
        tmp0flt  = (uint16_t)FLOATcycleTime & (uint16_t)0xFFFC;                 // Filter last 2 bit jitter
        tmp0flt /= 3000.0f;
        //VERY DIRTY! IS ALREADY FIXED BUT NOT IN THIS UPLOAD BECAUSE DONE IN IMU PART THERE
        tmp0  = ((int32_t)rcCommand[YAW] * (((int32_t)cfg.yawRate << 1) + 40)) >> 5;
        error = tmp0 - SpecialIntegerRoundUp(gyroData[YAW] * 0.25f);          // Less Gyrojitter works actually better
        errorGyroI_YW = constrain(errorGyroI_YW + (int32_t)(error * (float)cfg.I8[YAW] * tmp0flt), -268435454, +268435454);        
        if(cfg.rc_dbyw)
        {
            if (rcCommand[YAW]) errorGyroI_YW = 0;
        }
        else
        {
            if (abs(tmp0) > 50) errorGyroI_YW = 0;
        }
        axisPID[YAW] = constrain(errorGyroI_YW >> 13, -250, +250);
        PTermYW      = ((int32_t)error * (int32_t)cfg.P8[YAW]) >> 6;
        if(NumberOfMotors > 3)                                              // Constrain YAW by D value if not servo driven in that case servolimits apply
        {
            tmp0 = 300;
            if (cfg.D8[YAW]) tmp0 -= (int32_t)cfg.D8[YAW];
            PTermYW = constrain(PTermYW, -tmp0, tmp0);
        }
        axisPID[YAW] += PTermYW;  

        if(f.GTUNE && f.ARMED) calculate_Gtune(false, YAW);
        
        if(f.HORIZON_MODE) prop = (float)min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 450) / 450.0f;
        for (axis = 0; axis < 2; axis++)
        {
            rcCommandAxis = (float)rcCommand[axis];                             // Calculate common values for pid controllers
            if ((f.ANGLE_MODE || f.HORIZON_MODE)) error = constrain(2.0f * rcCommandAxis + GPS_angle[axis], -500.0f, +500.0f) - angle[axis] + cfg.angleTrim[axis];
            switch (cfg.mainpidctrl)
            {
            case 0:
                if (f.ANGLE_MODE || f.HORIZON_MODE)
                {
                    PTermACC          = error * (float)cfg.P8[PIDLEVEL] * 0.008f;
                    tmp0flt           = (float)cfg.D8[PIDLEVEL] * 5.0f;
                    PTermACC          = constrain(PTermACC, -tmp0flt, +tmp0flt);
                    errorAngleI[axis] = constrain(errorAngleI[axis] + error * ACCDeltaTimeINS, -30.0f, +30.0f);
                    ITermACC          = errorAngleI[axis] * (float)cfg.I8[PIDLEVEL] * 0.08f;
                }
                if (!f.ANGLE_MODE)
                {
                    if (abs((int16_t)gyroData[axis]) > 2560) errorGyroI[axis] = 0.0f;
                    else
                    {
                        error            = (rcCommandAxis * 320.0f / (float)cfg.P8[axis]) - gyroData[axis];
                        errorGyroI[axis] = constrain(errorGyroI[axis] + error * ACCDeltaTimeINS, -192.0f, +192.0f);
                    }
                    ITermGYRO = errorGyroI[axis] * (float)cfg.I8[axis] * 0.01f;
                    if (f.HORIZON_MODE)
                    {
                        PTerm = PTermACC + prop * (rcCommandAxis - PTermACC);
                        ITerm = ITermACC + prop * (ITermGYRO     - ITermACC);
                    }
                    else
                    {
                        PTerm = rcCommandAxis;
                        ITerm = ITermGYRO;
                    }
                }
                else
                {
                    PTerm = PTermACC;
                    ITerm = ITermACC;
                }
                PTerm           -= gyroData[axis] * dynP8[axis] * 0.003f;
                delta            = (gyroData[axis] - lastGyro[axis]) / ACCDeltaTimeINS;
                lastGyro[axis]   = gyroData[axis];
                lastDTerm[axis] += RCfactor * (delta - lastDTerm[axis]);
                DTerm            = lastDTerm[axis] * dynD8[axis] * 0.00007f;
                break;
// Alternative Controller by alex.khoroshko http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=30#p37465
// But a little modified...
            case 1:                                                             // 1 = New mwii controller (float pimped + pt1element)
                if (!f.ANGLE_MODE)                                              // control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                {
                    tmp0flt = (float)(((int32_t)(cfg.rollPitchRate + 27) * (int32_t)rcCommand[axis]) >> 2);
                    if (f.HORIZON_MODE) tmp0flt += ((float)cfg.I8[PIDLEVEL] * error) * 0.16f;
                }
                else tmp0flt      = (float)cfg.P8[PIDLEVEL] * error * 0.09f;
                tmp0flt          -= gyroData[axis];
                PTerm             = (float)cfg.P8[axis] * tmp0flt * 0.002f;
                errorGyroI[axis] += (float)cfg.I8[axis] * tmp0flt * ACCDeltaTimeINS;
                errorGyroI[axis]  = constrain_flt(errorGyroI[axis], -5500.0f, 5500.0f);// errorGyroI[axis]  = constrain(errorGyroI[axis], -17176.0f, 17176.0f);
                ITerm             = errorGyroI[axis] * 0.015f;
                delta             = (tmp0flt - lastGyro[axis]) / ACCDeltaTimeINS;
                lastGyro[axis]    = tmp0flt;
                lastDTerm[axis]  += RCfactor * (delta - lastDTerm[axis]);
                DTerm             = -((float)cfg.D8[axis] * lastDTerm[axis] * 0.00001f);// D scaled up by 2
                break;
            }                                                                   // End of Switch
            axisPID[axis] = SpecialIntegerRoundUp(PTerm + ITerm - DTerm);       // Round up result.
            if (f.GTUNE && f.ARMED) calculate_Gtune(false, axis);
        }
        
        if (f.ARMED)
        {
            if (rcCommand[THROTTLE] > ESCnoFlyThrottle) CopterFlying = true;
        }
        else
        {
            CopterFlying = false;
// SCHEDULE EEPROM WRITES HERE
// 1. Because all changes within that timeout are collected and then written at once without bothering the eeprom too much
// 2. While flying you can save so many Data as you want in cfg.x and when you land and disarm they are actually written
            if (ScheduleEEPROMwriteMS && currentTimeMS >= ScheduleEEPROMwriteMS)
            {
                writeParams(0);                                                 // Write, don't blink
                ScheduleEEPROMwriteMS = 0;                                      // Reset Timer do this only once
            }
        }

        tmp0 = rcCommand[THROTTLE];                                             // Save Original THROTTLE
        if(f.ARMED && cfg.rc_flpsp && cfg.acc_calibrated && UpsideDown && !f.ANGLE_MODE) // Putting flipsupport here
            rcCommand[THROTTLE] = cfg.esc_min + ((rcCommand[THROTTLE] - cfg.esc_min) / ((int16_t)cfg.rc_flpsp + 1));// will make it possible in althold as well
        mixTableAndWriteMotors();
        writeServos();
        DoMotorStats(false);                                                    // False means no hoverthrottlegeneration for failsafe
        rcCommand[THROTTLE] = tmp0;                                             // Restore Original THROTTLE
    }
    if((FLOATcycleTime - AvgCyclTime) > (AvgCyclTime * 0.05f)) serialCom(true); // If exceed 5% do reduced serial, but limited to 3 slow runs there
    else serialCom(false);
    AvgCyclTime += 0.0001f * (FLOATcycleTime - AvgCyclTime);                    // Slow Baseline cycletime Keep it out of cycletime loop.
}

/*
****************************************************************************
***                    G_Tune                                            ***
****************************************************************************
	G_Tune Mode
	This is the multiwii implementation of ZERO-PID Algorithm
	http://technicaladventure.blogspot.com/2014/06/zero-pids-tuner-for-multirotors.html
	The algorithm has been originally developed by Mohammad Hefny (mohammad.hefny@gmail.com)

	You may use/modify this algorithm on your own risk, kindly refer to above link in any future distribution.
*/
/*
// version 1.0.0: MIN & MAX & Tuned Band
// version 1.0.1:
				a. error is gyro reading not rc - gyro.
				b. OldError = Error no averaging.
				c. No Min MAX BOUNDRY
//	version 1.0.2:
				a. no boundaries
				b. I - Factor tune.
				c. time_skip

// Crashpilot: Reduced to just P tuning in a predefined range - so it is not "zero pid" anymore.
   Tuning is limited to just work when stick is centered besides that YAW is tuned in non Acro as well.
   See also:
   http://diydrones.com/profiles/blogs/zero-pid-tunes-for-multirotors-part-2
   http://www.multiwii.com/forum/viewtopic.php?f=8&t=5190
   Gyrosetting 2000DPS
   GyroScale = (1 / 16,4 ) * RADX(see board.h) = 0,001064225154 digit per rad/s

    cfg.gt_lolimP[ROLL]   = 20; [10..200] Lower limit of ROLL P during G tune.
    cfg.gt_lolimP[PITCH]  = 20; [10..200] Lower limit of PITCH P during G tune.
    cfg.gt_lolimP[YAW]    = 20; [10..200] Lower limit of YAW P during G tune.
    cfg.gt_hilimP[ROLL]   = 70; [0..200]  Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    cfg.gt_hilimP[PITCH]  = 70; [0..200]  Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    cfg.gt_hilimP[YAW]    = 70; [0..200]  Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    cfg.gt_pwr            = 0;  [0..10] Strength of adjustment
*/

static void calculate_Gtune(bool inirun, uint8_t ax)
{
    static  int8_t time_skip[3];
    static  int16_t OldError[3], result_P64[3];
    static  int32_t AvgGyro[3];
    int16_t error, diff_G, threshP;
    uint8_t i;

    if (inirun)
    {
        for (i = 0; i < 3; i++)
        {
            if ((cfg.gt_hilimP[i] && cfg.gt_lolimP[i] > cfg.gt_hilimP[i]) ||    // User config error disable axis for tuning
               (NumberOfMotors < 4 && i == YAW)) cfg.gt_hilimP[i] = 0;          // Disable Yawtuning for everything below a quadcopter
            if(cfg.P8[i] < cfg.gt_lolimP[i]) cfg.P8[i] = cfg.gt_lolimP[i];
            result_P64[i] = (int16_t)cfg.P8[i] << 6;                            // 6 bit extra resolution for P.
            OldError[i]   = 0;
            time_skip[i]  = -125;
        }
    }
    else
    {
        if(rcCommand[ax] || (ax != YAW && (f.ANGLE_MODE || f.HORIZON_MODE)))    // Block Tuning on stickinput. Always allow Gtune on YAW, Roll & Pitch only in acromode
        {
            OldError[ax]  = 0;
            time_skip[ax] = -125;                                               // Some settletime after stick center. (125 + 16)* 3ms clycle = 423ms (ca.)
        }
        else
        {
            if (!time_skip[ax]) AvgGyro[ax] = 0;
            time_skip[ax]++;
            if (time_skip[ax] > 0)
            {
                if (ax == YAW) AvgGyro[ax] += 32 * ((int16_t)gyroData[ax] / 32);// Chop some jitter and average
                else AvgGyro[ax] += 128 * ((int16_t)gyroData[ax] / 128);        // Chop some jitter and average
            }

            if (time_skip[ax] == 16)                                            // ca 48 ms
            {
                AvgGyro[ax] /= time_skip[ax];                                   // AvgGyro[ax] has now very clean gyrodata
                time_skip[ax] = 0;

                if (ax == YAW)
                {
                    threshP = 20;
                    error   = -AvgGyro[ax];
                }
                else
                {
                    threshP = 10;
                    error   = AvgGyro[ax];
                }
              
                if (cfg.gt_hilimP[ax] && error && OldError[ax] && error != OldError[ax]) // Don't run when not needed or pointless to do so
                {
                    diff_G = abs(error) - abs(OldError[ax]);
                    if ((error > 0 && OldError[ax] > 0) || (error < 0 && OldError[ax] < 0))
                    {
                        if (diff_G > threshP) result_P64[ax] += 64 + cfg.gt_pwr;// Shift balance a little on the plus side.
                        else
                        {
                            if (diff_G < -threshP)
                            {
                                if (ax == YAW) result_P64[ax] -= 64 + cfg.gt_pwr;
                                else result_P64[ax] -= 32;
                            }
                        }
                    }
                    else
                    {
                        if (abs(diff_G) > threshP && ax != YAW) result_P64[ax] -= 32; // Don't use antiwobble for YAW
                    }
                    result_P64[ax] = constrain(result_P64[ax], (int16_t)cfg.gt_lolimP[ax] << 6, (int16_t)cfg.gt_hilimP[ax] << 6);
                    cfg.P8[ax]     = result_P64[ax] >> 6;
                }
                OldError[ax] = error;
            }
        }
    }
}

/*
****************************************************************************
***                    MAIN LOOP MISC HELPERS                            ***
****************************************************************************
*/
static int16_t DoMotorStats(bool JustDoRcThrStat)
{
    static uint32_t motortotal[MAX_MONITORED_MOTORS], DatasetCnt, LastMotorStatTimeMS;
    static uint8_t  maxmotnr = 0, motcasestate = 0;
    uint32_t Onepercent, Sum;
    uint8_t i;

    if (JustDoRcThrStat)
    {
        if (motcasestate != 1 || !maxmotnr) return 0;                           // Error! No data / no motor
        Sum = 0;
        for (i = 0; i < maxmotnr; i++) Sum += ((motortotal[i] / DatasetCnt) + cfg.esc_min);
        return Sum / maxmotnr;
    }

    if (currentTimeMS - LastMotorStatTimeMS >= 100)                             // Do Motorstats at 10 Hz rate
    {
        LastMotorStatTimeMS = currentTimeMS;
        switch(motcasestate)
        {
        case 0:                                                                 // Bootuprun / Reset state
            for (i = 0; i < MAX_MONITORED_MOTORS; i++) motortotal[i] = 0;       // Clear sum
            DatasetCnt = 0;
            maxmotnr   = min(NumberOfMotors, MAX_MONITORED_MOTORS);
            if (f.ARMED) motcasestate++;                                        // Wait for Arming
            break;
        case 1:                                                                 // Collect data here
            if (f.ARMED)
            {
                for (i = 0; i < maxmotnr; i++) motortotal[i] += motor[i] - cfg.esc_min;
                DatasetCnt++;
            } else motcasestate++;                                              // Disarmed! Go on to calculate
            break;
        case 2:                                                                 // Calculate percentage and reset statemachine
            Onepercent = 0;
            for (i = 0; i < maxmotnr; i++)
            {
                Onepercent    +=  motortotal[i] / 100;                          // Overflow very unlikely but /100 saves us anyway. Precision is not relevant.
                motorabspwm[i] = (motortotal[i] / DatasetCnt) + cfg.esc_min;
            }
            for (i = 0; i < maxmotnr; i++) motorpercent[i] = motortotal[i] / Onepercent;
            motcasestate = 0;
            break;
        }
    }
    return 0;                                                                   // Not necessary
}

static void DisArmCopter(void)
{
    f.ARMED        = 0;
    f.OK_TO_ARM    = 0;
}

static void ZeroErrorAngleI(void)
{
    errorAngleI[0] = 0.0f;
    errorAngleI[1] = 0.0f;
}

int32_t SpecialIntegerRoundUp(float val)                                        // If neg value just represents a change in direction rounding to next higher number is "more" negative
{
    if (val > 0) return val + 0.5f;
    else if (val < 0) return val - 0.5f;
    else return 0;
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1)
    {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    }
    else
    {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

static float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

float constrain_flt(float amt, float low, float high)
{
    return constrain(amt, low, high);
}

int32_t constrain_int(int32_t amt, int32_t low, int32_t high)
{
    return constrain(amt, low, high);
}

/*
****************************************************************************
***                    RC FUNCTIONS                                      ***
****************************************************************************
*/
static void ArmIFpossible(void)
{
    if (sensors(SENSOR_ACC)  && !cfg.acc_calibrated) return;
    else if (sensors(SENSOR_BARO) && (!GroundAltInitialized || f.BARO_MODE)) return;
    else if (ScheduleEEPROMwriteMS || f.ARMED) return;
    else if (feature(FEATURE_FAILSAFE))
    {
        if (GoodRCcnt > (5 * cfg.fs_delay)) f.ARMED = 1;
    } else f.ARMED = 1;
    if(f.ARMED) headFreeModeHold = heading;
}

static void ManualDisArmDependOnBaro(void)
{
    if ((sensors(SENSOR_BARO) && f.BARO_MODE)) return;
    f.ARMED = 0;
}

static void DoRcArmingAndBasicStuff(void)
{
    static uint8_t rcDelayCommand = 0;
    uint8_t        limit[4], i;                                                 // 1 = min 2 = max

    for (i = 0; i < 4; i++)
    {
        if (rcData[i] > cfg.rc_maxchk)      limit[i] = 2;
        else if (rcData[i] < cfg.rc_minchk) limit[i] = 1;
        else limit[i] = 0;
    }

    if (limit[THROTTLE] == 1)
    {
        rcDelayCommand++;
        if (!f.BARO_MODE)                                                       // Keep "I" values in baromode + rc min (autoland)
        {
            errorGyroI[0]  = 0;
            errorGyroI[1]  = 0;
            errorGyroI_YW  = 0;
            ZeroErrorAngleI();
        }
        if (!f.ARMED && limit[YAW] == 1)
        {
            if (limit[PITCH] == 1)
            {
                if (rcDelayCommand == 20) systemReset(false);                   // calibratingGyro is blocking so we reset here.
            }
            else if ((limit[PITCH] + limit[ROLL]) == 4)
            {
                if (rcDelayCommand == 20 && !AirTrimstate) AirTrimstate = 1;
            }
            else rcDelayCommand = 0;
        }
        else if (cfg.activate[BOXARM])
        {
            if (!rcOptions[BOXARM]) f.OK_TO_ARM = 1;
            if (rcOptions[BOXARM] && f.OK_TO_ARM) ArmIFpossible();
            else ManualDisArmDependOnBaro();
            rcDelayCommand = 0;
        }
        else if ((limit[YAW] == 1 || (cfg.rc_rllrm && limit[ROLL] == 1)) && f.ARMED)
        {
            if (rcDelayCommand == 20)
            {
                ManualDisArmDependOnBaro();
                rcDelayCommand = 0;
            }
        }
        else if ((limit[YAW] == 2 || ((cfg.rc_rllrm + limit[ROLL]) == 3)) && limit[PITCH] != 2 && !f.ARMED && f.SMALL_ANGLES_25)
        {
            if (rcDelayCommand == 20)
            {
                ArmIFpossible();
                rcDelayCommand = 0;
            }
        }
        else rcDelayCommand = 0;
    }
    else if (!f.ARMED && limit[THROTTLE] == 2)
    {
        if (limit[YAW] == 1 && limit[PITCH] == 1)
        {
            if (rcDelayCommand == 20) calibratingA = true;                      // Launch ACC cal
            rcDelayCommand++;
        }
        else if (limit[YAW] == 2 && limit[PITCH] == 1)
        {
            if (rcDelayCommand == 20) calibratingM = true;                      // Launch Mag cal
            rcDelayCommand++;
        }
        else if (limit[PITCH] == 2)
        {
            cfg.angleTrim[PITCH] += RcTrimstep;
            AckTrimCheckTrimLimits();
        }
        else if (limit[PITCH] == 1)
        {
            cfg.angleTrim[PITCH] -= RcTrimstep;
            AckTrimCheckTrimLimits();
        }
        else if (limit[ROLL] == 2)
        {
            cfg.angleTrim[ROLL] += RcTrimstep;
            AckTrimCheckTrimLimits();
        }
        else if (limit[ROLL] == 1)
        {
            cfg.angleTrim[ROLL] -= RcTrimstep;
            AckTrimCheckTrimLimits();
        }
        else rcDelayCommand = 0;
    }
    if (!f.ARMED && feature(FEATURE_LCD))
    {
        if (limit[THROTTLE] == 1 && ((limit[YAW] + limit[PITCH]) == 4)) serialOSD();
        else if (OLED_Type > 0) OLED_Status();
    }
}

// Can not invoke Autolanding!
// cr is in cm/s Climbrate of "0" does althold
static void GetClimbrateTorcDataTHROTTLE(int16_t cr)
{
    float tmp;
    if (!cr)
    {
        rcData[THROTTLE] = cfg.rc_mid;
        return;
    }
    tmp = ((float)cr * 0.8f) + cfg.rc_mid;
    if (cr > 0) rcData[THROTTLE] = tmp + cfg.rc_dbah;
    else rcData[THROTTLE] = tmp - cfg.rc_dbah;
    rcData[THROTTLE] = constrain(rcData[THROTTLE], cfg.rc_minchk + 1, cfg.rc_maxchk);
}

static int16_t RCDeadband(int16_t rcvalue, uint8_t rcdead)                      // Actually needed for additional GPS deadband
{
    if (abs(rcvalue) < rcdead) rcvalue = 0;
    else if (rcvalue > 0) rcvalue = rcvalue - (int16_t)rcdead;
    else rcvalue = rcvalue + (int16_t)rcdead;
    return rcvalue;
}

uint16_t pwmReadRawRC(uint8_t chan)
{
    uint16_t data;
    if (chan > 7) data = pwmRead(chan);
    else data = pwmRead(cfg.rcmap[chan]);
    if (data < 750 || data > 2250) data = cfg.rc_mid;
    return data;
}

bool DoGetRc50HzTimer(void)
{
    static uint32_t LastTime = 0;
    bool timeup;
    uint32_t Tnow = micros();

    if ((Tnow - LastTime) >= 20000)                                             // 50Hz
    {
        LastTime = Tnow;
        timeup   = true;
    } else timeup = false;

    if (SerialRCRX)
    {
        if (spektrumFrameComplete() || graupnersumhFrameComplete()) computeRC();
    }
    else
    {
        if (timeup) computeRC();
    }
    if(timeup) GetRCandAuxfromBuf();
    return timeup;
}

static void computeRC(void)                                                     // Just harvest RC Data
{
    static uint16_t rcData2Values[MAX_RC_CHANNELS][2];
    static uint8_t  bufindex = 100;                                             // Abuse that for signalize Initrun, without casting another static
    uint16_t rawval, avg;
    uint8_t  chan;

    if(bufindex == 100)                                                         // Ini run, this is mainly a tribute for anal users to present them with 1500 in gui and not with 1498 or so
    {
        for (chan = 0; chan < MAX_RC_CHANNELS; chan++)
        {
            rcData[chan]           = cfg.rc_mid;
            rcDataSAVE[chan]       = cfg.rc_mid;
            rcData2Values[chan][0] = cfg.rc_mid;
            rcData2Values[chan][1] = cfg.rc_mid;
        }
        bufindex = 0;
    }
    else
    {
        for (chan = 0; chan < MAX_RC_CHANNELS; chan++)
        {
            if (chan > 3 + cfg.rc_auxch) rcDataSAVE[chan] = 1000;               // We are out of aux set to low
            else
            {
                rawval = rcReadRawFunc(chan);
                if(cfg.rc_lowlat || SerialRCRX)
                {
                    avg = (rawval + rcData2Values[chan][0]) >> 1 ;
                    rcData2Values[chan][0] = rawval;
                }
                else
                {
                    avg = (rawval + rcData2Values[chan][0] + rcData2Values[chan][1]) / 3; // GCC would do the same with a "for" loop
                    rcData2Values[chan][bufindex] = rawval;
                }
                if (avg < rcDataSAVE[chan] - 3) rcDataSAVE[chan] = avg + 2;     // rcDataSAVE can pass unchanged here
                else if (avg > rcDataSAVE[chan] + 3) rcDataSAVE[chan] = avg - 2;
            }
        }
        bufindex ^= 1;
    }
}

static void GetRCandAuxfromBuf(void)                                            // Make RC Data available
{
    uint32_t auxState = 0;
    uint8_t  i, MaxAuxNumber = max(cfg.rc_auxch, 4);
    int8_t   offs = -1;

    for (i = 0; i < MAX_RC_CHANNELS; i++) rcData[i] = constrain(rcDataSAVE[i], 1000, 2000); // Limit to PWM range 1000-2000us
    if (cfg.devorssi) rssi = ((constrain(rcData[cfg.rc_auxch + 3], 900, 1923) - 900) >> 2); // [0-255] Based on the Specs send in by BAMFAX
    if(BlockRoll)
    {
        rcData[ROLL] = cfg.rc_mid;
        BlockRoll    = false;
    }
    if(BlockPitch)
    {
        rcData[PITCH] = cfg.rc_mid;
        BlockPitch    = false;
    }
    if(BlockThrottle)
    {
        rcData[THROTTLE] = cfg.rc_mid;
        BlockThrottle    = false;
    }
    for (i = 0; i < MaxAuxNumber; i++)                                      // Even with no Aux capable TX (4CH) enable some fixed auxfunctions setable in GUI
    {
        auxState |= (uint32_t)(rcData[AUX1 + i] < 1300) << ++offs;
        auxState |= (uint32_t)(1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << ++offs;
        auxState |= (uint32_t)(rcData[AUX1 + i] > 1700) << ++offs;
    }
    for (i = 0; i < CHECKBOXITEMS; i++) rcOptions[i] = (auxState & cfg.activate[i]) > 0;
}

#define PTCHLEN  7
#define PTCHLMT  5
#define THRLEN  12
#define THRLMT  10
static void DoThrcmmd_DynPid(void)
{
    static   int16_t  LkpPtchRll[PTCHLEN], LKPThr[THRLEN];                  // lookup table for expo & RC rate PITCH+ROLL
    int32_t  i, prop1, prop2 = 100, y, tmp, tmp2;
    if(ForceRCExpInit)
    {
        ForceRCExpInit = false;
        for (i = 0; i < PTCHLEN; i++) LkpPtchRll[i] = ((2500 + (int32_t)cfg.rcExpo8 * (i * i - 25)) * i * (int32_t)cfg.rcRate8) / 2500;
        for (i = 0; i < THRLEN; i++)
        {
            tmp = i * 10 - cfg.thrMid8;
            if (tmp > 0) y = 100 - cfg.thrMid8;
             else if (tmp < 0) y = cfg.thrMid8;
              else y = 1;
            LKPThr[i] = (int32_t)cfg.thrMid8 * 10 + tmp * (100 - (int32_t)cfg.thrExpo8 + (int32_t)cfg.thrExpo8 * (tmp * tmp) / (y * y)) / 10;
            LKPThr[i] = cfg.esc_min + (int32_t)(cfg.esc_max - cfg.esc_min) * LKPThr[i] / 1000; // [esc_min;esc_max]
        }
    }

    if (rcData[THROTTLE] >= 1500 && cfg.dynThrPID)                          // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    {
        prop2 -= constrain(((uint16_t)cfg.dynThrPID * (rcData[THROTTLE] - 1500)) / 500, 0, 99);  // Leave at least 1%
    }

    for (i = 0; i < 3; i++)
    {
        tmp   = min(abs(rcData[i] - cfg.rc_mid), 500);
        prop1 = 100;
        if (i != YAW)                                                       // ROLL & PITCH
        {
            if (cfg.rc_db)
            {
                if (tmp > cfg.rc_db) tmp -= cfg.rc_db;
                else                 tmp  = 0;
            }
            tmp2         = min(tmp / 100, PTCHLMT);                         // Prevent out of bounds
            rcCommand[i] = LkpPtchRll[tmp2] + (tmp - tmp2 * 100) * (LkpPtchRll[tmp2 + 1] - LkpPtchRll[tmp2]) / 100;
            prop1       -= (((uint32_t)cfg.rollPitchRate * tmp) / 500);
            prop1        = prop1 * prop2 / 100;
            dynP8[i]     = ((int32_t)cfg.P8[i] * prop1) / 100;              // dynI8[axis] = (uint16_t) cfg.I8[axis] * prop1 / 100;
            dynD8[i]     = ((int32_t)cfg.D8[i] * prop1) / 100;
        }
        else                                                                // YAW
        {
            if (cfg.rc_dbyw)
            {
                if (tmp > cfg.rc_dbyw) tmp -= cfg.rc_dbyw;
                else                   tmp  = 0;
            }
            rcCommand[i] = tmp;
        }
        if (rcData[i] < cfg.rc_mid) rcCommand[i] = -rcCommand[i];
    }
    tmp = constrain(rcData[THROTTLE], cfg.rc_minchk, 2000);
    tmp = (tmp - cfg.rc_minchk) * 1000 / (2000 - cfg.rc_minchk);            // [MINCHCK;2000] -> [0;1000]
    tmp2 = min(tmp / 100, THRLMT);
    rcCommand[THROTTLE] = constrain(LKPThr[tmp2] + (tmp - tmp2 * 100) * (LKPThr[tmp2 + 1] - LKPThr[tmp2]) / 100, cfg.esc_min, cfg.esc_max); // [0;1000] -> expo -> [esc_min;esc_max]
}

static void DoRcHeadfree(void)
{
    int16_t rcCommand_PITCH;
    float   cosDiff, sinDiff, radDiff;
    if (!f.HEADFREE_MODE) return;
    radDiff = wrap_180(heading - headFreeModeHold) * RADX;                  // Degree to RAD
    cosDiff = cosf(radDiff);
    sinDiff = sinf(radDiff);
    rcCommand_PITCH  = (float)rcCommand[PITCH] * cosDiff + (float)rcCommand[ROLL]  * sinDiff;
    rcCommand[ROLL]  = (float)rcCommand[ROLL]  * cosDiff - (float)rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
}

static bool DeadPilot(void)
{
    static int16_t  lastchecksum = 0;
    static uint32_t deadtimer    = 0;
    bool            output       = false;
    int16_t         checksum     = 0;
    uint8_t         i;

    if (!f.ARMED || !cfg.fs_ddplt)
    {
        deadtimer = 0;
        return output;
    }
    for (i = 0; i < 4; i++) checksum += rcData[i];
    if (abs(checksum - lastchecksum) > 20) deadtimer = 0;
    else if(!deadtimer) deadtimer = currentTimeMS + (uint32_t)cfg.fs_ddplt * 1000;
    lastchecksum = checksum;
    if (deadtimer && currentTimeMS > deadtimer) output = true;
    return output;
}

static void DoAirTrim(void)
{
    static float Trimtemp[2];
    static bool  MeasureIni;
    uint8_t      i;

    switch(AirTrimstate)
    {
    case 0:                                                                 // Case0: Do nothing
        break;
    case 1:
        if (!cfg.activate[BOXARM] || !cfg.acc_calibrated || f.ARMED)        // f.ARMED is redundant, acc_calibrated can just turn true if acc present
        {
            AirTrimstate = 0;
            return;                                                         // End this here if requirements are not met
        }
        blinkLED(15, 20, 1);                                                // Blink for confirmation AirTrim enabled
        for (i = 0; i < 2; i++) cfg.angleTrim[i] = 0.0f;                    // Clear Trims
        AirTrimstate++;
        break;
    case 2:                                                                 // Wait for arm
        if (f.ARMED)
        {
            MeasureIni = false;
            AirTrimstate++;                                                 // Proceed
        }
        break;
    case 3:
        if(!rcOptions[BOXARM] && f.ARMED)                                   // Arm SW OFF in flight
        {
            if(!MeasureIni)
            {
                MeasureIni = true;
                for (i = 0; i < 2; i++)
                {
                    cfg.angleTrim[i] = 0.0f;                                // Neutral trims
                    Trimtemp[i]      = angle[i];                            // Reduce filterrunup
                }
            }
            for (i = 0; i < 2; i++) Trimtemp[i] += 0.02f * (angle[i] - Trimtemp[i]);
        }
        if(rcOptions[BOXARM] && MeasureIni)                                 // Arm SW ON AGAIN
        {
            MeasureIni = false;
            if(abs((int16_t)Trimtemp[0]) <= Trimlimit && abs((int16_t)Trimtemp[1]) <= Trimlimit)// Apply trimming when in range, otherwise keep zero.
            {
                cfg.angleTrim[0] = Trimtemp[0];
                cfg.angleTrim[1] = Trimtemp[1];
            }
        }
        if (!f.ARMED)
        {
            ScheduleEEPROMwriteMS = 1;                                      // Schedule write
            AirTrimstate = 0;
            blinkLED(15, 20, 1);                                            // Blink for confirmation Airtrim successfully done.
        }
        break;
    }
}

// If Copter is armed by box and a cfg.rc_killt (in ms) is defined
// Don't enable Killswitch when Notarmed or Rc Signal disturbed, dont do killswitch if we want to Airtrim
static void DoKillswitch(void)
{
    static uint32_t Killtimer = 0;
    if (!cfg.rc_killt || AirTrimstate || !cfg.activate[BOXARM]) return;
    if (!rcOptions[BOXARM] && !Killtimer) Killtimer = currentTimeMS + (uint32_t)cfg.rc_killt;
    if (!f.ARMED || rcOptions[BOXARM] || failsafeCnt > 1) Killtimer = 0;
    if (Killtimer && currentTimeMS >= Killtimer) DisArmCopter();            // Kill Copter
}

// Failsafe routine Modified & Moved here by Crashpilot
// 4 Cases:
// 1: No Sensors -> Do the old stuff
// 2: GPS -> Do RTL and do the old stuff
// 3: Baro -> Do Baro Autoland
// 4: BARO & GPS -> Do full feature RTL
// Logic Do FS if FScount is too high, or pilotdead (no stickinput for x seconds)
// If FS is already running (fstimer !=0) but the goodcount is too low (maybe single good spike) keep FS running
static void ChkFailSafe(void)
{
    static uint32_t Failsafetimer;
    uint8_t i;

    if (CopterFlying && ((failsafeCnt > (5 * cfg.fs_delay)) || DeadPilot() || (Failsafetimer && GoodRCcnt < (5 * cfg.fs_delay))))
    {
        if (!Failsafetimer) Failsafetimer = currentTimeMS + 100 * (uint32_t)(cfg.fs_delay + cfg.fs_ofdel);
        for (i = 0; i < 3; i++)                                             // Center Roll Pitch Yaw here
        {
            rcData[i]    = cfg.rc_mid;                                      // rcData contains raw rc values
            rcCommand[i] = 0;                                               // rcCommand contains RPY -500 0 + 500 and 1000-2000 for throttle
        }
        rcOptions[BOXPASSTHRU] = 0;                                         // Passthru OFF
        rcOptions[BOXHEADFREE] = 0;                                         // HeadFree OFF
        rcOptions[BOXHORIZON]  = 0;                                         // Horizon OFF
        rcOptions[BOXANGLE]    = 1;                                         // Angle ON
        rcOptions[BOXMAG]      = 1;                                         // MAG ON
        rcOptions[BOXBARO]     = 1;                                         // Baro ON
        PHminSat               = 5;                                         // Sloppy PH is sufficient
        AirTrimstate           = 0;                                         // Not necessary but disable Air - Trim

        if (cfg.fs_nosnr) SonarLandWanted = 0;                              // This can disable the aided sonarlanding, that could cause problems on trees

        i = 0;                                                              // i = (sensors(SENSOR_GPS) << 1) | sensors(SENSOR_BARO);// Bitshifting of a boolean returnvalue is not good?
        if (sensors(SENSOR_GPS))
        {
            i = 2;
            if (f.GPS_FIX_HOME && !cfg.fs_jstph)
            {
                rcOptions[BOXGPSHOME] = 1;                                  // Do RTL+Autoland+MotorOFF
                rcOptions[BOXGPSHOLD] = 0;                                  // No explicit PH
            }
            else                                                            // OMG we have no Homepos - just do Autoland, turn on PH just in case...
            {
                rcOptions[BOXGPSHOME] = 0;
                rcOptions[BOXGPSHOLD] = 1;                                  // Pos Hold ON
                rcData[THROTTLE]      = cfg.rc_minchk - 10;                 // Do Autoland turns off motors
            }
        }
#ifdef BARO
        if (sensors(SENSOR_BARO))
        {
            i++;
            if (!FSBaroThrottle)                                            // Throttlechannel not valid any more, find new baselinethrottle for althold
            {
                FSBaroThrottle = DoMotorStats(true);                        // Try to get FSBaro ESC Throttle from statistics
                if (FSBaroThrottle < ESCnoFlyThrottle) FSBaroThrottle = max(cfg.fs_rcthr, ESCnoFlyThrottle); // Throttle too low, take predefined throttle then
            }
        }
#endif
        switch(i)                                                           // Do Failsafe according to sensorset
        {
        case 0:                                                             // No Sensors just go down
        case 2:                                                             // Just GPS and no baro
            rcCommand[THROTTLE] = rcData[THROTTLE] = cfg.fs_rcthr;          // set rcCommand[THROTTLE] & rcData[THROTTLE](avoid disarm) to a predef. value, we have no baro...who builds such copters today?
            if (currentTimeMS > Failsafetimer) DisArmCopter();              // Turn OFF motors Time in 0.1sec
            break;
        case 1:                                                             // Just Baro. Do Autoland
            rcData[THROTTLE] = cfg.rc_minchk - 10;                          // Do Autoland turns off motors
        case 3:                                                             // GPS & Baro. Do RTH & Autoland. If Homepos not set just Autoland & try PH
            break;
        }
    }
    else
    {
        Failsafetimer  = 0;
        FSBaroThrottle = 0;                                                 // Set to invalid
    }

    if (!failsafeCnt) GoodRCcnt = min(GoodRCcnt + 1, 250);                  // Increase goodcount while failcount is zero
    else GoodRCcnt = 0;
    failsafeCnt++;                                                          // reset to 0 by RC driver on Signal
}

/*
****************************************************************************
***                    LED & BUZZER FUNCTIONS                            ***
****************************************************************************
*/
static void DoLEDandBUZZER(void)
{
    static uint32_t Timebase = 0;
    static uint8_t  buzzerFreq, vbatTimer = 0, ind = 0, blinkcase = 0, rdycase = 0, blinkblock;
    static uint16_t vbatRawArray[8], cnt, LEDringcnt = 0;
    bool            RedON = false, DoRedBlitzer = false, DoRedSatCnt = false;
    uint16_t        vbatRaw = 0;
    uint8_t         i;

    if (feature(FEATURE_VBAT))
    {
        if (!(++vbatTimer % VBATFREQ))
        {
            vbatRawArray[(ind++) % 8] = adcGetChannel(ADC_BATTERY);
            for (i = 0; i < 8; i++) vbatRaw += vbatRawArray[i];
            vbat = batteryAdcToVoltage(vbatRaw / 8);
        }
        if ((vbat > batteryWarningVoltage) || (vbat < cfg.vbatmincellvoltage)) buzzerFreq = 0;  // VBAT ok, buzzer off
        else buzzerFreq = 4;                                                // low battery
    }
    buzzer(buzzerFreq);                                                     // external buzzer routine that handles buzzer events globally now

    if (f.ARMED) LD0_ON();
    else
    {
        if (!f.SMALL_ANGLES_25) LD0_ON();
        else LD0_OFF();
    }

    if (currentTimeMS >= Timebase)                                          // GPS Blink sats, Indicate rdy to fly, Show Angle/Horizon in flight (if no gps sats)
    {
        Timebase = currentTimeMS + 10;                                      // Timebase 10 ms = ca 100Hz
        if (sensors(SENSOR_GPS) && GPS_FIX && GPS_numSat >= 5) DoRedSatCnt = true; // Satcount overrides everything
        else
        {
            if (!f.ARMED)
            {
                if (sensors(SENSOR_BARO))
                {
                    if (GroundAltInitialized) DoRedBlitzer = true;
                }
                else DoRedBlitzer = true;
                if (sensors(SENSOR_ACC) && !cfg.acc_calibrated) DoRedBlitzer = false;
            }
            else if (f.ANGLE_MODE || f.HORIZON_MODE) RedON = true;
        }

        if (DoRedSatCnt)                                                    // Blink Satnumbers
        {
            switch(blinkcase)
            {
            case 0:                                                         // Case new blinkblock
                blinkblock = GPS_numSat - 4;                                // Change suggested by Hinkel So 5 Sats will blink one time
                rdycase = 0;
            case 1:
                cnt = 30;                                                   // 300 ms on
                blinkcase = 2;
            case 2:                                                         // Case LED ON
                cnt--;
                RedON = true;
                if (!cnt)
                {
                    cnt = 70;                                               // 700 ms off
                    blinkcase++;
                }
                break;
            case 3:                                                         // Case LED OFF
                cnt--;
                if (!cnt) blinkcase++;
                break;
            case 4:
                blinkblock--;
                if (!blinkblock)
                {
                    cnt = 200;                                              // 2.0s Break before next flickerblock
                    blinkcase++;
                }
                else blinkcase = 1;
                break;
            case 5:                                                         // Case LED LONG OFF
                cnt--;
                if (!cnt) blinkcase = 0;
                break;
            }
        }
        else
        {
            blinkcase = 0;
            if (DoRedBlitzer)                                               // Blitz Copter is sharp
            {
                switch(rdycase)                                             // Rdy blitzer
                {
                case 0:
                    cnt = 2;
                    rdycase++;
                case 1:
                    if(!cnt)
                    {
                        cnt = 18;
                        rdycase++;
                    }
                    else cnt--;
                    RedON = true;
                    break;
                case 2:
                    if(!cnt) rdycase = 0;
                    else cnt--;
                    break;
                }
            } else rdycase = 0;
        }
        if(RedON)LD1_ON();
        else LD1_OFF();

        if (feature(FEATURE_LED))
        {
            MWCRGBMONO_LED(buzzerFreq);
#ifdef LEDRING
            if (cfg.LED_Type == 3)
            {
                if (++LEDringcnt >= 5)
                {
                    LEDringcnt = 0;
                    ledringState();
                }
            }
#endif
        }
    }

    switch(cfg.tele_prot)                                                   // 0 (Dfault)=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)
    {
    case 1:
        initFRSKYTelemetry(f.ARMED);                                        // This will switch to/from 9600 or 115200 baud depending on state.
        break;
    case 3:                                                                 // 3 = Mavlink @57KBaud
        initMinimOSDTelemetry(f.ARMED);
        break;
    default:
        break;
    }
}                                                                           // END

// Some misc stuff goes here, LED etc.
// Purpose: Make mw.c a little bit more readable
// LED Inverter
void LD0_OFF(void)
{
    if (cfg.LED_invert) LED0_ON
    else LED0_OFF
}
void LD0_ON(void)
{
    if (cfg.LED_invert) LED0_OFF
    else LED0_ON
}
void LD1_OFF(void)
{
    if (cfg.LED_invert) LED1_ON
    else LED1_OFF
}
void LD1_ON(void)
{
    if (cfg.LED_invert) LED1_OFF
    else LED1_ON
}

static void AckTrimCheckTrimLimits(void)
{
    if(abs((int16_t)cfg.angleTrim[0]) > Trimlimit || abs((int16_t)cfg.angleTrim[1]) > Trimlimit) return; // We may miss Trimlimit a little but no reason to go anal here
    blinkLED(15, 20, 1);
    ScheduleEEPROMwriteMS = millis() + 5000;
#ifdef LEDRING
    if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
}

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;
    for (r = 0; r < repeat; r++)
    {
        for (i = 0; i < num; i++)
        {
            LED0_TOGGLE                                                     // switch LEDPIN state
            systemBeep(true);
            delay(wait);
            systemBeep(false);
        }
        delay(60);
    }
}

static void MWCRGBMONO_LED(uint8_t buzz)
{
    if (cfg.LED_Type == 1)                                                  // MWCRGB
    {
        if (failsafeCnt > 5) LED_Value = 2100;
        else if (buzz > 3) LED_Value = 2200;                                // Changed http://fpv-treff.de/viewtopic.php?f=18&t=1368&start=1660#p33083
        else
        {
            if ((cfg.LED_Armed == 1) || f.ARMED)
            {
                if (rcData[cfg.LED_ControlChannel - 1] < 1300) LED_Value = cfg.LED_Pattern1;
                else if ((rcData[cfg.LED_ControlChannel - 1] >= 1300) && (rcData[cfg.LED_ControlChannel - 1] <= 1700)) LED_Value = cfg.LED_Pattern2;
                else LED_Value = cfg.LED_Pattern3;
            }
            else LED_Value = 1000;
        }
    }
    if (cfg.LED_Type == 2)                                                  // MONO_LED
    {
        if (failsafeCnt > 5)                                                // Update the leds every frame, must also be called if leds are disabled now
        {
            LED_Value = 2100;
            LED_Value_Delay = 10;
        }
        else if (buzz > 3)
        {
            LED_Value = 2200;
            LED_Value_Delay = 10;
        }
        else
        {
            if ((cfg.LED_Armed == 1) || f.ARMED)
            {
                if (rcData[cfg.LED_ControlChannel - 1] < 1300)
                {
                    LED_Value = cfg.LED_Pattern1;
                    LED_Value_Delay = cfg.LED_Toggle_Delay1;
                }
                else if ((rcData[cfg.LED_ControlChannel - 1] >= 1300) && (rcData[cfg.LED_ControlChannel - 1] <= 1700))
                {
                    LED_Value = cfg.LED_Pattern2;
                    LED_Value_Delay = cfg.LED_Toggle_Delay2;
                }
                else
                {
                    LED_Value = cfg.LED_Pattern3;
                    LED_Value_Delay = cfg.LED_Toggle_Delay3;
                }
            }
            else
            {
                LED_Value = 1000;
                LED_Value_Delay=10;
            }
        }
        ledToggleUpdate(true);
    }
    else ledToggleUpdate(false);
}
