#pragma once
#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/* for VBAT monitoring frequency */
#define VBATFREQ 6                          // to read battery voltage - nth number of loop iterations

#define  VERSION  211
#define  FIRMWARE  "HarakiriSG pre2.6 TestCode3" __DATE__ " / " __TIME__
#define  FIRMWAREFORLCD "Harakiri 10"

#define LAT  0
#define LON  1
#define GPS_Y 0
#define GPS_X 1

// RC & EXPO
#define RcTrimstep 2.0f
#define Trimlimit  300                      // Trimlimit in DEG*10

typedef struct stdev_t
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

// Serial GPS only variables
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_RTL,
    NAV_MODE_WP,
    NAV_MODE_CIRCLE
} NavigationMode;

typedef enum WPstatus
{
    WP_STATUS_NONE = 0,
    WP_STATUS_NAVIGATING,
    WP_STATUS_DONE
} WPstatus;

typedef enum PHstatus
{
    PH_STATUS_NONE = 0,
    PH_STATUS_WORKING,
    PH_STATUS_DONE
} PHstatus;

typedef enum Protocol
{
    PROTOCOL_AUTOSENSE = 0,
    PROTOCOL_MWII21,
    PROTOCOL_MAVLINK
} Protocol;

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
    ML_NAV_TAKEOFF,
    ML_NAV_SETROI
} MavlnkCommand;

/*
OLD
// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6 = 7,
    MULTITYPE_FLYING_WING = 8,
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,                  // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP = 12,               // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX = 13,               // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE = 14,                // airplane / singlecopter / dualcopter (not yet properly supported)
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_CUSTOM = 18,                  // no current GUI displays this
    MULTITYPE_LAST = 19
} MultiType;
*/

// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI           =  1,
    MULTITYPE_QUADP         =  2,
    MULTITYPE_QUADX         =  3,
    MULTITYPE_BI            =  4,
    MULTITYPE_GIMBAL        =  5,
    MULTITYPE_Y6            =  6,
    MULTITYPE_HEX6          =  7,
    MULTITYPE_FLYING_WING   =  8,
    MULTITYPE_Y4            =  9,
    MULTITYPE_HEX6X         = 10,
    MULTITYPE_OCTOX8        = 11,   // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP     = 12,   // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX     = 13,   // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE      = 14,   // airplane / singlecopter / dualcopter (not yet properly supported)
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG   = 16,
    MULTITYPE_VTAIL4        = 17,
    MULTITYPE_HEX6H         = 18,
    MULTITYPE_PPM_TO_SERVO  = 19,   // PPM -> servo relay 
    MULTITYPE_CUSTOM        = 20,   // no current GUI displays this
    MULTITYPE_LAST          = 21
} MultiType;

typedef enum GimbalFlags
{
    GIMBAL_NORMAL       = 1 << 0,
    GIMBAL_TILTONLY     = 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
    GIMBAL_FORWARDAUX   = 1 << 3,
    GIMBAL_MIXTILT      = 1 << 4,
} GimbalFlags;

/*********** RC alias *****************/
enum
{
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6
};

enum
{
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PIDITEMS
};

enum   // This is limited to 32 Boxes!!
{
    BOXANGLE = 0,
    BOXHORIZON,
    BOXBARO,
    BOXMAG,
    BOXCAMSTAB,
    BOXARM,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXGPSAUTO,
    BOXPASSTHRU,
    BOXHEADFREE,
    BOXBEEPERON,
    BOXHEADADJ,
    BOXOSD,
    BOXGTUNE,
    CHECKBOXITEMS
};

static const char boxnames[] =
    "ANGLE;"
    "HORIZON;"
    "BARO;"
    "MAG;"
    "CAMSTAB;"
    "ARM;"
    "GPS HOME;"
    "GPS HOLD;"
    "GPS AUTO;"
    "PASSTHRU;"
    "HEADFREE;"
    "BEEPER;"
    "HEADADJ;"
    "OSD SW;"
    "GTUNE;";

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef struct motorMixer_t
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct mixer_t
{
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

enum
{
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

typedef struct config_t
{
    uint8_t  version;
    uint16_t size;
    uint8_t  magic_be;                      // magic number, should be 0xBE
    uint8_t  mixerConfiguration;
    uint32_t enabledFeatures;
    uint16_t looptime;                      // imu loop time in us
    uint8_t  mainpidctrl;                   // 0 = OriginalMwiiPid pimped by me, 1 = New mwii controller (experimental, float pimped + pt1)
    uint8_t  maincuthz;                     // (1-50Hz) 0 Disables pt1element. Cuf Off Frequency for pt2 element D term in Hz of main Pid controller
    uint8_t  gpscuthz;                      // (1-50Hz) Cuf Off Frequency for D term in Hz of GPS Pid controller 
    uint8_t  P8[PIDITEMS];
    uint8_t  I8[PIDITEMS];
    uint8_t  D8[PIDITEMS];
    uint8_t  rcRate8;
    uint8_t  rcExpo8;
    uint8_t  thrMid8;
    uint8_t  thrExpo8;
    uint8_t  rollPitchRate;
    uint8_t  yawRate;
    uint8_t  dynThrPID;
    uint8_t  ShakyDataAvail;                // Is set to 1 when gyrodata are saved upon acc calibration as well.
    float    ShakyGyroZero[3];              // Saved together with acc calibration and will be used in places that are troublesome (boats etc.)
    float    accZero[3];
    int16_t  sens_1G;
    float    magZero[3];
    uint8_t  acc_calibrated;
    uint8_t  mag_calibrated;                // Just to supress crazymag in gui display
    int16_t  mag_dec;                       // Get your magnetic decliniation from here : http://magnetic-declination.com/
    uint8_t  mag_time;                      // Mag calib time in minutes
    uint8_t  mag_gain;                      // 0(default) = 1.9 GAUSS ; 1 = 2.5 GAUSS (problematic copters)
    float    angleTrim[2];                  // trim
    // sensor-related stuff
    int8_t   align[3][3];                   // acc, gyro, mag alignment (ex: with sensor output of X, Y, Z, align of 1 -3 2 would return X, -Z, Y)
    uint8_t  acc_hdw;                       // Which acc hardware to use on boards with more than one device
    float    acc_lpfhz;                     // Set the Low Pass Filter factor for ACC in Hz.
    uint16_t gy_lpf;                        // mpuX050 LPF setting (TODO make it work on L3GD as well)
    uint16_t gy_gcmpf;                      // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint16_t gy_mcmpf;
    uint8_t  gy_smrll;
    uint8_t  gy_smptc;
    uint8_t  gy_smyw;                       // In Tricopter mode a "1" will enable a moving average filter, anything higher will also enable a lowpassfilter
    float    accz_vcf;                      // Crashpilot: Value for complementary filter accz and barovelocity
    float    accz_acf;                      // Crashpilot: Value for complementary filter accz and altitude
    float    bar_lag;                       // Lag of Baro
    float    bar_dscl;                      // Scale downmovement down
    uint8_t  bar_dbg;                       // Crashpilot: 1 = Debug Barovalues
    uint8_t  gy_stdev;                      // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.

    uint32_t activate[CHECKBOXITEMS];       // activate switches
    uint8_t  vbatscale;                     // adjust this to match battery voltage to reported value
    uint8_t  vbatmaxcellvoltage;            // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t  vbatmincellvoltage;            // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)
    uint8_t  power_adc_channel;             // which channel is used for current sensor. Right now, only 2 places are supported: RC_CH2 (unused when in CPPM mode, = 1), RC_CH8 (last channel in PWM mode, = 9)

    // Radio/ESC-related configuration
    uint8_t  rcmap[MAX_RC_CHANNELS];        // uint8_t rcmap[8]; // mapping of radio channels to internal RPYTA+ order
    uint8_t  rc_auxch;                      // cGiesen: the number of supported aux channels. default = 4
    uint8_t  rc_db;                         // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t  rc_dbyw;                       // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t  rc_dbah;                       // defines the neutral zone of throttle stick during altitude hold, default setting is +/-20
    uint8_t  rc_dbgps;                      // Additional Deadband for all GPS functions;
    uint8_t  devorssi;                      // Will take the last channel for RSSI value, so add one to rc_auxch, don't use that auxchannel unless you want it to trigger something
    uint8_t  rssicut;                       // [0-80%] Below that percentage rssi will show zero.
    uint8_t  spektrum_hires;                // spektrum high-resolution y/n (1024/2048bit)
    uint16_t rc_minchk;                     // minimum rc end
    uint16_t rc_mid;                        // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t rc_maxchk;                     // maximum rc end
    uint8_t  rc_lowlat;                     // 1 = low latency, 0 = normal latency
    uint8_t  rc_rllrm;                      // allow disarsm/arm on throttle down + roll left/right
    uint16_t rc_killt;                      // Time in ms when your arm switch becomes a Killswitch, 0 disables
    uint8_t  rc_flpsp;                      // [0-1] When enabled(1) and upside down in acro or horizon mode only half throttle is applied
    uint8_t  rc_motor;                      // [0-2] Behaviour when thr < rc_minchk: 0= minthrottle no regulation, 1= minthrottle&regulation, 2= Motorstop 

    // G-tune related configuration
    uint8_t  gt_lolimP[3];                  // [10..200] Lower limit of P during G tune
    uint8_t  gt_hilimP[3];                  // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
    int8_t   gt_threP;                      // [1..50] Threshold for P during G tune

    // Failsafe related configuration
    uint8_t  fs_delay;                      // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t  fs_ofdel;                      // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint16_t fs_rcthr;                      // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint8_t  fs_ddplt;		                  // Time in sec when FS is engaged after idle on THR/YAW/ROLL/PITCH
    uint8_t  fs_jstph;                      // Does just PH&Autoland an not RTL,
    uint8_t  fs_nosnr;                      // When snr_land is set to 1, it is possible to ignore that on Failsafe, because FS over a tree could turn off copter

    // motor/esc/servo related stuff
    uint16_t esc_min;                       // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t esc_max;                       // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t esc_moff;                      // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t esc_nfly;                      // This is the absolute throttle that kicks off the "has landed timer" if it is too low cfg.rc_minchk is taken.
    uint8_t  esc_nwmx;                      // 0 = mwii style, 1 = scaled handling of maxthrottlesituations
    uint16_t esc_pwm;                       // The update rate of motor outputs (50-498Hz)
    uint16_t srv_pwm;                       // The update rate of servo outputs (50-498Hz)
    uint8_t  pass_mot;                      // Crashpilot: Only used with feature pass. If 0 = all Motors, otherwise specific Motor
    int16_t  servotrim[8];                  // Adjust Servo MID Offset & Swash angles
    int8_t   servoreverse[8];               // Invert servos by setting -1

    // mixer-related configuration
    uint16_t tri_ydel;                      // Tri Yaw Arm delay: Time in ms after wich the yaw servo after arming will engage. 0 disables Yawservo always active.
    int8_t   tri_ydir;
    uint16_t tri_ymid;                      // tail servo center pos. - use this for initial trim
    uint16_t tri_ymin;                      // tail servo min
    uint16_t tri_ymax;                      // tail servo max

    // flying wing related configuration
    uint16_t wing_left_min;                 // min/mid/max servo travel
    uint16_t wing_left_mid;
    uint16_t wing_left_max;
    uint16_t wing_right_min;
    uint16_t wing_right_mid;
    uint16_t wing_right_max;
    int8_t   pitch_direction_l;             // left servo - pitch orientation
    int8_t   pitch_direction_r;             // right servo - pitch orientation (opposite sign to pitch_direction_l if servos are mounted mirrored)
    int8_t   roll_direction_l;              // left servo - roll orientation
    int8_t   roll_direction_r;              // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)

    // gimbal-related configuration
    int8_t   gbl_pgn;                       // gimbal pitch servo gain (tied to angle) can be negative to invert movement
    int8_t   gbl_rgn;                       // gimbal roll servo gain (tied to angle) can be negative to invert movement
    uint8_t  gbl_flg;                       // in servotilt mode, various things that affect stuff
    uint16_t gbl_pmn;                       // gimbal pitch servo min travel
    uint16_t gbl_pmx;                       // gimbal pitch servo max travel
    uint16_t gbl_pmd;                       // gimbal pitch servo neutral value
    uint16_t gbl_rmn;                       // gimbal roll servo min travel
    uint16_t gbl_rmx;                       // gimbal roll servo max travel
    uint16_t gbl_rmd;                       // gimbal roll servo neutral value

    // Autoland
    uint8_t  al_barolr;                     // [10 - 200cm/s] Baro Landingrate
    uint8_t  al_snrlr;                      // [10 - 200cm/s] Sonar Landingrate - You can specify different landingfactor here on sonar contact, because sonar land maybe too fast when snr_cf is high
    uint8_t  al_debounce;                   // (0-20%) 0 Disables. Defines a Throttlelimiter on Autoland. This percentage defines the maximum deviation of assumed hoverthrottle during Autoland
    uint16_t al_tobaro;                     // Timeout in ms (100 - 5000) before shutoff on autoland. "esc_nfly" must be undershot for that timeperiod
    uint16_t al_tosnr;                      // Timeout in ms (100 - 5000) If sonar aided land is wanted (snr_land = 1) you can choose a different timeout here
    uint8_t  as_lnchr;                      // [5 - 20 no dimension DEFAULT:10] This is the std. deviation of the variometer when a liftoff is assumed. The higher the more unsensitive.
    uint8_t  as_clmbr;                      // [50 - 250cm/s] Autostart Rate in cm/s
    uint8_t  as_trgt;                       // [0 - 255m  DEFAULT:0 (0 = Disable)] Autostart Targethight in m Note: use 2m or more
    uint8_t  as_stdev;                      // [10 - 50 no dimension] This is the std. standard deviation of the variometer when a liftoff is assumed.

    // gps-related stuff
    uint8_t  gps_type;                      // Type of GPS hardware. 0: NMEA 1: UBX 2+ ??
    float    gps_ins_vel;                   // Crashpilot: Value for complementary filter INS and GPS Velocity
    uint16_t gps_lag;                       // GPS Lag in ms
    uint8_t  acc_altlpfhz;                  // LPF in HZ just for althold
    uint8_t  acc_gpslpfhz;                  // LPF in HZ just for GPS ins stuff
    uint8_t  gps_ph_minsat;                 // Minimal Satcount for PH, PH on RTL is still done with 5Sats or more
    uint8_t  gps_expo;                      // 1 - 99 % defines the actual Expo applied for GPS
    uint8_t  gps_ph_settlespeed;            // PH settlespeed in cm/s
    uint8_t  gps_maxangle;                  // maximal over all GPS bank angle
    uint8_t  gps_ph_brakemaxangle;          // Maximal 5 Degree Overspeedbrake
    uint8_t  gps_ph_minbrakepercent;        // 1-99% minimal percent of "brakemaxangle" left over for braking. Example brakemaxangle = 6 so 50 Percent is 3..
    uint16_t gps_ph_brkacc;                 // [50-500] Is the assumed negative braking acceleration in cm/(s*s) of copter. Value is positive though
    uint32_t gps_baudrate;                  // GPS baudrate
    uint16_t gps_wp_radius;                 // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint8_t  rtl_mnd;                       // Minimal distance for RTL, otherwise it will just autoland, prevent Failsafe jump in your face, when arming copter and turning off TX
    uint8_t  gps_rtl_flyaway;               // If during RTL the distance increases beyond this valus in meters, something is wrong, autoland
    uint8_t  gps_yaw;                       // Thats the MAG P during GPS functions, substitute for "cfg.P8[PIDMAG]"
    uint8_t  nav_tail_first;                // 1 = Copter comes back with ass first (only works with nav_controls_heading = 1)
    uint8_t  nav_controls_heading;          // copter faces toward the navigation point, maghold must be enabled for it
    uint8_t  nav_rtl_lastturn;              // Something like NAV_SET_TAKEOFF_HEADING on mwii
    uint8_t  nav_speed_min;                 // 10 - 200 cm/s don't set higher than nav_speed_max! That dumbness is not covered.
    uint16_t nav_speed_max;                 // 50 - 2000 cm/s don't set lower than nav_speed_min! That dumbness is not covered.
    uint8_t  nav_approachdiv;               // 2-10 This is the divisor for approach speed for wp_distance. Example: 400cm / 3 = 133cm/s if below nav_speed_min it will be adjusted
    uint8_t  nav_tiltcomp;                  // 0-100 Only arducopter really knows. This is some kind of a hack of them to reach actual nav_speed_max. 54 was default, 0 disables
    float    nav_ctrkgain;                  // 0 - 10.0 (Floatvariable) That is the "Crosstrackgain" APM default is "1". "0" disables
    uint8_t  rtl_mnh;                       // Minimal RTL hight in m, 0 disable  // Crashpilot
    uint8_t  rtl_cr;                        // [10 - 200cm/s] When rtl_minh is defined this is the climbrate in cm/s

    uint32_t serial_baudrate;               // serial(uart1) baudrate
    uint8_t  tele_prot;                     // Protocol ONLY used when Armed including Baudchange if necessary. 0 (Dfault)=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)

    // LED Stuff
    uint8_t  LED_invert;                    // Crashpilot invert LED 0&1
    uint8_t  LED_Type;                      // 1=MWCRGB / 2=MONO_LED / 3=LEDRing
    uint8_t  LED_Pinout;                    // choose LED pinout (MONO_LED: 0=LED rc5, 1=LED rc6 / MWCRGB: coming soon)
    uint8_t  LED_ControlChannel;            // RC Channel to control the LED Pattern
    uint8_t  LED_Armed;          		        // 0 = Show LED only if armed, 1 = always show LED
    uint8_t  LED_Toggle_Delay1;             // 16bit bit pattern to slow down led patterns
    uint8_t  LED_Toggle_Delay2;             // 16bit bit pattern to slow down led patterns
    uint8_t  LED_Toggle_Delay3;             // 16bit bit pattern to slow down led patterns
    uint32_t LED_Pattern1;            	  	// 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    uint32_t LED_Pattern2;            		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    uint32_t LED_Pattern3;            		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000

    // Sonar Stuff
    uint8_t  snr_type;                      // 0 = PWM56, 1 = RC78, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4=MBRC78
    uint8_t  snr_min;                       // Valid Sonar minimal range in cm (0-200)
    uint16_t snr_max;                       // Valid Sonar maximal range in cm (0-700)
    uint8_t  snr_dbg;                       // 1 Sets Sonardata within snr_min/max in debug[0]
    uint8_t  snr_tilt;                      // Somehow copter tiltrange in degrees (not exactly) in wich Sonar is possible
    float    snr_cf;                        // The bigger, the more Sonarinfluence
    uint8_t  snr_diff;                      // Maximal allowed difference in cm between sonar readouts (100ms rate and maxdiff = 50 means max 5m/s)
    uint8_t  snr_land;                      // This helps Sonar when landing, by setting upper throttle limit to current throttle. - Beware of Trees!!   
    motorMixer_t customMixer[MAX_MOTORS];   // custom mixtable

    // LOGGING
    uint8_t  stat_clear;                    // This will clear the stats between flights, or you can set to 0 and treasue overallstats
    uint16_t GPS_MaxDistToHome;             // Treasure Maximal distance from home for later Statistic chart
    uint16_t MAXGPSspeed;                   // Maxspeed in cm/s
    int16_t  MaxAltMeter;
    int16_t  MinAltMeter;
    uint16_t FDUsedDatasets;                // Number of valid datasets of current type
    uint8_t  FloppyDisk[FDByteSize];        // Reserve ca. 2200 general purpose Bytes
    uint8_t  magic_ef;                      // magic number, should be 0xEF
    uint8_t  chk;                           // XOR checksum
} config_t;

typedef struct flags_t
{
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t GPS_AUTO_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLES_25;
    uint8_t GTUNE;
} flags_t;

extern bool     SerialRCRX;   // MAIN

extern float    gyroData[3];
extern float    angle[2];
extern float    axisPID[3];
extern float    AvgCyclTime;

extern int16_t  rcCommand[4];
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern volatile uint16_t failsafeCnt;
extern float    TiltValue;

extern int16_t  debug[4];
extern float    accSmooth[3];
extern float    accADC[3], gyroADC[3], magADCfloat[3];
extern uint32_t currentTime;
extern uint32_t currentTimeMS;
extern uint32_t previousTime;
extern bool     calibratingA;
extern bool     calibratingM;

extern float    BaroAlt;
extern int32_t  sonarAlt;
extern float    EstAlt;
extern float    AltHold;
extern float    vario;
extern float    BaroActualTemp;
extern int16_t  BaroP;
extern int16_t  BaroI;
extern int16_t  BaroD;
extern bool     newbaroalt;
extern bool     GroundAltInitialized;

extern int16_t  motor[MAX_MOTORS];
extern int16_t  servo[8];
extern int16_t  rcData[MAX_RC_CHANNELS];
extern int16_t  rcDataSAVE[MAX_RC_CHANNELS];
extern uint8_t  rssi;                       // 0 - 255 = 0%-100%
extern uint8_t  vbat;
extern float    telemTemperature1;          // gyro sensor temperature

extern uint8_t  motorpercent[MAX_MONITORED_MOTORS];
extern uint16_t motorabspwm[MAX_MONITORED_MOTORS];
extern uint8_t  NumberOfMotors;             // Store number of Motors used

// IMU
extern float    ACC_speed[2];
extern float    ACCDeltaTimeINS;

// Sensors
extern float    MainDptCut;
extern bool     MpuSpecial;
extern float    magCal[3];
extern bool     maggainok;
extern bool     HaveNewMag;

// IMU / MAG etc.
extern float    headFreeModeHold;
extern float    heading;
extern float    magHold;
extern float    magneticDeclination;
extern bool     UpsideDown;

// SONAR /BARO / AUTOLAND
extern uint8_t  SonarStatus;                // 0 = no contact, 1 = made contact, 2 = steady contact
extern uint8_t  SonarBreach;                // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
extern uint8_t  SonarLandWanted;            // This is done to virtualize cfg.snr_land because failsafe can disable it and it could be saved in the worst case
extern float    ActualPressure;
extern float    GroundAlt;
extern int16_t  ESCnoFlyThrottle;
extern bool     ForceRCExpInit;

// GPS stuff
extern volatile int32_t  IRQGPS_coord[2];   // They occure serial IRQ is done, and they are fed to Real_GPS_coord synchronized, so no uneval. rawdata pop up suddenly
extern volatile uint16_t IRQGPS_speed;
extern volatile uint16_t IRQGPS_grcrs;
extern volatile bool GPS_FIX;
extern int32_t  Real_GPS_coord[2];          // RAW GPS Coords
extern int32_t  GPS_home[2];
extern int32_t  GPS_WP[2];                  // Currently used WP
extern volatile uint8_t GPS_numSat;
extern uint32_t GPS_distanceToHome;         // distance to home
extern int32_t  GPS_directionToHome;        // direction to home
extern uint16_t GPS_speed;                  // speed in cm/s
extern volatile uint16_t GPS_altitude;      // altitude in m
extern uint8_t  GPS_update;                 // it's a binary toogle to distinct a GPS position update
extern float    GPS_angle[2];               // it's the angles that must be applied for GPS correction
extern float    Last_GPS_angle[2];
extern uint16_t GPS_ground_course;          // degrees*10
extern float    nav[2];
extern int8_t   nav_mode;                   // Navigation mode
extern int8_t   wp_status;
extern int8_t   ph_status;
extern int16_t  maxbank10;                  // Maximum GPS Tiltangle in degree * 10
extern int16_t  maxbank100;                 // Maximum GPS Tiltangle in degree * 100
extern float    GPSEXPO;
extern int32_t  WP_Target_Alt;
extern int16_t  WP_Desired_Climbrate;
extern bool     WP_Fastcorner;              // Dont decrease Speed at Target
extern float    sin_yaw_y;
extern float    cos_yaw_x;
extern bool     BlockGPSAngles;
extern bool     BlockThrottle;
extern bool     BlockPitch;
extern bool     BlockRoll;
extern volatile uint32_t TimestampNewGPSdata; // Crashpilot in micros
extern int32_t  target_bearing;             // target_bearing is where we should be heading
extern uint32_t wp_distance;
extern float    waypoint_speed_gov;         // used for slow speed wind up when start navigation;
extern int32_t  nav_bearing;                // This is the angle from the copter to the "next_WP" location  with the addition of Crosstrack error in degrees * 100 // Crosstrack eliminated left here on purpose
extern int16_t  nav_takeoff_heading;        // saves the heading at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
extern int32_t  original_target_bearing;    // deg*100, The original angle to the next_WP when the next_WP was set Also used to check when we pass a WP
extern float    LocError[2];                // updated after GPS read - 5-10hz Error in cm from target

// General
extern config_t cfg;
extern flags_t  f;
extern sensor_t acc;
extern sensor_t gyro;
extern baro_t   baro;
extern uint8_t  Currentprotocol;
extern uint32_t ScheduleEEPROMwriteMS;

// Serial
extern bool AllowProtocolAutosense;

// MWCRGB
extern uint32_t LED_Value;
extern uint32_t LED_Value;
extern uint8_t  LED_Value_Delay;

// Main
void     loop(void);
void     pass(void);
void     LD0_OFF(void);                         // Crashpilot LED Inverter stuff
void     LD1_OFF(void);
void     LD0_ON(void);
void     LD1_ON(void);
void     devClear(stdev_t *dev);
void     devPush(stdev_t *dev, float x);
float    devStandardDeviation(stdev_t *dev);

// IMU
void     imuInit(void);
void     computeIMU(void);
void     blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
void     getEstimatedAltitude(void);
void     getAltitudePID(void);

// Sensors
void     SensorDetectAndINI(void);
void     batteryInit(void);
uint16_t batteryAdcToVoltage(uint16_t src);
void     ACC_getADC(void);
void     alignSensors(uint8_t type, int16_t *data);
void     Baro_update(void);
void     Gyro_getADC(void);
void     Mag_getADC(void);
void     Sonar_update(void);
void     MPU6050ReadAllShit(int16_t *accData, float *tempData, int16_t *gyroData);
void     GETMPU6050(void);

// Output
void     mixerInit(void);
void     mixerLoadMix(int index);
void     writeServos(void);
void     writeMotors(void);
void     writeAllMotors(int16_t mc);
void     mixTable(void);

// Serial Mwii & CLI & Mavlink
void     serialInit(uint32_t baudrate);
void     serialCom(bool singlestep);
void     serialOSD(void);
void     cliSave(char *cmdline);
void     baseflight_mavlink_init(void);

// Config
void     parseRcChannels(const char *input);
void     readEEPROM(void);
void     writeParams(uint8_t b);
void     checkFirstTime(bool reset);
bool     sensors(uint32_t mask);
void     sensorsSet(uint32_t mask);
void     sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
bool     feature(uint32_t mask);
void     featureSet(uint32_t mask);
void     featureClear(uint32_t mask);
void     featureClearAll(void);
uint32_t featureMask(void);
void     ClearStats(void);

// General RC stuff
bool     DoGetRc50HzTimer(void);

// buzzer
void     buzzer(uint8_t warn_vbat);

// cli
void     cliProcess(void);
void     LCDoff(void);

// gps
void     DoChkGPSDeadin50HzLoop(void);
void     GPS_set_pids(void);
void     GPS_reset_nav(void);
bool     GPS_alltime(void);
void     GPS_calc_longitude_scaling(bool force);
void     GPS_set_next_wp(int32_t *lat, int32_t *lon);
void     GPS_calc_velocity(void);
void     GPS_calc_posholdCrashpilot(bool overspeed);
void     GPS_calc_location_error(int32_t *TrgtLAT, int32_t *TrgtLON);
void     GPS_distance_cm_bearing(int32_t *TrgtLAT, int32_t *TrgtLON, uint32_t *dist, int32_t *bearing);

// Currently not used int32_t  GPS_TargetBearing(int32_t *TrgtLAT, int32_t *TrgtLON);

uint16_t GPS_calc_desired_speed(void);
void     GPS_calc_nav_rate(uint16_t max_speed);
bool     check_missed_wp(void);
bool     DoingGPS(void);
float    wrap_180(float value);

// floppy
void     FloppyClear(void);
bool     FloppyWriteWP (uint16_t *Nr, wp_t *wppnt);
bool     FloppyReadWP  (uint16_t *Nr, wp_t *wppnt);

// telemetry
void     initFRSKYTelemetry(bool State);
void     initMinimOSDTelemetry(bool State);
void     sendFRSKYTelemetry(void);

// Init the led gpio port when enabled
void     ledToggleInit(void);

// Update the leds, enabled signals that the leds are enabled
void     ledToggleUpdate(bool activated);
