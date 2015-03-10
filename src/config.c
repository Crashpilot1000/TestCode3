#include "board.h"
#include "mw.h"
#include <string.h>

config_t cfg;
const char rcChannelLetters[] = "AERT1234";

static uint8_t  EEPROM_CONF_VERSION = 34;
static uint32_t enabledSensors      = 0;
static void resetConf(void);

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s) cfg.rcmap[s - rcChannelLetters] = c - input;
    }
}

static bool validEEPROM(void)
{
    const config_t *temp = (const config_t *)FLASH_WRITE_ADDR;
    const uint8_t  *p;
    uint8_t        chk = 0;
    if (EEPROM_CONF_VERSION != temp->version) return false;                               // check version number
    if (temp->size != sizeof(config_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF) return false; // check size and magic numbers
    for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(config_t)); p++) chk ^= *p;   // verify integrity of temporary copy
    if (chk) return false;                                                                // checksum failed
    return true;                                                                          // looks good, let's roll!
}

void readEEPROM(void)
{
    memcpy(&cfg, (char *)FLASH_WRITE_ADDR, sizeof(config_t));                             // Read flash
    ForceRCExpInit = true;                                                                // Trigger ini of rc lists
    cfg.tri_ymid = constrain_int(cfg.tri_ymid, cfg.tri_ymin, cfg.tri_ymax);               // REAR
    GPS_set_pids();                                                                       // Set GPS PIDS in any case
    GPS_reset_nav();
}

void writeParams(uint8_t b)
{
    FLASH_Status status;
    uint32_t i;
    uint8_t chk = 0;
    const uint8_t *p;

    cfg.version    = EEPROM_CONF_VERSION;
    cfg.size       = sizeof(config_t);
    cfg.magic_be   = 0xBE;
    cfg.magic_ef   = 0xEF;
    cfg.chk        = 0;
    for (p = (const uint8_t *)&cfg; p < ((const uint8_t *)&cfg + sizeof(config_t)); p++) chk ^= *p; // recalculate checksum before writing
    cfg.chk = chk;
    FLASH_Unlock();                                                                       // write it
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  
    for (i = 0; i < FLASH_PAGES_FORCONFIG; i++)                                           // Erase the pages here
    {
        while (FLASH_ErasePage(FLASH_WRITE_ADDR + (i * FLASH_PAGE_SIZE)) != FLASH_COMPLETE);
    }

    for (i = 0; i < sizeof(config_t); i += 4)                                             // Write that config now.
    {
        status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *) &cfg + i));
        if (status != FLASH_COMPLETE) break;                                              // TODO: fail
    }
    FLASH_Lock();
    readEEPROM();
    if (b) blinkLED(15, 20, 1);
}

void checkFirstTime(bool reset)
{
    if ((!validEEPROM() || reset) && !f.ARMED) resetConf();                               // check the EEPROM integrity
}

// Default settings
static void resetConf(void)
{
    uint8_t i;
    const int8_t default_align[3][3] = { /* GYRO */ { 0, 0, 0 }, /* ACC */ { 0, 0, 0 }, /* MAG */ { 0, 0, 0 } };
    memset(&cfg, 0, sizeof(config_t));

    cfg.version = EEPROM_CONF_VERSION;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    featureClearAll();
//    featureSet(FEATURE_VBAT);
    featureSet(FEATURE_PPM);
//    featureSet(FEATURE_FAILSAFE);
//    featureSet(FEATURE_LCD);
//    featureSet(FEATURE_GPS);
//    featureSet(FEATURE_PASS);                   // Just pass Throttlechannel
//    featureSet(FEATURE_SONAR);

    cfg.P8[ROLL]                  =  35;        // 40
    cfg.I8[ROLL]                  =  30;
    cfg.D8[ROLL]                  =  30;

    cfg.P8[PITCH]                 =  35;        // 40
    cfg.I8[PITCH]                 =  30;
    cfg.D8[PITCH]                 =  30;

    cfg.P8[YAW]                   =  60;        // 70
    cfg.I8[YAW]                   =  45;

    cfg.P8[PIDALT]                = 100;
    cfg.I8[PIDALT]                =  30;
    cfg.D8[PIDALT]                =  80;

    cfg.P8[PIDPOS]                =  10;        // FIND YOUR VALUE
    cfg.I8[PIDPOS]                =  40;        // USED

    cfg.P8[PIDPOSR]               =  70;        // FIND YOUR VALUE                    // Controls the speed part with my PH logic
    cfg.D8[PIDPOSR]               = 100;        // FIND YOUR VALUE                    // Controls the speed part with my PH logic

    cfg.P8[PIDNAVR]               =  15;        // 14 More ?
    cfg.I8[PIDNAVR]               =   0;        // NAV_I * 100;                       // Scaling/Purpose unchanged
    cfg.D8[PIDNAVR]               =   0;        // NAV_D * 1000;                      // Scaling/Purpose unchanged

//    cfg.P8[PIDPOS]                = 11;         // APM PH Stock values
//    cfg.I8[PIDPOS]                = 0;
//    cfg.D8[PIDPOS]                = 0;

//    cfg.P8[PIDPOSR]               = 20;         // POSHOLD_RATE_P * 10;
//    cfg.I8[PIDPOSR]               = 8;          // POSHOLD_RATE_I * 100;
//    cfg.D8[PIDPOSR]               = 45;         // POSHOLD_RATE_D * 1000;

//    cfg.P8[PIDNAVR]               = 14;         // NAV_P * 10;
//    cfg.I8[PIDNAVR]               = 20;         // NAV_I * 100;
//    cfg.D8[PIDNAVR]               = 80;         // NAV_D * 1000;

    cfg.P8[PIDLEVEL]              = 70;         // 70
    cfg.I8[PIDLEVEL]              = 10;
    cfg.D8[PIDLEVEL]              = 50;

    cfg.P8[PIDMAG]                = 80;         // cfg.P8[PIDVEL] = 0;// cfg.I8[PIDVEL] = 0;// cfg.D8[PIDVEL] = 0;

    cfg.rcRate8                   = 100;
    cfg.rcExpo8                   = 80;         // cfg.rollPitchRate = 0;// cfg.yawRate = 0;// cfg.dynThrPID = 0;
    cfg.thrMid8                   = 50;

    memcpy(&cfg.align, default_align, sizeof(cfg.align));
    cfg.align_board_yaw           = 0;          // 0 = 0 Deg. 1 = 90 Deg. 2 = 180 Deg. 3 = 270 Deg Clockwise
    cfg.mag_dec                   = 113;        // Crashpilot //cfg.acc_hdw = ACC_DEFAULT;// default/autodetect
    cfg.mag_time                  = 1;          // (1-6) Calibration time in minutes
    cfg.mag_gain                  = 0;          // 0(default) = 1.9 GAUSS ; 1 = 2.5 GAUSS (problematic copters, will reduce 20% resolution)
    cfg.acc_hdw                   = 2;          // Crashpilot MPU6050
    cfg.acc_lpfhz                 = 10.0f;      // [0.x-100Hz] LPF for angle/horizon 0.536f resembles somehow the orig mwii factor
    cfg.acc_altlpfhz              = 10;         // [1-100Hz]   LPF for althold
    cfg.acc_gpslpfhz              = 15;         // [1-100Hz]   LPF for GPS ins stuff
    cfg.looptime                  = 3000;
    cfg.mainpidctrl               = 0;          // 0 = OriginalMwiiPid pimped by me, 1 = New mwii controller (experimental, float pimped + pt1)
    cfg.maincuthz                 = 12;         // [1-100Hz] Cuf Off Frequency for D term of main Pid controller
    cfg.gpscuthz                  = 45;         // [1-100Hz] Cuf Off Frequency for D term of GPS Pid controller 

    cfg.gy_gcmpf                  = 700;        // (10-1000) 400 default. Now 1000. The higher, the more weight gets the gyro and the lower is the correction with Acc data.
    cfg.gy_mcmpf                  = 200;        // (10-2000) 200 default for 10Hz. Now higher. Gyro/Magnetometer Complement.
    cfg.gy_smrll                  = 0;
    cfg.gy_smptc                  = 0;
    cfg.gy_smyw                   = 0;          // In Tricopter mode a "1" will enable a moving average filter, anything higher will also enable a lowpassfilter
    cfg.gy_lpf                    = 42;         // Values for MPU 6050/3050: 256, 188, 98, 42, 20, 10, (HZ) For L3G4200D: 93, 78, 54, 32
    cfg.gy_stdev                  = 5;

    // Baro
    cfg.accz_vcf                  = 0.985f;     // Crashpilot: Value for complementary filter accz and barovelocity
    cfg.accz_acf                  = 0.960f;     // Crashpilot: Value for complementary filter accz and altitude
    cfg.bar_lag                   = 0.3f;       // Lag of Baro/Althold stuff in general, makes stop in hightchange snappier
    cfg.bar_dscl                  = 0.7f;       // Scale downmovement down (because copter drops faster than rising)
    cfg.bar_dbg                   = 0;          // Crashpilot: 1 = Debug Barovalues //cfg.baro_noise_lpf = 0.6f;// Crashpilot: Not used anymore//cfg.baro_cf = 0.985f;// Crashpilot: Not used anymore

    // Autoland
    cfg.al_barolr                 = 50;         // [10 - 200cm/s] Baro Landingrate
    cfg.al_snrlr                  = 50;         // [10 - 200cm/s] Sonar Landingrate - You can specify different landingfactor here on sonar contact, because sonar land maybe too fast when snr_cf is high
    cfg.al_debounce               = 5;          // (0-20%) 0 Disables. Defines a Throttlelimiter on Autoland. Percentage defines the maximum deviation of assumed hoverthrottle during Autoland
    cfg.al_tobaro                 = 2000;       // Timeout in ms (100 - 5000) before shutoff on autoland. "esc_nfly" must be undershot for that timeperiod
    cfg.al_tosnr                  = 1000;       // Timeout in ms (100 - 5000) If sonar aided land is wanted (snr_land = 1) you can choose a different timeout here

    // Autostart
    cfg.as_lnchr                  = 200;        // [50 - 250 no dimension DEFAULT:200] Autostart initial launchrate to get off the ground. When as_stdev is exceeded, as_clmbr takes over
    cfg.as_clmbr                  = 100;        // [50 - 250cm/s DEFAULT:100] Autostart climbrate in cm/s after liftoff! Autostart Rate in cm/s will be lowered when approaching targethight.
    cfg.as_trgt                   = 0;          // [0 - 255m  DEFAULT:0 (0 = Disable)] Autostart Targethight in m Note: use 2m or more
    cfg.as_stdev                  = 10;         // [5 - 20 no dimension DEFAULT:10] This is the std. deviation of the variometer when a liftoff is assumed. The higher the more unsensitive.

    cfg.vbatscale                 = 110;
    cfg.vbatmaxcellvoltage        = 43;
    cfg.vbatmincellvoltage        = 33;
    cfg.power_adc_channel         = 0;

    // Radio
    parseRcChannels("AETR1234");
    cfg.rc_db                     = 20;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.rc_dbyw                   = 20;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.rc_dbah                   = 50;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.rc_dbgps                  = 5;          // Additional Deadband for all GPS functions;
    cfg.devorssi                  = 0;          // Will take the last channel for RSSI value, so add one to rc_auxch, don't use that auxchannel unless you want it to trigger something
                                                // Note Spektrum or Graupner will override that setting to 0.
    cfg.rssicut                   = 0;          // [0-80%][0 Disables] Below that percentage rssi will show zero.
    // cfg.spektrum_hires = 0;
    cfg.rc_minchk                 = 1100;
    cfg.rc_mid                    = 1500;
    cfg.rc_maxchk                 = 1900;
    cfg.rc_lowlat                 = 1;          // [0 - 1] Default 1. 1 = lower latency, 0 = normal latency/more filtering.
    cfg.rc_rllrm                  = 0;          // disable arm/disarm on roll left/right
    cfg.rc_auxch                  = 4;          // [4 - 6] cGiesen: Default = 4, then like the standard! Crashpilot: Limited to 6 aux for safety
    cfg.rc_killt                  = 0;          // Time in ms when your arm switch becomes a Killswitch, 0 disables the Killswitch, can not be used together with FEATURE_INFLIGHT_ACC_CAL
    cfg.rc_flpsp                  = 0;          // [0-3] When enabled(1) and upside down in acro or horizon mode throttle is reduced (see readme)
    cfg.rc_motor                  = 0;          // [0-2] Behaviour when thr < rc_minchk: 0= minthrottle no regulation, 1= minthrottle&regulation, 2= Motorstop 

    // G-tune
    cfg.gt_lolimP[ROLL]           = 20;         // [10..200] Lower limit of ROLL P during G tune.
    cfg.gt_lolimP[PITCH]          = 20;         // [10..200] Lower limit of PITCH P during G tune.
    cfg.gt_lolimP[YAW]            = 20;         // [10..200] Lower limit of YAW P during G tune.
    cfg.gt_hilimP[ROLL]           = 70;         // [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    cfg.gt_hilimP[PITCH]          = 70;         // [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    cfg.gt_hilimP[YAW]            = 70;         // [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    cfg.gt_pwr                    = 0;          // [0..10] Strength of adjustment

    // Motor/ESC/Servo
//  cfg.esc_min                   = 1150;       // ORIG
    cfg.esc_min                   = 1100;
    cfg.esc_max                   = 1950;
    cfg.esc_moff                  = 1000;
    cfg.esc_nfly                  = 1300;       // This is the absolute throttle that kicks off the "has landed timer" if it is too low cfg.rc_minchk + 5% is taken. Also baselinethr for Autostart, also plausibility check for initial Failsafethrottle
//  cfg.esc_nfly                  = 0;          // This is the absolute throttle that kicks off the "has landed timer" if it is too low cfg.rc_minchk + 5% is taken.
    cfg.esc_pwm                   = 400;
    cfg.esc_nwmx                  = 1;          // NewMix: 0 = mwii style, 1 = scaled handling of maxthrottlesituations
    cfg.srv_pwm                   = 50;
    cfg.pass_mot                  = 0;          // Crashpilot: Only used with feature pass. If 0 = all Motors, otherwise specific Motor

    // servos
    cfg.tri_ydir                  = 1;
    cfg.tri_ymid                  = 1500;
    cfg.tri_ymin                  = 1020;
    cfg.tri_ymax                  = 2000;
    cfg.tri_ydel                  = 0;          // [0-1000ms] Tri Yaw Arm delay: Time in ms after wich the yaw servo after arming will engage (useful with "yaw arm"). 0 disables Yawservo always active.

    // flying wing
    cfg.wing_left_min             = 1020;
    cfg.wing_left_mid             = 1500;
    cfg.wing_left_max             = 2000;
    cfg.wing_right_min            = 1020;
    cfg.wing_right_mid            = 1500;
    cfg.wing_right_max            = 2000;
    cfg.pitch_direction_l         = 1;
    cfg.pitch_direction_r         = -1;
    cfg.roll_direction_l          = 1;
    cfg.roll_direction_r          = 1;

    // gimbal
    cfg.gbl_pgn                   = 10;
    cfg.gbl_rgn                   = 10;
    cfg.gbl_flg                   = GIMBAL_NORMAL;
    cfg.gbl_pmn                   = 1020;
    cfg.gbl_pmx                   = 2000;
    cfg.gbl_pmd                   = 1500;
    cfg.gbl_rmn                   = 1020;
    cfg.gbl_rmx                   = 2000;
    cfg.gbl_rmd                   = 1500;

    // gps/nav
    cfg.gps_type                  = 1;          // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
    cfg.gps_baudrate              = 115200;     //38400; // Changed 8/6/13 to 115200;
    cfg.gps_ins_vel               = 0.6f;       // Crashpilot GPS INS The LOWER the value the closer to gps speed // Dont go to high here
    cfg.gps_lag                   = 2000;       // GPS Lag in ms
    cfg.gps_ph_minsat             = 6;          // Minimal Satcount for PH, PH on RTL is still done with 5Sats or more
    cfg.gps_expo                  = 20;         // 1 - 99 % defines the actual Expo applied for GPS
    cfg.gps_ph_settlespeed        = 10;         // 1 - 200 cm/s PH settlespeed in cm/s
    cfg.gps_ph_brakemaxangle      = 15;         // 1 - 45 Degree Maximal Overspeedbrake
    cfg.gps_ph_minbrakepercent    = 50;         // 1 - 99% minimal percent of "brakemaxangle" left over for braking. Example brakemaxangle = 6 so 50 Percent is 3..
    cfg.gps_ph_brkacc             = 40;         // [1 - 500] Is the assumed negative braking acceleration in cm/(s*s) of copter. Value is positive though. It will be a timeout. The lower the Value the longe the Timeout
    cfg.gps_maxangle              = 35;         // 10 - 45 Degree Maximal over all GPS bank angle
    cfg.gps_wp_radius             = 200;
//  cfg.rtl_mnh                   = 20;         // (0 - 200m) Minimal RTL hight in m, 0 disables feature
	  cfg.rtl_mnh                   = 0;          // (0 - 200m) Minimal RTL hight in m, 0 disables feature
    cfg.rtl_cr                    = 80;         // [10 - 200cm/s] When rtl_mnh is defined this is the climbrate in cm/s
    cfg.rtl_mnd                   = 0;          // 0 Disables. Minimal distance for RTL in m, otherwise it will just autoland, prevent Failsafe jump in your face, when arming copter and turning off TX
    cfg.gps_rtl_flyaway           = 0;          // [0 - 100m] 0 Disables. If during RTL the distance increases beyond this value (in meters relative to RTL activation point), something is wrong, autoland

    cfg.gps_yaw                   = 30;         // Thats the MAG P during GPS functions, substitute for "cfg.P8[PIDMAG]"
    cfg.nav_rtl_lastturn          = 1;          // 1 = when copter gets to home position it rotates it's head to takeoff direction independend of nav_controls_heading
    cfg.nav_tail_first            = 0;          // 1 = Copter comes back with ass first (only works with nav_controls_heading = 1)
    cfg.nav_controls_heading      = 0;          // 1 = Nav controls YAW during WP ONLY
//  cfg.nav_controls_heading      = 1;          // 1 = Nav controls YAW during WP ONLY
    cfg.nav_speed_min             = 100;        // 10 - 200 cm/s don't set higher than nav_speed_max! That dumbness is not covered.
    cfg.nav_speed_max             = 350;        // 50 - 2000 cm/s don't set lower than nav_speed_min! That dumbness is not covered.
    cfg.nav_approachdiv           = 3;          // 2 - 10 This is the divisor for approach speed for wp_distance. Example: 400cm / 3 = 133cm/s if below nav_speed_min it will be adjusted
    cfg.nav_tiltcomp              = 30;         // 0 - 100 (20 TestDefault) Only arducopter really knows. Dfault was 54. This is some kind of a hack of them to reach actual nav_speed_max. 54 was Dfault, 0 disables
    cfg.nav_ctrkgain              = 0.5f;       // 0 - 10.0 (0.5 TestDefault) (Floatvariable) That is the "Crosstrackgain" APM Dfault is "1". "0" disables

    // Failsafe Variables
    cfg.fs_delay                  = 10;         // in 0.1s (10 = 1sec)
    cfg.fs_ofdel                  = 200;        // in 0.1s (200 = 20sec)
    cfg.fs_rcthr                  = 1200;       // decent Dfault which should always be below hover throttle for people.
    cfg.fs_ddplt                  = 0;		      // EXPERIMENTAL Time in sec when FS is engaged after idle on THR/YAW/ROLL/PITCH, 0 disables max 250
    cfg.fs_jstph                  = 0;          // Does just PH&Autoland an not RTL, use this in difficult areas with many obstacles to avoid RTL crash into something
    cfg.fs_nosnr                  = 1;          // When snr_land is set to 1, it is possible to ignore that on Failsafe, because FS over a tree could turn off copter

    // serial (USART1) baudrate
    cfg.serial_baudrate           = 115200;
    cfg.tele_prot                 = 0;          // Protocol ONLY used when Armed including Baudchange if necessary. 0 (Dfault)=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)

    // LED Stuff
    cfg.LED_invert                = 0;          // Crashpilot: Inversion of LED 0&1 Partly implemented because Bootup is not affected
    cfg.LED_Type                  = 1;		      // 1=MWCRGB / 2=MONO_LED / 3=LEDRing
    cfg.LED_Pinout                = 1;		      // rc6
    cfg.LED_ControlChannel        = 8;		      // AUX4 (Channel 8)
    cfg.LED_Armed                 = 0;		      // 0 = Show LED only if armed, 1 = always show LED
    cfg.LED_Pattern1			        = 1300; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    cfg.LED_Pattern2			        = 1800; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    cfg.LED_Pattern3			        = 1900; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    cfg.LED_Toggle_Delay1         = 0x08;       // slow down LED_Pattern
    cfg.LED_Toggle_Delay2         = 0x08;       // slow down LED_Pattern
    cfg.LED_Toggle_Delay3         = 0x08;       // slow down LED_Pattern

    // SONAR
    // SOME INFO ON SONAR:
    // PWM56 are 5V resistant, RC78 only tolerate 3.3V(!!) so add a 1K Ohms resistor!!!
    // Note: You will never see the maximum possible sonar range in a copter, so go for the half of it (or less?)
    //
    // Connection possibilities depending on Receivertype:
    // PPSUM: RC78 possible, PWM56 possible (on max. quadcopters, see below)
    // Normal RX: Just Connection on Motorchannel 5&6 (PWM56) is possible.
    // The PWM56 sonar connection option is only available in setups with max motors 4, otherwise sonar is not initialized.
    //
    // HC-SR04:
    // Operation Voltage: 5V (!! Use PWM56 or 1K resistor !!)
    // Range: 2cm - 400cm
    // Angle: 15 Degrees (Test out for yourself: cfg.snr_tilt = X)
    //
    // Maxbotix in general
    // Operation Voltage: (some 2.5V)3.3V - 5V ((!! Use PWM56 or resistor with 5V !!)
    // Only wire the Maxbotics for PWM output (more precise anyway), not the analog etc. modes, just wire echopin (normally pin 2)
    // Range: 20cm(!) - 765cm (some >1000cm), MaxTiltAngle is not specified, depending on Model
    // Tested on MB1200 XL-MaxSonar-EZ0
    //
    // I2C sonar in general (by mj666)
    // If operation voltage of the sonar sensor is 5 Volt (NAZE I2C is 3.3 Volt), take care they do not have pull up resistors connected to 5 Volt.
    // Outputs are always open drain so there is no risk kill something only signals may be critical so keep wires short as possible.
    // Maxbotix I2CXL series operates with 3.3 and 5 Volt but 5Volt are preferred for best performance and stability.
    //
    // Devantech Ltd. (SRF02, SRF235, SRF08, SRF10):
    // Type; Range; Cycletime; Angle; Comment
    // SRF02; 16 to 600cm; 65ms; 55 degree; automatic calibration, minimum rage can be read from sensor (not implemented)
    // SRF235; 10 - 1200cm; 10ms; 15 degree; angle is may be to small for the use case
    // SRF08; 3 - 600cm; 65ms; 55 degree; range, gain an cycletime can be adjusted, multiple echos are measured (both not implemented)
    // SRF10; 6 - 600cm; 65ms; 72 degree; range, gain an cycletime can be adjusted (not implemented)
    // be sure to adjust settings accordingly, no additional test are done.
    // more details at: http://www.robot-electronics.co.uk/index.html
    //
    // Maxbotix I2CXL (MB1202, MB1212, MB1222, MB1232, MB1242)
    // I"CXL Series of sensors only differentiated by the beam pattern and sensibility. Maxbotix is recommending the MX1242 for quadcopter applications. The interface is always the same
    // NOTE: Maxbotix Sonars only operate with lower I2C speed, so the speed is changed on the fly during Maxbotix readout.
    // Thanks must go to mj666 for implementing that!
    // GENERAL WARNING: DON'T SET snr_min TOO LOW, OTHERWISE THE WRONG SONARVALUE WILL BE TAKEN AS REAL MEASUREMENT!!
    // I implemented some checks to prevent that user error, but still keep that in mind.
    // Min/Max are checked and changed if they are too stupid for your sonar. So if you suddenly see other values, thats not an eeprom error or so.
    // MAXBOTICS: SET snr_min to at least 30! I check this in sensors and change the value, if needed.
    // NOTE: I limited Maxbotics to 7 meters in the code, knowing that some types will do >10m, if you have one of them 7m is still the limit for you.
    // HC-SR04:   SET snr_min to at least 10 ! I check this in sensors and change the value, if needed.
    // DaddyWalross Sonar: I DON'T KNOW! But it uses HC-SR04 so i apply the same limits (10cm-400cm) to its output
    // Sonar minimal hight must be higher (including temperature difference) than the physical lower limit of the sensor to do a proximity alert
    // NOTE: Sonar is def. not a must - have. But nice to have.
    cfg.snr_type                  = 3;          // 0=PWM56 HC-SR04, 1=RC78 HC-SR04, 2=I2C(DaddyWalross), 3=MBPWM56, 4=MBRC78, 5=I2C(SRFxx), 6=I2C (MX12x2)
    cfg.snr_min                   = 30;         // Valid Sonar minimal range in cm (10 - 200) see warning above
    cfg.snr_max                   = 200;        // Valid Sonar maximal range in cm (50 - 700)
    cfg.snr_dbg                   = 0;          // 1 Sends Sonardata (within defined range and tilt) to debug[0] and tiltvalue to debug[1], debug[0] will be -1 if out of range/tilt. debug[2] contains raw sonaralt, like before
    cfg.snr_tilt                  = 18;         // Somehow copter tiltrange in degrees (Not exactly but good enough. Value * 0.9 = realtilt) in wich Sonar is possible
    cfg.snr_cf                    = 0.5f;       // The bigger, the more Sonarinfluence, makes switch between Baro/Sonar smoother and defines baroinfluence when sonarcontact. 1.0f just takes Sonar, if contact (otherwise baro)
    cfg.snr_diff                  = 0;          // 0 disables that check. Range (0-200) Maximal allowed difference in cm between sonar readouts (100ms rate and snr_diff = 50 means max 5m/s)
    cfg.snr_land                  = 1;          // Aided Sonar - landing, by setting upper throttle limit to current throttle. - Beware of Trees!! Can be disabled for Failsafe with fs_nosnr = 1

    cfg.FDUsedDatasets            = 0;          // Default no Datasets stored
    cfg.stat_clear                = 1;          // This will clear the stats between flights, or you can set to 0 and treasue overallstats, but you have to write manually eeprom or have logging enabled
    cfg.sens_1G                   = 1;          // Just feed a dummy "1" to avoid div by zero
    ClearStats();

    for (i = 0; i < MAX_MOTORS; i++) cfg.customMixer[i].throttle = 0.0f;// custom mixer. clear by Dfaults.
    writeParams(0);
}

void ClearStats(void)
{
    cfg.GPS_MaxDistToHome = cfg.MAXGPSspeed = 0;
    cfg.MaxAltMeter = cfg.MinAltMeter = 0;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}

bool feature(uint32_t mask)
{
    return cfg.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    cfg.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    cfg.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    cfg.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return cfg.enabledFeatures;
}
