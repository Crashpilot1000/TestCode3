#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"

typedef enum
{
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct
{
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
    const uint8_t lcd; // 1 = Displayed in LCD // 0 = Not displayed
} clivalue_t;

// we unset this on 'exit'
static void cliAuxset(char *cmdline);
static void cliCMix(char *cmdline);
static void cliDefault(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);
static void cliScanbus(char *cmdline);
static void cliPassgps(char *cmdline);
static void cliFlash(char *cmdline);
static void cliErrorMessage(void);
static void cliRecal(void);
static void cliSetVar(const clivalue_t *var, const int32_t value);
static void changeval(const clivalue_t *var, const int8_t adder);
static void cliPrintVar(const clivalue_t *var, uint32_t full);
static void cliWpflush(char *cmdline);

// serial LCD
static void LCDinit(void);
static void LCDclear(void);
static void LCDline1(void);
static void LCDline2(void);

// OLED-Display
extern bool i2cLCD;                         // true, if an OLED-Display is connected

// from sensors.c
extern uint8_t batteryCellCount;
extern uint8_t accHardware;
extern float   gyroZero[3];                 // Populated upon initialization
extern bool    havel3g4200d;


// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// gpspass
static void GPSbyteRec(uint16_t c);
static uint8_t NewGPSByte;
static bool HaveNewGpsByte;

// buffer
static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
const char * const mixerNames[] =
{
    "TRI", "QUADP", "QUADX", "BI", "GIMBAL", "Y6", "HEX6", "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP",
    "OCTOFLATX", "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "HEX6H", "PPM_TO_SERVO", "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
const char * const featureNames[] =
{
    "PPM",
    "VBAT",
    "SPEKTRUM",
    "GRAUPNERSUMH",
    "SERVO_TILT",
    "LED",
    "GPS",
    "FAILSAFE",
    "SONAR",
    "PASS",
    "LCD",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] =
{
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

//
const char * const accNames[] =
{
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct
{
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] =
{
    { "auxset", "alternative to GUI", cliAuxset },
    { "cmix", "design custom mixer", cliCMix },
    { "default", "load defaults & reboot", cliDefault },
    { "dump",    "dump config", cliDump },
    { "exit",    "exit & reboot", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "flash",   "flashmode", cliFlash },
    { "help",    "this text", cliHelp },
    { "map",     "mapping of rc channel order", cliMap },
    { "mixer",   "mixer name or list", cliMixer },
    { "passgps", "pass through gps data", cliPassgps },
    { "save",    "save and reboot", cliSave },
    { "scanbus", "scan i2c bus", cliScanbus },
    { "set",     "name=value or blank or * for list", cliSet },
    { "status",  "sys status & stats", cliStatus },
    { "version", "", cliVersion },
    { "wpflush", "clear wp list", cliWpflush },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

const clivalue_t valueTable[] =
{
    { "rc_db",                     VAR_UINT8,  &cfg.rc_db,                       0,         32, 1 },
    { "rc_dbyw",                   VAR_UINT8,  &cfg.rc_dbyw,                     0,        100, 1 },
    { "rc_dbah",                   VAR_UINT8,  &cfg.rc_dbah,                     1,        100, 1 },
    { "rc_dbgps",                  VAR_UINT8,  &cfg.rc_dbgps,                    0,        100, 1 },
    { "rc_trm_rll",                VAR_FLOAT,  &cfg.angleTrim[ROLL],          -300,        300, 1 },
    { "rc_trm_ptch",               VAR_FLOAT,  &cfg.angleTrim[PITCH],         -300,        300, 1 },
    { "rc_minchk",                 VAR_UINT16, &cfg.rc_minchk,                1001,       1250, 0 },
    { "rc_mid",                    VAR_UINT16, &cfg.rc_mid,                   1400,       1600, 0 },
    { "rc_maxchk",                 VAR_UINT16, &cfg.rc_maxchk,                1750,       1999, 0 },
    { "rc_lowlat",                 VAR_UINT8,  &cfg.rc_lowlat,                   0,          1, 0 },
    { "rc_rllrm",                  VAR_UINT8,  &cfg.rc_rllrm,                    0,          1, 0 },
    { "rc_killt",                  VAR_UINT16, &cfg.rc_killt,                    0,      10000, 1 },
    { "rc_flpsp",                  VAR_UINT8,  &cfg.rc_flpsp,                    0,          3, 1 },
    { "rc_motor",                  VAR_UINT8,  &cfg.rc_motor,                    0,          2, 1 },
    { "rc_auxch",                  VAR_UINT8,  &cfg.rc_auxch,                    0,          6, 0 },
    { "rc_rate",                   VAR_UINT8,  &cfg.rcRate8,                     0,        250, 1 },
    { "rc_expo",                   VAR_UINT8,  &cfg.rcExpo8,                     0,        100, 1 },
    { "thr_mid",                   VAR_UINT8,  &cfg.thrMid8,                     0,        100, 1 },
    { "thr_expo",                  VAR_UINT8,  &cfg.thrExpo8,                    0,        250, 1 },
    { "roll_pitch_rate",           VAR_UINT8,  &cfg.rollPitchRate,               0,        100, 1 },
    { "yawrate",                   VAR_UINT8,  &cfg.yawRate,                     0,        100, 1 },
    { "devorssi",                  VAR_UINT8,  &cfg.devorssi,                    0,          1, 0 },
    { "rssicut",                   VAR_UINT8,  &cfg.rssicut,                     0,         80, 0 },
    { "gt_loP_rll",                VAR_UINT8,  &cfg.gt_lolimP[ROLL],            10,        200, 1 },
    { "gt_loP_ptch",               VAR_UINT8,  &cfg.gt_lolimP[PITCH],           10,        200, 1 },
    { "gt_loP_yw",                 VAR_UINT8,  &cfg.gt_lolimP[YAW],             10,        200, 1 },
    { "gt_hiP_rll",                VAR_UINT8,  &cfg.gt_hilimP[ROLL],             0,        200, 1 },
    { "gt_hiP_ptch",               VAR_UINT8,  &cfg.gt_hilimP[PITCH],            0,        200, 1 },
    { "gt_hiP_yw",                 VAR_UINT8,  &cfg.gt_hilimP[YAW],              0,        200, 1 },
    { "gt_pwr",                    VAR_INT8,   &cfg.gt_pwr,                      0,         10, 1 },
    { "esc_min",                   VAR_UINT16, &cfg.esc_min,                     0,       2000, 0 },
    { "esc_max",                   VAR_UINT16, &cfg.esc_max,                     0,       2000, 0 },
    { "esc_nfly",                  VAR_UINT16, &cfg.esc_nfly,                    0,       2000, 1 },
    { "esc_nwmx",                  VAR_UINT8,  &cfg.esc_nwmx,                    0,          1, 1 },
    { "esc_moff",                  VAR_UINT16, &cfg.esc_moff,                    0,       2000, 0 },
    { "esc_pwm",                   VAR_UINT16, &cfg.esc_pwm,                    50,        498, 0 },
    { "srv_pwm",                   VAR_UINT16, &cfg.srv_pwm,                    50,        498, 0 },
    { "pass_mot",                  VAR_UINT8,  &cfg.pass_mot,                    0,         10, 0 },
    { "fs_delay",                  VAR_UINT8,  &cfg.fs_delay,                    0,         40, 1 },
    { "fs_ofdel",                  VAR_UINT8,  &cfg.fs_ofdel,                    0,        200, 1 },
    { "fs_rcthr",                  VAR_UINT16, &cfg.fs_rcthr,                 1000,       2000, 1 },
    { "fs_ddplt",                  VAR_UINT8,  &cfg.fs_ddplt,                    0,        250, 1 },
    { "fs_jstph",                  VAR_UINT8,  &cfg.fs_jstph,                    0,          1, 1 },
    { "fs_nosnr",                  VAR_UINT8,  &cfg.fs_nosnr,                    0,          1, 1 },
    { "serial_baudrate",           VAR_UINT32, &cfg.serial_baudrate,          1200,     115200, 0 },
    { "tele_prot",                 VAR_UINT8,  &cfg.tele_prot,                   0,          3, 1 },
    { "spektrum_hires",            VAR_UINT8,  &cfg.spektrum_hires,              0,          1, 0 },
    { "vbatscale",                 VAR_UINT8,  &cfg.vbatscale,                  10,        200, 0 },
    { "vbatmaxcellvoltage",        VAR_UINT8,  &cfg.vbatmaxcellvoltage,         10,         50, 0 },
    { "vbatmincellvoltage",        VAR_UINT8,  &cfg.vbatmincellvoltage,         10,         50, 0 },
    { "power_adc_channel",         VAR_UINT8,  &cfg.power_adc_channel,           0,          9, 0 },
    { "tri_ydir",                  VAR_INT8,   &cfg.tri_ydir,                   -1,          1, 0 },
    { "tri_ymid",                  VAR_UINT16, &cfg.tri_ymid,                    0,       2000, 1 },
    { "tri_ymin",                  VAR_UINT16, &cfg.tri_ymin,                    0,       2000, 1 },
    { "tri_ymax",                  VAR_UINT16, &cfg.tri_ymax,                    0,       2000, 1 },
    { "tri_ydel",                  VAR_UINT16, &cfg.tri_ydel,                    0,       1000, 1 },    
    { "wing_left_min",             VAR_UINT16, &cfg.wing_left_min,               0,       2000, 0 },
    { "wing_left_mid",             VAR_UINT16, &cfg.wing_left_mid,               0,       2000, 0 },
    { "wing_left_max",             VAR_UINT16, &cfg.wing_left_max,               0,       2000, 0 },
    { "wing_right_min",            VAR_UINT16, &cfg.wing_right_min,              0,       2000, 0 },
    { "wing_right_mid",            VAR_UINT16, &cfg.wing_right_mid,              0,       2000, 0 },
    { "wing_right_max",            VAR_UINT16, &cfg.wing_right_max,              0,       2000, 0 },
    { "pitch_direction_l",         VAR_INT8,   &cfg.pitch_direction_l,          -1,          1, 0 },
    { "pitch_direction_r",         VAR_INT8,   &cfg.pitch_direction_r,          -1,          1, 0 },
    { "roll_direction_l",          VAR_INT8,   &cfg.roll_direction_l,           -1,          1, 0 },
    { "roll_direction_r",          VAR_INT8,   &cfg.roll_direction_r,           -1,          1, 0 },
    { "gbl_flg",                   VAR_UINT8,  &cfg.gbl_flg,                     0,        255, 0 },
    { "gbl_pgn",                   VAR_INT8,   &cfg.gbl_pgn,                  -100,        100, 0 },
    { "gbl_rgn",                   VAR_INT8,   &cfg.gbl_rgn,                  -100,        100, 0 },
    { "gbl_pmn",                   VAR_UINT16, &cfg.gbl_pmn,                   100,       3000, 0 },
    { "gbl_pmx",                   VAR_UINT16, &cfg.gbl_pmx,                   100,       3000, 0 },
    { "gbl_pmd",                   VAR_UINT16, &cfg.gbl_pmd,                   100,       3000, 0 },
    { "gbl_rmn",                   VAR_UINT16, &cfg.gbl_rmn,                   100,       3000, 0 },
    { "gbl_rmx",                   VAR_UINT16, &cfg.gbl_rmx,                   100,       3000, 0 },
    { "gbl_rmd",                   VAR_UINT16, &cfg.gbl_rmd,                   100,       3000, 0 },
    { "al_barolr",                 VAR_UINT8,  &cfg.al_barolr,                  10,        200, 1 },
    { "al_snrlr",                  VAR_UINT8,  &cfg.al_snrlr,                   10,        200, 1 },
    { "al_debounce",               VAR_UINT8,  &cfg.al_debounce,                 0,         20, 1 },
    { "al_tobaro",                 VAR_UINT16, &cfg.al_tobaro,                 100,       5000, 1 },
    { "al_tosnr",                  VAR_UINT16, &cfg.al_tosnr,                  100,       5000, 1 },
    { "as_lnchr",                  VAR_UINT8,  &cfg.as_lnchr,                   50,        250, 1 },
    { "as_clmbr",                  VAR_UINT8,  &cfg.as_clmbr,                   50,        250, 1 },
    { "as_trgt",                   VAR_UINT8,  &cfg.as_trgt,                     0,        255, 1 },
    { "as_stdev",                  VAR_UINT8,  &cfg.as_stdev,                    5,         20, 1 },
    { "align_gyro_x",              VAR_INT8,   &cfg.align[ALIGN_GYRO][0],       -3,          3, 0 },
    { "align_gyro_y",              VAR_INT8,   &cfg.align[ALIGN_GYRO][1],       -3,          3, 0 },
    { "align_gyro_z",              VAR_INT8,   &cfg.align[ALIGN_GYRO][2],       -3,          3, 0 },
    { "align_acc_x",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][0],      -3,          3, 0 },
    { "align_acc_y",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][1],      -3,          3, 0 },
    { "align_acc_z",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][2],      -3,          3, 0 },
    { "align_mag_x",               VAR_INT8,   &cfg.align[ALIGN_MAG][0],        -3,          3, 0 },
    { "align_mag_y",               VAR_INT8,   &cfg.align[ALIGN_MAG][1],        -3,          3, 0 },
    { "align_mag_z",               VAR_INT8,   &cfg.align[ALIGN_MAG][2],        -3,          3, 0 },
    { "align_board_yaw",           VAR_UINT8,  &cfg.align_board_yaw,             0,          3, 0 },
    { "acc_hdw",                   VAR_UINT8,  &cfg.acc_hdw,                     0,          3, 0 },
    { "acc_lpfhz",                 VAR_FLOAT,  &cfg.acc_lpfhz,                   0,        100, 1 },
    { "acc_altlpfhz",              VAR_UINT8,  &cfg.acc_altlpfhz,                1,        100, 1 },
    { "acc_gpslpfhz",              VAR_UINT8,  &cfg.acc_gpslpfhz,                1,        100, 1 },
    { "gy_lpf",                    VAR_UINT16, &cfg.gy_lpf,                      0,        256, 0 },
    { "gy_gcmpf",                  VAR_UINT16, &cfg.gy_gcmpf,                    1,      10000, 1 },
    { "gy_mcmpf",                  VAR_UINT16, &cfg.gy_mcmpf,                    1,      10000, 1 },
    { "gy_smrll",                  VAR_UINT8,  &cfg.gy_smrll,                    0,        200, 1 },
    { "gy_smptc",                  VAR_UINT8,  &cfg.gy_smptc,                    0,        200, 1 },
    { "gy_smyw",                   VAR_UINT8,  &cfg.gy_smyw,                     0,        200, 1 },
    { "gy_stdev",                  VAR_UINT8,  &cfg.gy_stdev,                    5,        100, 0 },
    { "accz_vcf",                  VAR_FLOAT,  &cfg.accz_vcf,                    0,          1, 1 },
    { "accz_acf",                  VAR_FLOAT,  &cfg.accz_acf,                    0,          1, 1 },
    { "bar_lag",                   VAR_FLOAT,  &cfg.bar_lag,                     0,         10, 1 },
    { "bar_dscl",                  VAR_FLOAT,  &cfg.bar_dscl,                    0,          1, 1 },
    { "bar_dbg",                   VAR_UINT8,  &cfg.bar_dbg,                     0,          1, 0 },
    { "mag_dec",                   VAR_INT16,  &cfg.mag_dec,                -18000,      18000, 1 },
    { "mag_time",                  VAR_UINT8,  &cfg.mag_time,                    1,          6, 1 },
    { "mag_gain",                  VAR_UINT8,  &cfg.mag_gain,                    0,          1, 1 },
    { "gps_baudrate",              VAR_UINT32, &cfg.gps_baudrate,             1200,     115200, 0 },
    { "gps_type",                  VAR_UINT8,  &cfg.gps_type,                    0,          9, 0 },
    { "gps_ins_vel",               VAR_FLOAT,  &cfg.gps_ins_vel,                 0,          1, 1 },
    { "gps_lag",                   VAR_UINT16, &cfg.gps_lag,                     0,      10000, 1 },
    { "gps_ph_minsat",             VAR_UINT8,  &cfg.gps_ph_minsat,               5,         10, 1 },
    { "gps_expo",                  VAR_UINT8,  &cfg.gps_expo,                    0,         99, 1 },
    { "gps_ph_settlespeed",        VAR_UINT8,  &cfg.gps_ph_settlespeed,          1,        200, 1 },
    { "gps_maxangle",              VAR_UINT8,  &cfg.gps_maxangle,               10,         45, 1 },
    { "gps_ph_brakemaxangle",      VAR_UINT8,  &cfg.gps_ph_brakemaxangle,        1,         45, 1 },
    { "gps_ph_minbrakepercent",    VAR_UINT8,  &cfg.gps_ph_minbrakepercent,      1,         99, 1 },
    { "gps_ph_brkacc",             VAR_UINT16, &cfg.gps_ph_brkacc,               1,        500, 1 },
    { "gps_wp_radius",             VAR_UINT16, &cfg.gps_wp_radius,               0,       2000, 1 },
    { "rtl_mnh",                   VAR_UINT8,  &cfg.rtl_mnh,                     0,        200, 1 },
    { "rtl_cr",                    VAR_UINT8,  &cfg.rtl_cr,                     10,        200, 1 },
    { "rtl_mnd",                   VAR_UINT8,  &cfg.rtl_mnd,                     0,         50, 1 },
    { "gps_rtl_flyaway",           VAR_UINT8,  &cfg.gps_rtl_flyaway,             0,        100, 1 },
    { "gps_yaw",                   VAR_UINT8,  &cfg.gps_yaw,                    20,        150, 1 },
    { "nav_rtl_lastturn",          VAR_UINT8,  &cfg.nav_rtl_lastturn,            0,          1, 1 },
    { "nav_speed_min",             VAR_UINT8,  &cfg.nav_speed_min,              10,        200, 1 },
    { "nav_speed_max",             VAR_UINT16, &cfg.nav_speed_max,              50,       2000, 1 },
    { "nav_approachdiv",           VAR_UINT8,  &cfg.nav_approachdiv,             2,         10, 1 },
    { "nav_tiltcomp",              VAR_UINT8,  &cfg.nav_tiltcomp,                0,        100, 1 },
    { "nav_ctrkgain",              VAR_FLOAT,  &cfg.nav_ctrkgain,                0,         10, 1 },
    { "nav_controls_heading",      VAR_UINT8,  &cfg.nav_controls_heading,        0,          1, 1 },
    { "nav_tail_first",            VAR_UINT8,  &cfg.nav_tail_first,              0,          1, 1 },
    { "stat_clear",                VAR_UINT8,  &cfg.stat_clear,                  0,          1, 1 },    
    { "gps_pos_p",                 VAR_UINT8,  &cfg.P8[PIDPOS],                  0,        200, 1 },
    { "gps_pos_i",                 VAR_UINT8,  &cfg.I8[PIDPOS],                  0,        200, 0 },
    { "gps_pos_d",                 VAR_UINT8,  &cfg.D8[PIDPOS],                  0,        200, 0 },
    { "gps_posr_p",                VAR_UINT8,  &cfg.P8[PIDPOSR],                 0,        200, 1 },
    { "gps_posr_i",                VAR_UINT8,  &cfg.I8[PIDPOSR],                 0,        200, 1 },
    { "gps_posr_d",                VAR_UINT8,  &cfg.D8[PIDPOSR],                 0,        200, 1 },
    { "gps_nav_p",                 VAR_UINT8,  &cfg.P8[PIDNAVR],                 0,        200, 1 },
    { "gps_nav_i",                 VAR_UINT8,  &cfg.I8[PIDNAVR],                 0,        200, 1 },
    { "gps_nav_d",                 VAR_UINT8,  &cfg.D8[PIDNAVR],                 0,        200, 1 },
    { "looptime",                  VAR_UINT16, &cfg.looptime,                 1000,       9000, 1 },
    { "mainpidctrl",               VAR_UINT8,  &cfg.mainpidctrl,                 0,          1, 1 },
    { "maincuthz",                 VAR_UINT8,  &cfg.maincuthz,                   1,        100, 1 },
    { "gpscuthz",                  VAR_UINT8,  &cfg.gpscuthz,                    1,        100, 1 },    
    { "p_pitch",                   VAR_UINT8,  &cfg.P8[PITCH],                   1,        200, 1 },
    { "i_pitch",                   VAR_UINT8,  &cfg.I8[PITCH],                   0,        200, 1 },
    { "d_pitch",                   VAR_UINT8,  &cfg.D8[PITCH],                   0,        200, 1 },
    { "p_roll",                    VAR_UINT8,  &cfg.P8[ROLL],                    1,        200, 1 },
    { "i_roll",                    VAR_UINT8,  &cfg.I8[ROLL],                    0,        200, 1 },
    { "d_roll",                    VAR_UINT8,  &cfg.D8[ROLL],                    0,        200, 1 },
    { "p_yaw",                     VAR_UINT8,  &cfg.P8[YAW],                     0,        200, 1 },
    { "i_yaw",                     VAR_UINT8,  &cfg.I8[YAW],                     0,        200, 1 },
    { "d_yaw",                     VAR_UINT8,  &cfg.D8[YAW],                     0,        200, 1 },
    { "p_alt",                     VAR_UINT8,  &cfg.P8[PIDALT],                  0,        200, 1 },
    { "i_alt",                     VAR_UINT8,  &cfg.I8[PIDALT],                  0,        200, 1 },
    { "d_alt",                     VAR_UINT8,  &cfg.D8[PIDALT],                  0,        200, 1 },
    { "p_level",                   VAR_UINT8,  &cfg.P8[PIDLEVEL],                0,        200, 1 },
    { "i_level",                   VAR_UINT8,  &cfg.I8[PIDLEVEL],                0,        200, 1 },
    { "d_level",                   VAR_UINT8,  &cfg.D8[PIDLEVEL],                0,        200, 1 },
    { "snr_type",                  VAR_UINT8,  &cfg.snr_type,                    0,          6, 0 },
    { "snr_min",                   VAR_UINT8,  &cfg.snr_min,                    20,        200, 1 },
    { "snr_max",                   VAR_UINT16, &cfg.snr_max,                    50,        700, 1 },
    { "snr_dbg",                   VAR_UINT8,  &cfg.snr_dbg,                     0,          1, 0 },
    { "snr_tilt",                  VAR_UINT8,  &cfg.snr_tilt,                   10,         50, 1 },
    { "snr_cf",                    VAR_FLOAT,  &cfg.snr_cf,                      0,          1, 1 },
    { "snr_diff",                  VAR_UINT8,  &cfg.snr_diff,                    0,        200, 1 },
    { "snr_land",                  VAR_UINT8,  &cfg.snr_land,                    0,          1, 1 },    
    { "LED_invert",                VAR_UINT8,  &cfg.LED_invert,                  0,          1, 0 },
    { "LED_Type",                  VAR_UINT8,  &cfg.LED_Type,                    0,          3, 0 },
    { "LED_pinout",                VAR_UINT8,  &cfg.LED_Pinout,                  0,          1, 0 },
    { "LED_ControlChannel",        VAR_UINT8,  &cfg.LED_ControlChannel,          1,         12, 0 }, // Aux Channel to controll the LED Pattern
    { "LED_ARMED",                 VAR_UINT8,  &cfg.LED_Armed,                   0,          1, 1 }, // 0 = Show LED only if armed, 1 = always show LED
    { "LED_Toggle_Delay1",         VAR_UINT8,  &cfg.LED_Toggle_Delay1,           0,        255, 0 },
    { "LED_Toggle_Delay2",         VAR_UINT8,  &cfg.LED_Toggle_Delay2,           0,        255, 0 },
    { "LED_Toggle_Delay3",         VAR_UINT8,  &cfg.LED_Toggle_Delay3,           0,        255, 0 },
    { "LED_Pattern1",              VAR_UINT32, &cfg.LED_Pattern1,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 1
    { "LED_Pattern2",              VAR_UINT32, &cfg.LED_Pattern2,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 2
    { "LED_Pattern3",              VAR_UINT32, &cfg.LED_Pattern3,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 3};
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0)
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0)
    {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    }
    else
        *i2a(i, a, r) = 0;
    return a;
}

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    int frac = 0;
    double sign, value, scale;
    while (white_space(*p) )                                            // Skip leading white space, if any.
    {
        p += 1;
    }
    sign = 1.0;                                                         // Get sign, if any.
    if (*p == '-')
    {
        sign = -1.0;
        p += 1;

    }
    else if (*p == '+')
    {
        p += 1;
    }
    value = 0.0;                                                        // Get digits before decimal point or exponent, if any.
    while (valid_digit(*p))
    {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }
    if (*p == '.')                                                      // Get digits after decimal point, if any.
    {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p))
        {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }
    scale = 1.0;                                                        // Handle exponent, if any.
    if ((*p == 'e') || (*p == 'E'))
    {
        unsigned int expon;
        p += 1;
        frac = 0;                                                       // Get sign of exponent, if any.
        if (*p == '-')
        {
            frac = 1;
            p += 1;

        }
        else if (*p == '+')
        {
            p += 1;
        }
        expon = 0;                                                      // Get digits of exponent, if any.
        while (valid_digit(*p))
        {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;
        while (expon >= 50)                                             // Calculate scaling factor.
        {
            scale *= 1E50;
            expon -= 50;
        }
        while (expon >=  8)
        {
            scale *= 1E8;
            expon -=  8;
        }
        while (expon >   0)
        {
            scale *= 10.0;
            expon -=  1;
        }
    }
    return sign * (frac ? (value / scale) : (value * scale));           // Return signed and scaled floating point result.
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0) x += 0.0005f;
    else x -= 0.0005f;
    value = (int32_t) (x * 1000.0f);                                    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);                                   // Create string from abs of integer value

    if (value >= 0) intString2[0] = ' ';                                // Positive number, add a pad space
    else intString2[0] = '-';                                           // Negative number, add a negative sign

    if (strlen(intString1) == 1)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 2)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 3)
    {
        intString2[1] = '0';
        strcat(intString2, intString1);
    }
    else
    {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);
    return floatString;
}

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void PrintBox(uint8_t number, bool fillup)                       // Prints Out Boxname, left aligned and 8 chars. Cropping or filling with blank may occur
{
#define MaxCharInline 8
    uint8_t i = 0, k = 0;
    bool DoBlank = false;

    if (number >= CHECKBOXITEMS) return;
    while (k != number)
    {
        if(boxnames[i] == ';') k++;
        i++;
    }
    k = i;
    for (i = 0; i < MaxCharInline; i++)
    {
        if (boxnames[k + i] == ';')
        {
            if (fillup) DoBlank = true;
            else return;
        }
        if (DoBlank) printf(" ");
        else printf("%c", boxnames[k + i]);          
    }
}

static void PrintoutAUX(void)
{
    uint8_t  i, k, MaxAuxNumber = max(cfg.rc_auxch, 4);
    char     buf[3];
    uint32_t val;
    printf("ID|AUXCHAN :");
    for (i = 0; i < MaxAuxNumber; i++) printf(" %02u  ", i + 1);
    for (i = 0; i < CHECKBOXITEMS; i++)                                 // print out aux channel settings
    {
        printf("\r\n%02u|", i);
        PrintBox(i, true);
        printf(":");
        for (k = 0; k < MaxAuxNumber; k++)
        {
            strcpy(buf,"---");
            val = cfg.activate[i];
            val = val >> (k * 3);
            if (val & 1) buf[0] = 'L';
            if (val & 2) buf[1] = 'M';
            if (val & 4) buf[2] = 'H';
            printf(" %s ", buf);
        }
    }
    printf("\r\n");
}

static void cliAuxset(char *cmdline)
{
    uint32_t val = 1;
    bool     remove = false;
    char     *ptr = cmdline, buf[4];
    uint8_t  i, AuxChNr, ItemID, len = strlen(cmdline), MaxAuxNumber = max(cfg.rc_auxch, 4);

    if (len && *ptr == '-')
    {
        ptr++;
        if (*ptr == '-')
        {
            for (i = 0; i < CHECKBOXITEMS; i++) cfg.activate[i] = 0;
            printf("Wiped aux.\r\n");
            PrintoutAUX();
            return;
        }
        remove = true;
    }

    if (!len || len < 5)
    {
        printf("\r\nSet: auxset ID aux state(H/M/L)\r\n");
        printf("Remove: auxset -ID etc.\r\n");
        printf("Wipe all: auxset --\r\n");
        printf("Ex: auxset 1 4 h Sets Box 1 to Aux4 High\r\n\r\n");
        PrintoutAUX();
        return;
    }

    ItemID = atoi(ptr);
    ptr = strchr(ptr, ' ') + 1;
    AuxChNr = atoi(ptr);
    if (AuxChNr > MaxAuxNumber || !AuxChNr || ItemID >= CHECKBOXITEMS)
    {
        cliErrorMessage();
        return;
    }
    AuxChNr--;
    ptr = strchr(ptr, ' ') + 1;
    i   = AuxChNr * 3;
    switch(*ptr)
    {
    case 'L':
    case 'l':
        strcpy(buf,"LOW ");
        val <<= i;
        break;
    case 'M':
    case 'm':
        strcpy(buf,"MED ");
        val <<= (i + 1);
        break;
    case 'H':
    case 'h':
        strcpy(buf,"HIGH");
        val <<= (i + 2);
        break;
    default:
        cliErrorMessage();
        return;
    }
    cfg.activate[ItemID] |= val;                                        // Set it here in any case, so we can eor it away if needed
    if(remove)
    {
        cfg.activate[ItemID] ^= val;
        printf("Removing ");
    }
    else
    {
        printf("Setting ");
    }
    PrintBox(ItemID, false);
    printf(" Aux %02u %s\r\n", AuxChNr + 1, buf);
    PrintoutAUX();
}

static void cliCMix(char *cmdline)
{
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    float mixsum[3];
    char *ptr;
    len = strlen(cmdline);
    if (!len)
    {
        uartPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++)
        {
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            printf("#%d:\t", i + 1);
            printf("%s\t", ftoa(cfg.customMixer[i].throttle, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].roll, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].pitch, buf));
            printf("%s\r\n", ftoa(cfg.customMixer[i].yaw, buf));
        }
        for (i = 0; i < 3; i++)                                         // Fix by meister
            mixsum[i] = 0.0f;
        for (i = 0; i < num_motors; i++)
        {
            mixsum[0] += cfg.customMixer[i].roll;
            mixsum[1] += cfg.customMixer[i].pitch;
            mixsum[2] += cfg.customMixer[i].yaw;
        }
        uartPrint("Sanity check:\t");
        for (i = 0; i < 3; i++)
            uartPrint(fabs(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        uartPrint("\r\n");
        return;
    }
    else if (strncasecmp(cmdline, "load", 4) == 0)
    {
        ptr = strchr(cmdline, ' ');
        if (ptr)
        {
            len = strlen(++ptr);
            for (i = 0; ; i++)
            {
                if (mixerNames[i] == NULL)
                {
                    cliErrorMessage();                                  // uartPrint("Invalid mixer type...\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0)
                {
                    mixerLoadMix(i);
                    printf("Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    }
    else
    {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_MOTORS)
        {
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].throttle = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].roll = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].pitch = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].yaw = _atof(++ptr);
                check++;
            }
            if (check != 4) uartPrint("Invalid arguments, needs idx thr roll pitch yaw\r\n");
            else cliCMix("");
        }
        else printf("Motor nr not in range 1 - %d\r\n", MAX_MOTORS);
    }
}

static void cliDefault(char *cmdline)
{
    uartPrint("Resetting to defaults...\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting...");
    systemReset(false);
}

static void cliDump(char *cmdline)
{
    printf("Config:\r\n");
    printf("FW: %s\r\n", FIRMWARE);
    PrintoutAUX();
    cliMixer(cmdline);
    cliCMix(cmdline);
    cliFeature(cmdline);
    cliMap(cmdline);
    cliSet(cmdline);
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (!len)
    {
        uartPrint("Enabled features: ");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            if (mask & (1 << i)) printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available features: \r\n");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            printf("%s \r\n", featureNames[i]);
        }
        uartPrint("\r\n");
        return;
    }
    else
    {
        bool remove = false;
        bool fpass  = feature(FEATURE_PASS);
        if (cmdline[0] == '-')
        {
            remove = true;                                              // remove feature
            cmdline++;                                                  // skip over -
            len--;
        }

        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL)
            {
                uartPrint("Invalid feature name\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0)
            {
                if (remove)
                {
                    featureClear(1 << i);
                    uartPrint("Disabled ");
                }
                else
                {
                    featureSet(1 << i);
                    uartPrint("Enabled ");
                }
                if (fpass != feature(FEATURE_PASS)) cfg.pass_mot = 0;   // Reset to all motors if feature pass was changed
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;
    uartPrint("Available commands:\r\n\r\n");
    for (i = 0; i < CMD_COUNT; i++) printf("%s\t %s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];
    len = strlen(cmdline);
    if (len == 8)
    {
        // uppercase it
        for (i = 0; i < 8; i++) cmdline[i] = toupper((unsigned char)cmdline[i]); // toupper(cmdline[i]);
        for (i = 0; i < 8; i++)
        {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            uartPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    uartPrint("Current assignment: ");
    for (i = 0; i < 8; i++) out[cfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    printf("%s\r\n", out);
}

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len;

    len = strlen(cmdline);

    if (!len)
    {
        printf("Current mixer: %s\r\n", mixerNames[cfg.mixerConfiguration - 1]);
        return;
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available mixers: ");
        for (i = 0; ; i++)
        {
            if (mixerNames[i] == NULL) break;
            printf("%s ", mixerNames[i]);
        }
        uartPrint("\r\n");
        return;
    }

    for (i = 0; ; i++)
    {
        if (mixerNames[i] == NULL)
        {
            uartPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0)
        {
            cfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode without saving\r\n");
    uartPrint("\r\nRebooting...");
    systemReset(false);                                                 // Just Reset without saving makes more sense
}

void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeParams(0);
    uartPrint("\r\nRebooting...");
    systemReset(false);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)var->ptr;
        break;

    case VAR_INT8:
        value = *(int8_t *)var->ptr;
        break;

    case VAR_UINT16:
        value = *(uint16_t *)var->ptr;
        break;

    case VAR_INT16:
        value = *(int16_t *)var->ptr;
        break;

    case VAR_UINT32:
        value = *(uint32_t *)var->ptr;
        break;

    case VAR_FLOAT:
        printf("%s", ftoa(*(float *)var->ptr, buf));
        if (full)
        {
            printf(" %s", ftoa((float)var->min, buf));
            printf(" %s", ftoa((float)var->max, buf));
        }
        return;                                                         // return from case for float only
    }
    printf("%d", value);
    if (full)
        printf(" %d %d", var->min, var->max);
}

static void cliSetVar(const clivalue_t *var, const int32_t value)
{
    switch (var->type)
    {
    case VAR_UINT8:
    case VAR_INT8:
        *(char *)var->ptr = (char)value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        *(short *)var->ptr = (short)value;
        break;

    case VAR_UINT32:
        *(int *)var->ptr = (int)value;
        break;

    case VAR_FLOAT:
        *(float *)var->ptr = *(float *)&value;
        break;
    }
}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const    clivalue_t *val;
    char     *eqptr = NULL;
    int32_t  value = 0;
    float    valuef = 0;
    uint8_t  acc_hdwsave = cfg.acc_hdw;
    uint8_t  mag_gainsave = cfg.mag_gain;
    bool     needcal = false;

    len = strlen(cmdline);

    if (!len || (len == 1 && cmdline[0] == '*'))
    {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            printf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
        }
    }
    else if ((eqptr = strstr(cmdline, "=")))
    {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0)
            {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max)// here we compare the float value since... it should work, RIGHT?
                {
                    cliSetVar(val, valueTable[i].type == VAR_FLOAT ? *(uint32_t *)&valuef : value); // this is a silly dirty hack. please fix me later.
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                    if(acc_hdwsave != cfg.acc_hdw)                      // Check for hardwarechanges
                    {
                        if(cfg.acc_calibrated) needcal = true;          // Acc was calibrated needs recal now
                        cfg.angleTrim[ROLL] = cfg.angleTrim[PITCH] = 0.0f;
                        cfg.accZero[ROLL] = cfg.accZero[PITCH] = cfg.accZero[YAW] = 0.0f;
                        cfg.sens_1G  = 1;
                        cfg.acc_calibrated = 0;
                    }
                    if(mag_gainsave != cfg.mag_gain)
                    {
                        needcal = (cfg.mag_calibrated);                 // Mag was calibrated and gain is changed now needs recal
                        cfg.mag_calibrated = 0;
                    }
                    if (needcal) cliRecal();
                }
                else
                {
                    cliErrorMessage();                                  // uartPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        cliErrorMessage();                                              // uartPrint("ERR: Unknown variable name\r\n");
    }
}

static void EEPROMandFloppyStatusReport(void)
{
    printf("EEPROM:\r\n");
    printf("Total : %d B FreeFlash: %d B\r\n", cfg.size, FLASH_PAGE_SIZE * FLASH_PAGES_FORCONFIG - cfg.size);
    printf("Config: %d B\r\n", cfg.size - FDByteSize);
    printf("Floppy: %d B, %d Datasets of %d possible\r\n\r\n", FDByteSize, cfg.FDUsedDatasets, FDByteSize / sizeof(wp_t));
}

static void printxyzcalval(float *off)
{
    uint8_t i;
    char    X = 'X';
    for (i = 0; i < 3; i++) printf("\r\n%c Offset: %d", X++, (int32_t)off[i]);

}

static void cliStatus(char *cmdline)
{
    uint8_t  i, k;
    uint32_t mask;
    uint16_t tmpu16;
    char     X = 'X';

    printf("\r\nSystem Uptime: %d sec, Volt: %d * 0.1V (%dS battery)\r\n", currentTimeMS / 1000, vbat, batteryCellCount);
    mask = sensorsMask();
    printf("CPU %dMHz, detected sensors: ", (SystemCoreClock / 1000000));
    for (i = 0; ; i++)
    {
        if (sensorNames[i] == NULL) break;
        if (mask & (1 << i)) printf("%s ", sensorNames[i]);
    }
    printf("\r\nCycle Time: %d, I2C Errors: %d\r\n\r\n", (int16_t)AvgCyclTime, i2cGetErrorCounter());
    EEPROMandFloppyStatusReport();
    printf("SENSORS:");
    printf("\r\nGyro ");
    if(havel3g4200d) printf("L3G4200D");
    else printf("MPU");
    printf("\r\nActual");
    printxyzcalval(gyroZero);
    if (cfg.ShakyDataAvail)
    {
        printf("\r\nFallback");
        printxyzcalval(cfg.ShakyGyroZero);
    }
    printf("\r\nTemp: %d",(int32_t)telemTemperature1);
    if (sensors(SENSOR_ACC))
    {
        printf("\r\n\r\nAcc %s", accNames[accHardware]);
        printf("\r\nStatus: ");
        if(cfg.acc_calibrated)
        {
            printf("calibrated");
            printxyzcalval(cfg.accZero);
            printf("\r\n1G val: %d", cfg.sens_1G);
        }
        else cliRecal();
    }
    if (sensors(SENSOR_MAG))
    {
        printf("\r\n\r\nMag HMC5883");
        printf("\r\nStatus: ");
        if (cfg.mag_calibrated)
        {
            printf("calibrated");
            printxyzcalval(cfg.magZero);
            for (i = 0; i < 3; i++) printf("\r\n%c Gain*1000: %d", X++, (int32_t)(magCal[i] * 1000));
        }
        else cliRecal();
        printf("\r\nGain ");
        if (maggainok) printf("OK"); else printf("NOT OK");
    }
    if (sensors(SENSOR_BARO))
    {
        printf("\r\n\r\nBaro ");
        if(baro.baro_type == 1) printf("BMP085");
        else printf("MS5611");
        printf("\r\nTemp: %d",(int32_t)BaroActualTemp);
    }
    printf("\r\n\r\nSTATS:");
    if (sensors(SENSOR_BARO) || sensors(SENSOR_GPS))
    {
        if (sensors(SENSOR_GPS))
        {
            printf("\r\nGPS:");
            tmpu16 = (uint16_t)((float)cfg.MAXGPSspeed * 0.036f);
            printf("\r\nMax Dist: %d m", cfg.GPS_MaxDistToHome );
            printf("\r\nMax Speed: %dcm/s = %dKm/h", cfg.MAXGPSspeed, tmpu16);
        }
        if (sensors(SENSOR_BARO))
        {
            printf("\r\nHight:");
            printf("\r\nMax Alt AGL: %d m", cfg.MaxAltMeter);
            printf("\r\nMin Alt AGL: %d m", cfg.MinAltMeter);
        }
    }
    printf("\r\nMotor:\r\n");
    printf("Actual Range: %d - %d at %d Hz PWM.\r\n", cfg.esc_min, cfg.esc_max, cfg.esc_pwm);
    tmpu16 = (cfg.esc_max - cfg.esc_min) / 100;
    if(motorpercent[0])
    {
        k = min(NumberOfMotors, MAX_MONITORED_MOTORS);
        for (i = 0; i < k; i++) printf("Mot: %d Session Usage: %d%% Abs PWM: %d Rel to PWM range: %d%%\r\n", i + 1, motorpercent[i], motorabspwm[i],(motorabspwm[i] - cfg.esc_min) / tmpu16);
    } else printf("No Stats\r\n");
}

static void cliWpflush(char *cmdline)
{
    EEPROMandFloppyStatusReport();
    FloppyClear();
    printf("Flushing.\r\n\r\n");
    EEPROMandFloppyStatusReport();
}

static void cliVersion(char *cmdline)
{
    uartPrint(FIRMWARE);
}

void serialOSD(void)
{
#define LCDdelay 12
#define RcEndpoint 50
    uint8_t   input, lastinput = 0, brake = 0, brakeval = 0, speeduptimer = 0, exitLCD = 0;
    uint16_t  DatasetNr = 0;
    const clivalue_t *setval;
    LCDinit();
    printf(FIRMWAREFORLCD);                                             // Defined in mw.h
    LCDline2();
    printf("LCD Interface");
    delay(2000);
    LCDclear();
    LCDline1();
    printf("%s", valueTable[DatasetNr].name);                           // Display first item anyway (even if lcd ==0) no need for special attention
    LCDline2();
    setval = &valueTable[DatasetNr];
    cliPrintVar(setval, 0);
    while (!exitLCD)
    {
        if (DoGetRc50HzTimer())                                         // Start of 50Hz Loop Gathers all Rc Data
        {
            LED1_TOGGLE
            LED0_TOGGLE
            if (rcData[THROTTLE] < (cfg.rc_minchk + RcEndpoint) && rcData[PITCH] > (cfg.rc_maxchk - RcEndpoint))
            {
                if (rcData[YAW] > (cfg.rc_maxchk - RcEndpoint)) exitLCD = 1; // Quit don't save
                if (rcData[YAW] < (cfg.rc_minchk + RcEndpoint)) exitLCD = 2; // Quit and save
            }

            input = 0;
            if (!exitLCD)
            {
                if (rcData[PITCH] < (cfg.rc_minchk + RcEndpoint)) input = 1;
                if (rcData[PITCH] > (cfg.rc_maxchk - RcEndpoint)) input = 2;
                if (rcData[ROLL]  < (cfg.rc_minchk + RcEndpoint)) input = 3;
                if (rcData[ROLL]  > (cfg.rc_maxchk - RcEndpoint)) input = 4;
            }

            if (lastinput == input)                                     // Adjust Inputspeed
            {
                speeduptimer++;
                if (speeduptimer >= 100)
                {
                    speeduptimer = 99;
                    brakeval     = 8;
                }
                else brakeval = 17;
            }
            else
            {
                brakeval = 0;
                speeduptimer = 0;
            }
            lastinput = input;
            brake++;
            if (brake >= brakeval) brake = 0;
            else input = 0;

            switch (input)
            {
            case 0:
                break;
            case 1:                                                     // Down
                do                                                      // Search for next Dataset
                {
                    DatasetNr++;
                    if (DatasetNr == VALUE_COUNT) DatasetNr = 0;
                }
                while (!valueTable[DatasetNr].lcd);
                LCDclear();
                printf("%s", valueTable[DatasetNr].name);
                LCDline2();
                setval = &valueTable[DatasetNr];
                cliPrintVar(setval, 0);
                break;
            case 2:                                                     // UP
                do                                                      // Search for next Dataset
                {
                    if (!DatasetNr) DatasetNr = VALUE_COUNT;
                    DatasetNr--;
                }
                while (!valueTable[DatasetNr].lcd);
                LCDclear();
                printf("%s", valueTable[DatasetNr].name);
                LCDline2();
                setval = &valueTable[DatasetNr];
                cliPrintVar(setval, 0);
                break;
            case 3:                                                     // LEFT
                if (brakeval != 8) changeval(setval,-1);                // Substract within the limit
                else changeval(setval,-5);
                LCDline2();
                cliPrintVar(setval, 0);
                break;
            case 4:                                                     // RIGHT
                if (brakeval != 8) changeval(setval,1);                 // Add within the limit
                else changeval(setval,5);
                LCDline2();
                cliPrintVar(setval, 0);
                break;
            }
        }                                                               // End of 50Hz Loop
    }
    delay(500);
    LCDclear();
    printf(" Exit & Reboot ");
    LCDline2();
    switch (exitLCD)
    {
    case 1:
        printf(" NOT Saving");
        break;
    case 2:
        printf(".!.!.Saving.!.!.");
        writeParams(0);
        break;
    }
    delay(800);
    LCDoff();                                                           // Reset coming from LCD so reset it.
    systemReset(false);
}

static void changeval(const clivalue_t *var, const int8_t adder)
{
    int32_t value, maximum, minimum;
    float   valuef;

    maximum = var->max;
    minimum = var->min;
    switch (var->type)
    {
    case VAR_UINT8:
    case VAR_INT8:
        value = *(char *)var->ptr;
        value = constrain_int(value + adder, minimum, maximum);
        *(char *)var->ptr = (char)value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        value = *(short *)var->ptr;
        value = constrain_int(value + adder, minimum, maximum);
        *(short *)var->ptr = (short)value;
        break;

    case VAR_UINT32:
        value = *(int *)var->ptr;
        value = constrain_int(value + adder, minimum, maximum);
        *(int *)var->ptr = (int)value;
        break;

    case VAR_FLOAT:
        *(float *)&valuef = *(float *)var->ptr;
        valuef = constrain_flt(valuef + (float)adder/1000.0f, minimum, maximum);
        *(float *)var->ptr = *(float *)&valuef;
        break;
    }
}

static void LCDinit(void)                                               // changed Johannes
{
    if (!initI2cLCD(true))                                              // Will set i2cLCD
    {
        serialInit(9600);                                               // INIT LCD HERE
        LCDoff();
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x0C);                                                // Display ON
        delay(LCDdelay);
        uartWrite(0x7C);
        delay(LCDdelay);
        uartWrite(0x9D);                                                // 100% Brightness
        LCDclear();      
    }
}

void LCDoff(void)
{
    if (i2cLCD) i2c_clear_OLED();                                       // Johannes
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x08);                                                // LCD Display OFF
        delay(LCDdelay);
    }
}

static void LCDclear(void)                                              // clear screen, cursor line 1, pos 0
{
    if (i2cLCD) i2c_clear_OLED();                                       // Johannes
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x01);                                                // Clear
    }
    LCDline1();
}

static void LCDline1(void)                                              // Sets LCD Cursor to line 1 pos 0
{
    if (i2cLCD) i2c_clr_line(7);                                        // Johannes
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x80);                                                // Line #1 pos 0
        delay(LCDdelay);
    }
}

static void LCDline2(void)                                              // Sets LCD Cursor to line 2 pos 0
{
    if (i2cLCD) i2c_clr_line(8);                                        // Johannes
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0xC0);                                                // Line #2
        delay(LCDdelay);
        printf("               ");                                      // Clear Line #2
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0xC0);                                                // Line #2
        delay(LCDdelay);
    }
}

void cliProcess(void)
{
    static uint32_t bufferIndex = 0;
    char   cliBuffer[48], dummy;
    writeAllMotors(cfg.esc_moff);                                       // Set all motors to OFF just to be sure if user is messing in cli without saving
    memset(cliBuffer, 0, sizeof(cliBuffer));
    uartPrint("\r\nEntering CLI Mode, type 'exit' or 'save' to return, or 'help' \r\n\r\n");
    cliVersion(&dummy);
    uartPrint("\r\n\r\n");
    cliHelp(&dummy);
    cliPrompt();
    for(; ;)                                                            // CLI ENDLESS LOOP THAT SAVES STACK
    {
        while (uartAvailable())
        {
            uint8_t c = uartRead();
            if (c == '\t' || c == '?')
            {
                const clicmd_t *cmd, *pstart = NULL, *pend = NULL;      // do tab completion
                int i = bufferIndex;
                for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++)
                {
                    if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0)) continue;
                    if (!pstart) pstart = cmd;
                    pend = cmd;
                }
                if (pstart)                                             // Buffer matches one or more commands
                {
                    for (; ; bufferIndex++)
                    {
                        if (pstart->name[bufferIndex] != pend->name[bufferIndex]) break;
                        if (!pstart->name[bufferIndex])
                        {
                            cliBuffer[bufferIndex++] = ' ';             // Unambiguous -- append a space */
                            break;
                        }
                        cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                    }
                }
                if (!bufferIndex || pstart != pend)
                {
                    uartPrint("\r\033[K");                              // Print list of ambiguous matches
                    for (cmd = pstart; cmd <= pend; cmd++)
                    {
                        uartPrint(cmd->name);
                        uartWrite('\t');
                    }
                    cliPrompt();
                    i = 0;                                              // Redraw prompt
                }
                for (; i < bufferIndex; i++) uartWrite(cliBuffer[i]);
            }
            else if (!bufferIndex && c == 4)
            {
                cliExit(cliBuffer);
                return;
            }
            else if (c == 12)                                           // clear screen
            {
                uartPrint("\033[2J\033[1;1H");
                cliPrompt();
            }
            else if (bufferIndex && (c == '\n' || c == '\r'))           // enter pressed
            {
                clicmd_t *cmd = NULL;
                clicmd_t target;
                uartPrint("\r\n");
                cliBuffer[bufferIndex] = 0;                             // null terminate
                target.name  = cliBuffer;
                target.param = NULL;
                cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
                if (cmd) cmd->func(cliBuffer + strlen(cmd->name) + 1);
                else cliErrorMessage();
                memset(cliBuffer, 0, sizeof(cliBuffer));
                bufferIndex = 0;
                cliPrompt();
            }
            else if (c == 127)
            {
                if (bufferIndex)                                        // backspace
                {
                    cliBuffer[--bufferIndex] = 0;
                    uartPrint("\010 \010");
                }
            }
            else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126)
            {
                if (!bufferIndex && c == 32) continue;
                cliBuffer[bufferIndex++] = c;
                uartWrite(c);
            }
        }
    }
}

// ************************************************************************************************************
// TestScan on the I2C bus
// ************************************************************************************************************
#define MMA8452_ADDRESS     0x1C
#define HMC5883L_ADDRESS    0x1E  // 0xA
#define DaddyW_SONAR        0x20  // Daddy Walross Sonar
#define EagleTreePowerPanel 0x3B  // Eagle Tree Power Panel
#define OLD1_ADDRESS        0x3C  // OLED at address 0x3C in 7bit
#define OLD2_ADDRESS        0x3D  // OLED at address 0x3D in 7bit
#define ADXL345_ADDRESS     0x53
#define BMA180_ADDRESS      0x64  // don't respond ??
#define MPU6050_ADDRESS     0x68  // 0x75     or 0x68  0x15
#define L3G4200D_ADDRESS    0x68  // 0x0f
#define BMPandMS_ADDRESS    0x77  // 0xD0
#define MBandSRF_ADDRESS    0x70  // Devantech I2C SONAR, Standard address 0x70 (SRF02, SRF235, SRF08, SRF10)

/*
new May 15 2013 Johannes && Some stuff from me as well :)
*/
static void cliScanbus(char *cmdline)
{
    bool    ack, msbaro = false, L3G4200D = false;
    uint8_t address, nDevices = 0, sig, bufsnr[2];
    char    buf[20];

    printf("\r\nScanning I2C-Bus\r\n\r\n");
    i2cFastSpeed(false);                                // set I2C Standard mode
    for(address = 1; address < 127; address++ )
    {
        sig = 0;
        ack = i2cRead(address, address, 1, &sig);       // Do a blind read. Perhaps it's sufficient? Otherwise the hard way...
        if(!ack)                                        // Try to get ack with more force
        {
            switch(address)
            {
            case MMA8452_ADDRESS:
                i2cRead(MMA8452_ADDRESS, 0x0D, 1, &sig);
                if (sig == 0x2A || sig == 0x1A) ack = true;
                else ack = false;
                break;
            case DaddyW_SONAR:
                ack = i2cRead(DaddyW_SONAR, 0x32, 2, bufsnr);
                break;
            case BMPandMS_ADDRESS:
                ack = i2cRead(BMPandMS_ADDRESS, 0xA0, 1, &sig);  // Sig is irrelevant?
                msbaro = ack;
                break;
            case MPU6050_ADDRESS:
                i2cRead(MPU6050_ADDRESS, 0x0F, 1, &sig);
                if (sig == 0xD3)
                {
                    ack = true;
                    L3G4200D = true;
                }
                break;
            }
        }
        if (ack)
        {
            printf("I2C device at 0x");
            if (address<16) printf("0");
            printf("%x",address);
            switch (address)
            {
            case MMA8452_ADDRESS:                       // Detection altered
                strcpy(buf,"MMA8452");
                break;
            case HMC5883L_ADDRESS:
                strcpy(buf,"HMC5883L");
                break;
            case DaddyW_SONAR:                          // Summarize as "Sonar"
            case MBandSRF_ADDRESS:
                strcpy(buf,"Sonar");
                break;
            case EagleTreePowerPanel:                   // Summarize as "Display"
            case OLD1_ADDRESS:
            case OLD2_ADDRESS:
                strcpy(buf,"Display");
                break;
            case ADXL345_ADDRESS:                       // ADXL added
                strcpy(buf,"ADXL345");
                break;
            case BMA180_ADDRESS:                        // Sensor currently not supported by a driver
                strcpy(buf,"BMA180");
                break;
            case MPU6050_ADDRESS:
                if (L3G4200D) strcpy(buf,"L3G4200D");
                else strcpy(buf,"MPU3050/MPU6050");
                break;
            case BMPandMS_ADDRESS:
                if(msbaro) strcpy(buf,"MS5611");
                else strcpy(buf,"BMP085");
                break;
            default:                                    // Unknown case added
                strcpy(buf,"UNKNOWN TO ME");
                break;
            }
            printf(" probably %s \r\n",buf);
            nDevices++;
        }
        delay(50);
    }
    uartPrint("\r\n");
    i2cFastSpeed(true);                                 // set I2C I2C Fast mode
    if (!nDevices) printf("No I2C devices\r\n");
    else printf("%d Devices\r\n",nDevices);
}

// ************************************************************************************************************
// More or less Dumb passthrough for gps config - Hacky but working
// Maybe we miss a byte or something but GPS communication is checksummed, so GPS and Tool will keep it valid.
// ************************************************************************************************************
static void cliPassgps(char *cmdline)
{
    uint8_t  serbyte, i;
    uint32_t wantedbaud = 0;
    bool     HaveMTK;

    if (!feature(FEATURE_GPS))                                  // Don't ask for sensors(gps) here, it may not be initialized
    {
        printf("GPS not enabled!\r\n");
        return;
    }

    if (cfg.gps_type == 2 || cfg.gps_type == 3) HaveMTK = true;
    else HaveMTK = false;

    if (!strlen(cmdline))
    {
        printf("Need option\r\n");
        printf("Writing ubx conf with different baudsetting must fail.\r\n");
        printf("Set Baud of planned config now. Repower after ucenter.\r\n");
        printf("MTK go with '0', set type to NMEA, set Baud of FW.\r\n\r\n");
        printf("Select Ublox Options\r\n\r\n");
        printf("0 No Options. All GPS\r\n");
        printf("1 UBX Force Sgnlstrngth\r\n");
        printf("2 UBX 115K Baud\r\n");
        printf("3 UBX  57K Baud\r\n");
        printf("4 UBX  38K Baud\r\n");
        printf("5 UBX  19K Baud\r\n\r\n");
        if (HaveMTK) printf("Actual MTK 57K Baud.\r\n");        // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
        else
        {
            if (!cfg.gps_type) printf("Actual NMEA");
            else printf("Actual UBLOX");
            printf(" %d Baud.\r\n", cfg.gps_baudrate);
        }
    }
    else
    {
        if (HaveMTK && cmdline[0] != '0')
        {
            cliErrorMessage();
            return;
        }
        switch(cmdline[0])
        {
        case '0':
            if (HaveMTK) wantedbaud = 57600;
            else wantedbaud = cfg.gps_baudrate;
            break;
        case '1':
            wantedbaud = cfg.gps_baudrate;
            UblxSignalStrength();
            break;
        case '2':
            wantedbaud = 115200;
            break;
        case '3':
            wantedbaud = 57600;
            break;
        case '4':
            wantedbaud = 38400;
            break;
        case '5':
            wantedbaud = 19200;
            break;
        default:
            cliErrorMessage();
            return;
        }

        if(!HaveMTK)
        {
            if (wantedbaud == cfg.gps_baudrate) printf("Keeping GPS Baud: %d.", wantedbaud);
            else
            {
                printf("Setting %d Baud.", wantedbaud);
                UbloxForceBaud(wantedbaud);
            }
        }

        printf("\r\nProceeding. Close Terminal.");
        delay(2000);
        HaveNewGpsByte = false;
        serialInit(wantedbaud);                                 // Set USB Baudrate
        uart2Init(wantedbaud, GPSbyteRec, false);               // Set GPS Baudrate and callbackhandler
        i = 0;
        while (i < 3)
        {
            if (uartAvailable())
            {
                serbyte = uartRead();                           // Read from USB
                if (serbyte == '\r') i++;                       // Break out with 3 times RETURN
                else i = 0;
                uart2Write(serbyte);                            // Write to GPS
                while (!uart2TransmitEmpty())
                {
                    ;                                           // wait for GPS Byte to be send
                }
                LED1_TOGGLE
            }
            if (HaveNewGpsByte)
            {
                serbyte        = NewGPSByte;                    // Read from GPS
                HaveNewGpsByte = false;
                uartWrite(serbyte);                             // Write to USB
                LED0_TOGGLE
            }
        }
        uartPrint("Rebooting");
        systemReset(false);
    }
}

static void GPSbyteRec(uint16_t c)
{
    NewGPSByte = c;
    HaveNewGpsByte = true;
}

static void cliFlash(char *cmdline)
{
    printf("Close terminal & flash\r\n");
    systemReset(true);                                  // reboot to bootloader
}

static void cliErrorMessage(void)
{
    uartPrint("That was Harakiri, try 'help'");
}

static void cliRecal(void)
{
    uartPrint(" Needs Calibr.");
}

// MAVLINK STUFF AFFECTING CLI GOES HERE
bool baseflight_mavlink_send_paramlist(bool Reset)
{
    static  int16_t i = 0;
    uint8_t StrLength;
    float   value = 0;
    char    buf[17];                                    // Always send 16 chars reserve one zero byte
    mavlink_message_t msg;

    if(Reset)
    {
        i = 0;
        AllowProtocolAutosense = true;
        return true;                                    // Return status not relevant but true because the "Reset" was a success
    }
    AllowProtocolAutosense = false;                     // Block Autodetect during transmission
    if(i < 0 || i > (VALUE_COUNT - 1)) return true;     // Done with error but DONE
    memset (buf, 0, 17);                                // Fill with 0 For Stringtermination
    StrLength = min(strlen(valueTable[i].name), 16);    // Copy max 16 Bytes
    memcpy (buf, valueTable[i].name, StrLength);
    switch(valueTable[i].type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)valueTable[i].ptr;
        break;
    case VAR_INT8:
        value = *(int8_t *)valueTable[i].ptr;
        break;
    case VAR_UINT16:
        value = *(uint16_t *)valueTable[i].ptr;
        break;
    case VAR_INT16:
        value = *(int16_t *)valueTable[i].ptr;
        break;
    case VAR_UINT32:
        value = *(uint32_t *)valueTable[i].ptr;
        break;
    case VAR_FLOAT:
        value = *(float *)valueTable[i].ptr;
        break;
    }
    mavlink_msg_param_value_pack(MLSystemID, MLComponentID, &msg, buf, value, MAVLINK_TYPE_FLOAT, VALUE_COUNT, i);
		baseflight_mavlink_send_message(&msg);
    i++;
    if (i == VALUE_COUNT)
    {
        i = 0;
        AllowProtocolAutosense = true;                  // Allow Autodetection again
        return true;                                    // I am done
    }
    else return false;
}

bool baseflight_mavlink_set_param(mavlink_param_set_t *packet)
{
    mavlink_message_t msg;
    uint16_t i;
    uint8_t  StrLength, k;
    bool     returnval = false;
    float    value;

    if (strcmp((*packet).param_id, "") == 0) return false;   // Filter Shit Message here
    for (i = 0; i < VALUE_COUNT; i++)
    {
        StrLength = min(strlen(valueTable[i].name), 16);     // Compare max 16 Bytes
        for (k = 0; k < StrLength; k++) if(valueTable[i].name[k] - (*packet).param_id[k]) break; // Eat this strcmp !!
        if (k == StrLength)                                  // Strings match here
        {
            value = (*packet).param_value;
            if ((value > valueTable[i].max) || (value < valueTable[i].min)) return false;

            switch(valueTable[i].type)
            {
            case VAR_UINT8:
                *(uint8_t *)valueTable[i].ptr  = (uint8_t)value;
                break;
            case VAR_INT8:
                *(int8_t *)valueTable[i].ptr   = (int8_t)value;
                break;
            case VAR_UINT16:
                *(uint16_t *)valueTable[i].ptr = (uint16_t)value;
                break;
            case VAR_INT16:
                *(int16_t *)valueTable[i].ptr  = (int16_t)value;
                break;
            case VAR_UINT32:
                *(uint32_t *)valueTable[i].ptr = (uint32_t)value;
                break;
            case VAR_FLOAT:
                *(float *)valueTable[i].ptr = value;
                break;
            }
            mavlink_msg_param_value_pack(MLSystemID, MLComponentID, &msg, (*packet).param_id, value, (*packet).param_type, VALUE_COUNT, i); // Report parameter back if everything was fine.
		        baseflight_mavlink_send_message(&msg);
            returnval = true;
        }
    }
    return returnval;
}
