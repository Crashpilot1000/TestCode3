#include "board.h"
#include "mw.h"

uint8_t useServo = 0;
int16_t motor[MAX_MOTORS];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

static motorMixer_t currentMixer[MAX_MOTORS];

static const motorMixer_t mixerTri[] =
{
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] =
{
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

static const motorMixer_t mixerQuadX[] =
{
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

static const motorMixer_t mixerBi[] =
{
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};

static const motorMixer_t mixerY6[] =
{
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

/*
OLD
static const motorMixer_t mixerHex6P[] =
{
    { 1.0f, -1.0f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -1.0f, -0.866025f, -1.0f },     // FRONT_R
    { 1.0f,  1.0f,  0.866025f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f, -0.866025f,  1.0f },     // FRONT
    { 1.0f,  0.0f,  0.866025f, -1.0f },     // REAR
};
*/
//NEW commit 289
static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};

static const motorMixer_t mixerY4[] =
{
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

/*
static const motorMixer_t mixerHex6X[] =
{
    { 1.0f, -0.866025f,  1.0f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  1.0f, -1.0f },     // REAR_L
    { 1.0f,  0.866025f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f, -0.866025f,  0.0f, -1.0f },     // RIGHT
    { 1.0f,  0.866025f,  0.0f,  1.0f },     // LEFT
};
*/
//NEW commit 289
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};

static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerOctoX8[] =
{
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] =
{
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] =
{
    { 1.0f,  1.0f, -0.5f,  1.0f },          // MIDFRONT_L
    { 1.0f, -0.5f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  0.5f,  1.0f },          // MIDREAR_R
    { 1.0f,  0.5f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  0.5f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -0.5f, -1.0f },          // MIDFRONT_R
    { 1.0f, -0.5f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  0.5f, -1.0f },          // MIDREAR_L
};

static const motorMixer_t mixerVtail4[] =
{
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -0.0f },          // FRONT_L
};

// Keep this synced with MultiType struct in mw.h!
/*
OLD
const mixer_t mixers[] =
{
    //    Mo Se Mixtable
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
    { 2, 1, mixerBi },             // MULTITYPE_BI
    { 0, 1, NULL },                // * MULTITYPE_GIMBAL
    { 6, 0, mixerY6 },             // MULTITYPE_Y6
    { 6, 0, mixerHex6P },          // MULTITYPE_HEX6
    { 1, 1, NULL },                // * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MULTITYPE_OCTOFLATX
    { 1, 1, NULL },                // * MULTITYPE_AIRPLANE
    { 0, 1, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, 1, NULL },                // * MULTITYPE_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // MULTITYPE_VTAIL4
    { 0, 0, NULL },                // MULTITYPE_CUSTOM
};
*/

  const mixer_t mixers[] = {       // Motors, UseServo, Mixtable
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
    { 2, 1, mixerBi },             // MULTITYPE_BI
    { 0, 1, NULL },                // * MULTITYPE_GIMBAL
    { 6, 0, mixerY6 },             // MULTITYPE_Y6
    { 6, 0, mixerHex6P },          // MULTITYPE_HEX6
    { 1, 1, NULL },                // * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MULTITYPE_OCTOFLATX
    { 1, 1, NULL },                // * MULTITYPE_AIRPLANE
    { 0, 1, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, 1, NULL },                // * MULTITYPE_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // MULTITYPE_VTAIL4
    { 6, 0, mixerHex6H },          // MULTITYPE_HEX6H
    { 0, 1, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { 0, 0, NULL },                // MULTITYPE_CUSTOM
};

void mixerInit(void)
{
    uint8_t i;
    NumberOfMotors = 0;
    useServo = mixers[cfg.mixerConfiguration].useServo;                           // enable servos for mixes that require them. note, this shifts motor counts.
    if (feature(FEATURE_SERVO_TILT)) useServo = 1;                                // if we want camstab/trig, that also enables servos, even if mixer doesn't

    if (cfg.mixerConfiguration == MULTITYPE_CUSTOM)
    {
        for (i = 0; i < MAX_MOTORS; i++)                                          // load custom mixer into currentMixer
        {
            if (cfg.customMixer[i].throttle == 0.0f) break;                       // check if done
            currentMixer[i] = cfg.customMixer[i];
            NumberOfMotors++;
        }
    }
    else
    {
        NumberOfMotors = mixers[cfg.mixerConfiguration].numberMotor;
        if (mixers[cfg.mixerConfiguration].motor)                                 // copy motor-based mixers
        {
            for (i = 0; i < NumberOfMotors; i++) currentMixer[i] = mixers[cfg.mixerConfiguration].motor[i];
        }
    }
}

void mixerLoadMix(int index)
{
    int i;
    index++;                                                                      // we're 1-based
    for (i = 0; i < MAX_MOTORS; i++) cfg.customMixer[i].throttle = 0.0f;          // clear existing

    if (mixers[index].motor != NULL)                                              // do we have anything here to begin with?
    {
        for (i = 0; i < mixers[index].numberMotor; i++) cfg.customMixer[i] = mixers[index].motor[i];
    }
}

void writeServos(void)
{
    static uint32_t yawarmdelaytimer = 0;
    if (!useServo) return;
    switch (cfg.mixerConfiguration)
    {
    case MULTITYPE_BI:
        pwmWriteServo(0, servo[4]);
        pwmWriteServo(1, servo[5]);
        break;

    case MULTITYPE_TRI:
        if (!cfg.tri_ydel)
        {
            pwmWriteServo(0, servo[5]);                                           // like always
        }
        else
        {
            if (f.ARMED)
            {
                if (!yawarmdelaytimer) yawarmdelaytimer = currentTimeMS + (uint32_t)cfg.tri_ydel;
                if (currentTimeMS >= yawarmdelaytimer) pwmWriteServo(0, servo[5]);// like always
                else pwmWriteServo(0, cfg.tri_ymid);                              // Give middlesignal to yaw servo when disarmed
            }
            else
            {
                yawarmdelaytimer = 0;
                pwmWriteServo(0, cfg.tri_ymid);                                   // Give middlesignal to yaw servo when disarmed
            }
        }
        break;

    case MULTITYPE_FLYING_WING:
        pwmWriteServo(0, servo[3]);
        pwmWriteServo(1, servo[4]);
        break;
        
    case MULTITYPE_GIMBAL:
        pwmWriteServo(0, servo[0]);
        pwmWriteServo(1, servo[1]);
        break;

    case MULTITYPE_AIRPLANE:
        break;

    default:                                                                      // Two servos for SERVO_TILT, if enabled
        if (feature(FEATURE_SERVO_TILT))
        {
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
        }
        break;
    }
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;
    for (i = 0; i < NumberOfMotors; i++)
    {
        motor[i] = mc;                           // Sends commands to all motors
        pwmWriteMotor(i, mc);
    }
}

void mixTableAndWriteMotors(void)
{
    int32_t  Overshoot, Range, Range85;
    uint16_t aux[2];
    int16_t  MaxMotor, MinMotor, limit;
    uint8_t  i, offset;

    switch (cfg.mixerConfiguration)                                               // airplane / servo mixes
    {
    case MULTITYPE_BI:
        servo[4] = constrain_int(1500 + (cfg.tri_ydir * axisPID[YAW]) + axisPID[PITCH], 1020, 2000); // LEFT
        servo[5] = constrain_int(1500 + (cfg.tri_ydir * axisPID[YAW]) - axisPID[PITCH], 1020, 2000); // RIGHT
        break;

    case MULTITYPE_TRI:
        servo[5] = constrain_int(cfg.tri_ymid + cfg.tri_ydir * axisPID[YAW], cfg.tri_ymin, cfg.tri_ymax); // REAR
        break;

    case MULTITYPE_GIMBAL:
        servo[0] = constrain_int(cfg.gbl_pmd + (int16_t)((float)cfg.gbl_pgn * angle[PITCH] * 0.0625f) + rcCommand[PITCH], cfg.gbl_pmn, cfg.gbl_pmx);
        servo[1] = constrain_int(cfg.gbl_rmd + (int16_t)((float)cfg.gbl_rgn * angle[ROLL]  * 0.0625f) + rcCommand[ROLL] , cfg.gbl_rmn, cfg.gbl_rmx);
        break;

    case MULTITYPE_AIRPLANE:
        break;
        
   case MULTITYPE_FLYING_WING:
        if (!f.ARMED) servo[7] = cfg.rc_minchk;
        else servo[7] = constrain_int(rcCommand[THROTTLE], cfg.esc_min, cfg.esc_max);
        motor[0] = servo[7];
        if (f.PASSTHRU_MODE)                                                      // do not use sensors for correction, simple 2 channel mixing
        {
            servo[3] = cfg.pitch_direction_l * (rcData[PITCH] - cfg.rc_mid) + cfg.roll_direction_l * (rcData[ROLL] - cfg.rc_mid);
            servo[4] = cfg.pitch_direction_r * (rcData[PITCH] - cfg.rc_mid) + cfg.roll_direction_r * (rcData[ROLL] - cfg.rc_mid);
        }
        else                                                                      // use sensors to correct (gyro only or gyro + acc)
        {
            servo[3] = cfg.pitch_direction_l * axisPID[PITCH] + cfg.roll_direction_l * axisPID[ROLL];
            servo[4] = cfg.pitch_direction_r * axisPID[PITCH] + cfg.roll_direction_r * axisPID[ROLL];
        }
        servo[3] = constrain_int(servo[3] + cfg.wing_left_mid, cfg.wing_left_min, cfg.wing_left_max);
        servo[4] = constrain_int(servo[4] + cfg.wing_right_mid, cfg.wing_right_min, cfg.wing_right_max);
        break;
    }

    if (feature(FEATURE_SERVO_TILT))                                              // do camstab
    {
        if ((cfg.gbl_flg & GIMBAL_NORMAL) || (cfg.gbl_flg & GIMBAL_TILTONLY)) aux[0] = rcData[AUX3] - cfg.rc_mid;
        else aux[0] = 0;
        if (!(cfg.gbl_flg & GIMBAL_DISABLEAUX34)) aux[1] = rcData[AUX4] - cfg.rc_mid;
        else aux[1] = 0;
        servo[0] = cfg.gbl_pmd + aux[0];
        servo[1] = cfg.gbl_rmd + aux[1];
        if (rcOptions[BOXCAMSTAB])
        {
            if (cfg.gbl_flg & GIMBAL_MIXTILT)
            {
                servo[0] -= (int16_t)(((float)(-cfg.gbl_pgn) * angle[PITCH] * 0.0625f) - (float)cfg.gbl_rgn * angle[ROLL] * 0.0625f);
                servo[1] += (int16_t)(((float)(-cfg.gbl_pgn) * angle[PITCH] * 0.0625f) + (float)cfg.gbl_rgn * angle[ROLL] * 0.0625f);
            }
            else
            {
                servo[0] += (int16_t)((float)cfg.gbl_pgn * angle[PITCH] * 0.0625f);
                servo[1] += (int16_t)((float)cfg.gbl_rgn * angle[ROLL]  * 0.0625f);
            }
        }
        servo[0] = constrain_int(servo[0], cfg.gbl_pmn, cfg.gbl_pmx);
        servo[1] = constrain_int(servo[1], cfg.gbl_rmn, cfg.gbl_rmx);
    }

    if (cfg.gbl_flg & GIMBAL_FORWARDAUX)
    {
        if (feature(FEATURE_SERVO_TILT)) offset = 2;
        else offset = 0;
        for (i = 0; i < 4; i++) pwmWriteServo(i + offset, rcData[AUX1 + i]);
    }

    if (feature(FEATURE_LED) && (cfg.LED_Type == 1))
    {
        if (feature(FEATURE_SERVO_TILT)) pwmWriteServo(2, LED_Value);
        else pwmWriteServo(0, LED_Value);
    }

    if (!f.ARMED)                                                                 // If not armed
    {
        writeAllMotors(cfg.esc_moff);                                             // Stop all motors
        return;                                                                   // And stop the rest from happening
    }

    Range    = cfg.esc_max - cfg.esc_min;
    Range85  = (Range * 109) >> 7;                                                // ca. 85% overshoot range for Maximal motor value. Note: This could be a constant as well
    MinMotor = cfg.esc_min;
    MaxMotor = cfg.esc_max;
    if (NumberOfMotors > 1)                                                       // motors for non-servo mixes
    {
        limit = cfg.esc_max + Range85;
        for (i = 0; i < NumberOfMotors; i++)
        {
            motor[i] = (float)rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + cfg.tri_ydir * axisPID[YAW] * currentMixer[i].yaw;
            if (motor[i] < MinMotor) MinMotor = motor[i];
            motor[i] = constrain(motor[i], cfg.esc_min, limit);
            if (motor[i] > MaxMotor) MaxMotor = motor[i];
        }
        limit    = max(cfg.esc_min - (Range85 >> 1), 0);                          // Use Halfrange85limit for Minimal motor value influence
        MinMotor = max(MinMotor, limit);
    }

    if (MaxMotor > cfg.esc_max)
    {
        if(cfg.esc_nwmx) Overshoot = (Range << 10) / (int32_t)(MaxMotor - MinMotor);// 0 = mwii style, 1 = scaled handling of maxthrottlesituations
        else Overshoot = MaxMotor - cfg.esc_max;
    }
    else Overshoot = 0;
    for (i = 0; i < NumberOfMotors; i++)
    {
        if(Overshoot)
        {
            if(cfg.esc_nwmx) motor[i] = cfg.esc_min + (((int32_t)(motor[i] - cfg.esc_min) * Overshoot) >> 10);
            else motor[i] -= Overshoot;
            motor[i] = constrain(motor[i], cfg.esc_min, cfg.esc_max);
        }
        if ((rcData[THROTTLE]) < cfg.rc_minchk)
        {
            if(!cfg.rc_motor) motor[i] = cfg.esc_min;                             // cfg.rc_motor [0-2] Behaviour when thr < rc_minchk: 0= minthrottle no regulation, 1= minthrottle&regulation, 2= Motorstop 
            else if(cfg.rc_motor == 2) motor[i] = cfg.esc_moff;
        }
        pwmWriteMotor(i, motor[i]);
    }
}
