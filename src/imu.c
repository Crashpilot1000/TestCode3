#include "board.h"
#include "mw.h"

typedef struct fp_vector
{
    float X, Y, Z;
} t_fp_vector_def;

typedef union
{
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

float   accSmooth[3], ACC_speed[2] = { 0, 0 }, accADC[3], gyroADC[3], magADCfloat[3], angle[2] = { 0, 0 }; // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
float   BaroAlt, EstAlt, AltHold, vario;
int32_t sonarAlt;
int16_t BaroP, BaroI, BaroD;
bool    newbaroalt = false, GroundAltInitialized = false;

// **************
// gyro+acc IMU
// **************
float  gyroData[3] = { 0, 0, 0 };                         
static uint8_t SmoothingFactor[3]  = { 0, 0, 0 };
static bool    GyroSmoothing;
static void getEstimatedAttitude(void);

void imuInit(void)                                                            // Initialize & precalculate some values here
{
    if (cfg.gy_smrll || cfg.gy_smptc || cfg.gy_smyw)
    {
        SmoothingFactor[ROLL]  = cfg.gy_smrll;
        SmoothingFactor[PITCH] = cfg.gy_smptc;
        SmoothingFactor[YAW]   = cfg.gy_smyw;
        GyroSmoothing          = true;
    } else GyroSmoothing = false;
}

void computeIMU(void)
{
    static   float LastGyroSmooth[3] = { 0.0f, 0.0f, 0.0f };
    static   int16_t triywavg[4];
    static   uint8_t triywavgpIDX = 0;
    static   uint32_t prevT;
    uint8_t  axis, i;
    float    flttmp;
    if (MpuSpecial) GETMPU6050();
    else
    {
        gyro.temperature(&telemTemperature1);                                 // Read out gyro temperature
        Gyro_getADC();                                                        // Also feeds gyroData
        if (sensors(SENSOR_ACC)) ACC_getADC();
    }
    currentTime     = micros();
    FLOATcycleTime  = (float)constrain(currentTime - prevT, 1, 100000);       // 1us - 100ms
    ACCDeltaTimeINS = FLOATcycleTime * 0.000001f;                             // ACCDeltaTimeINS is in seconds now
    prevT           = currentTime;

    if(cfg.acc_calibrated) getEstimatedAttitude();                            // acc_calibrated just can turn true if acc present.
    
    if(cfg.mixerConfiguration == MULTITYPE_TRI && cfg.gy_smyw)                // Moving average for yaw in tri mode
    {
        triywavg[triywavgpIDX] = (int16_t)gyroData[YAW]; triywavgpIDX++;
        if (triywavgpIDX == 4) triywavgpIDX = 0;
        flttmp = 0;
        for (i = 0; i < 4; i++) flttmp += triywavg[i];
        gyroData[YAW] = flttmp * 0.25f;
    }

    if (GyroSmoothing)
    {
        for (axis = 0; axis < 3; axis++)
        {
            if (SmoothingFactor[axis] > 1)                                    // Circumvent useless action
            {
                flttmp               = (float)SmoothingFactor[axis];
                gyroData[axis]       = ((LastGyroSmooth[axis] * (flttmp - 1.0f)) + gyroData[axis]) / flttmp;
                LastGyroSmooth[axis] = gyroData[axis];
            }
        }
    }
}

static void RotGravAndMag(struct fp_vector *Grav, struct fp_vector *Mag, float *delta)// Rotate vectors according to the gyro delta
{
    struct    fp_vector v_tmp = *Grav;
    float     mat[3][3], cosx, sinx, cosy, siny, cosz, sinz, coszcosx, sinzcosx, coszsinx, sinzsinx;
    cosx      =  cosf(-delta[PITCH]);
    sinx      =  sinf(-delta[PITCH]);
    cosy      =  cosf(delta[ROLL]);
    siny      =  sinf(delta[ROLL]);
    cosz      =  cosf(delta[YAW]);
    sinz      =  sinf(delta[YAW]);
    coszcosx  =  cosz * cosx;
    sinzcosx  =  sinz * cosx;
    coszsinx  =  sinx * cosz;
    sinzsinx  =  sinx * sinz;
    mat[0][0] =  cosz * cosy;
    mat[0][1] =  sinz * cosy;
    mat[0][2] = -siny;
    mat[1][0] =  coszsinx * siny - sinzcosx;
    mat[1][1] =  sinzsinx * siny + coszcosx;
    mat[1][2] =  cosy * sinx;
    mat[2][0] =  coszcosx * siny + sinzsinx;
    mat[2][1] =  sinzcosx * siny - coszsinx;
    mat[2][2] =  cosy * cosx;
    Grav->X   =  v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    Grav->Y   =  v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    Grav->Z   =  v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
    if (cfg.mag_calibrated)                                                   // mag_calibrated can just be true if MAG present
    {
        v_tmp  = *Mag;                                                        // Proceed here if mag present and calibrated
        Mag->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];// saves recalculating matrix values
        Mag->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
        Mag->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
    }
}

// *Somehow* modified by me..
#define OneGcmss        980.665f                                              // 1G in cm/(s*s)
static void getEstimatedAttitude(void)
{
    static t_fp_vector EstG, EstM;
    static float    cms[3] = {0.0f, 0.0f, 0.0f}, Tilt_25deg, AccScaleCMSS;
    static float    INV_GY_CMPF, INV_GY_CMPFM, ACC_GPS_RC, ACC_ALT_RC, ACC_RC;
    static uint32_t UpsDwnTimer, SQ1G;
    static bool     init = false;
    float           tmp[3], DeltGyRad[3], rollRAD, pitchRAD;
    float           Norm, A, B, cr, sr, cp, sp, spcy, spsy, accycp, CmsFac;
    uint8_t         i;
    uint32_t        tmpu32 = 0;
    if(!init)                                                                 // Setup variables & constants
    {
        init = true;
        AccScaleCMSS = OneGcmss / (float)cfg.sens_1G;                         // scale to cm/ss
        SQ1G         = (int32_t)cfg.sens_1G * (int32_t)cfg.sens_1G;
        Tilt_25deg   = cosf(25.0f * RADX);
        INV_GY_CMPF  = 1.0f / (float)(cfg.gy_gcmpf + 1);                      // Default 400
        INV_GY_CMPFM = 1.0f / (float)(cfg.gy_mcmpf + 1);                      // Default 200
        if(!cfg.acc_lpfhz) cfg.acc_lpfhz = 0.001f;                            // Avoid DivByZero
        ACC_RC       = RCconstPI / cfg.acc_lpfhz;                             // Default 0,536 Hz
        ACC_ALT_RC   = RCconstPI / (float)cfg.acc_altlpfhz;                   // Default 10 Hz
        ACC_GPS_RC   = RCconstPI / (float)cfg.acc_gpslpfhz;                   // Default 5 Hz

        for (i = 0; i < 3; i++)                                               // Preset some values to reduce runup time
        {
            accSmooth[i] = accADC[i];
            EstG.A[i]    = accSmooth[i];
            EstM.A[i]    = magADCfloat[i];                                    // Using /2 for more stability
        }
    }
    CmsFac = ACCDeltaTimeINS * AccScaleCMSS;                                  // We need that factor for INS below
    tmp[0] = ACCDeltaTimeINS / (ACC_RC + ACCDeltaTimeINS);
    for (i = 0; i < 3; i++)
    {
        accSmooth[i] += tmp[0] * (accADC[i] - accSmooth[i]);                  // For Gyrodrift correction
        DeltGyRad[i]  = (ACCDeltaTimeINS * gyroADC[i]) * 0.0625f;             // gyroADC delivered in 16 * rad/s
        tmpu32       += (int32_t)accSmooth[i] * (int32_t)accSmooth[i];        // Accumulate ACC magnitude there
    }
    RotGravAndMag(&EstG.V, &EstM.V, DeltGyRad);                               // Rotate Grav & Mag together to avoid doublecalculation
    tmpu32 = (tmpu32 * 100) / SQ1G;                                           // accMag * 100 / ((int32_t)acc_1G * acc_1G);
    if (72 < tmpu32 && tmpu32 < 133)                                          // Gyro drift correct between 0.85G - 1.15G
    {
        for (i = 0; i < 3; i++) EstG.A[i] = (EstG.A[i] * (float)cfg.gy_gcmpf + accSmooth[i]) * INV_GY_CMPF;
    }
    tmp[0]       = EstG.A[0] * EstG.A[0] + EstG.A[2] * EstG.A[2];             // Start Angle Calculation. tmp[0] is used for heading below
    Norm         = sqrtf(tmp[0] + EstG.A[1] * EstG.A[1]);
    if(!Norm) return;                                                         // Should never happen but break here to prevent div-by-zero-evil
    Norm         = 1.0f / Norm;
    rollRAD      =  atan2f(EstG.A[0] * Norm, EstG.A[2] * Norm);               // Norm seems to be obsolete, but testing shows different result :)
    pitchRAD     = -asinf(constrain(EstG.A[1] * Norm, -1.0f, 1.0f));          // Ensure range, eliminate rounding stuff that might occure.
    cr           = cosf(rollRAD);
    sr           = sinf(rollRAD);
    cp           = cosf(pitchRAD);
    sp           = sinf(pitchRAD);
    TiltValue    = cr * cp;                                                   // We do this correctly here
    angle[ROLL]  = SpecialIntegerRoundUp( rollRAD  * RADtoDEG10);
    angle[PITCH] = SpecialIntegerRoundUp(-pitchRAD * RADtoDEG10);
    if (TiltValue >= 0)   UpsDwnTimer = 0;
    else if(!UpsDwnTimer) UpsDwnTimer = currentTime + 20000;                  // Use 20ms Timer here to make absolutely sure we are upsidedown
    if (UpsDwnTimer && currentTime > UpsDwnTimer) UpsideDown = true;
    else UpsideDown = false;
    if (TiltValue > Tilt_25deg) f.SMALL_ANGLES_25 = 1;
    else f.SMALL_ANGLES_25 = 0;
    if (cfg.mag_calibrated)                                                   // mag_calibrated can just be true if MAG present
    {
        if(HaveNewMag)                                                        // Only do Complementary filter when new MAG data are available
        {
            HaveNewMag = false;
            for (i = 0; i < 3; i++) EstM.A[i] = (EstM.A[i] * (float)cfg.gy_mcmpf + magADCfloat[i]) * INV_GY_CMPFM;
        }
        A = EstM.A[1] * tmp[0] - (EstM.A[0] * EstG.A[0] + EstM.A[2] * EstG.A[2]) * EstG.A[1];// Mwii method is more precise (less rounding errors)
        B = EstM.A[2] * EstG.A[0] - EstM.A[0] * EstG.A[2];
        heading = wrap_180(atan2f(B, A * Norm) * RADtoDEG + magneticDeclination);
        if (sensors(SENSOR_GPS) && !UpsideDown)
        {
            tmp[0]    = heading * RADX;                                       // Do GPS INS rotate ACC X/Y to earthframe no centrifugal comp. yet
            cos_yaw_x = cosf(tmp[0]);                                         // Store for general use
            sin_yaw_y = sinf(tmp[0]);
            spcy      = sp * cos_yaw_x;
            spsy      = sp * sin_yaw_y;
            accycp    = cp * accADC[1];
            tmp[0]    = accycp * cos_yaw_x + (sr * spcy - cr * sin_yaw_y) * accADC[0] + ( sr * sin_yaw_y + cr * spcy) * accADC[2]; // Rotate raw acc here
            tmp[1]    = accycp * sin_yaw_y + (cr * cos_yaw_x + sr * spsy) * accADC[0] + (-sr * cos_yaw_x + cr * spsy) * accADC[2];
            tmp[2]    = ACCDeltaTimeINS / (ACC_GPS_RC + ACCDeltaTimeINS);
            for (i = 0; i < 2; i++)
            {
                cms[i]       += tmp[2] * (tmp[i] * CmsFac - cms[i]);
                ACC_speed[i] -= cms[i];                                       //cm/s N+ E+
            }
        }
    }
    if(GroundAltInitialized && !UpsideDown)                                   // GroundAltInitialized can just be true if baro present
    {
        tmp[0]  = ((-sp) * accADC[1] + sr * cp * accADC[0] + cp * cr * accADC[2]) - (float)cfg.sens_1G;
        cms[2] += (ACCDeltaTimeINS / (ACC_ALT_RC + ACCDeltaTimeINS)) * (tmp[0] * CmsFac - cms[2]);
        vario  += cms[2] * constrain(TiltValue, 0.5f, 1.0f);                  // Empirical hightdrop reduction on tilt.
    }
}

#ifdef BARO
///////////////////////////////////////////////
//Crashpilot1000 Mod getEstimatedAltitude ACC//
///////////////////////////////////////////////
#define VarioTabsize 8
void getEstimatedAltitude(void)
{
    static int8_t   VarioTab[VarioTabsize];
    static uint8_t  Vidx = 0, IniStep = 0, IniCnt = 0;
    static uint32_t LastBarotime = 0;
    static float    AvgHz = 0.0f, LastEstAltBaro = 0.0f, SNRcorrect, SNRavg = 0.0f;
    float           NewVal, EstAltBaro;
    uint32_t        TimeTemp;
    uint8_t         i;

    if (!GroundAltInitialized)
    {
        if (newbaroalt)
        {
            TimeTemp     = micros();
            NewVal       = (float)(TimeTemp - LastBarotime);
            LastBarotime = TimeTemp;
            switch(IniStep)                                                   // Casemachine here for further extension
            {
            case 0:
                IniCnt++;
                if(IniCnt == 50)                                              // Waste 50 Cycles to let things (buffers) settle then ini some vars and proceed
                {
                    for (i = 0; i < VarioTabsize; i++) VarioTab[i] = 0;
                    EstAlt = GroundAlt = vario = 0;
                    IniCnt = 0;
                    IniStep++;
                }
                break;
            case 1:
                GroundAlt += BaroAlt;
                AvgHz     += NewVal;
                IniCnt++;
                if (IniCnt == 50)                                             // Gather 50 values
                {
                    GroundAlt *= 0.02f;
                    AvgHz      = 50000000.0f / AvgHz;                         // Calculate Average Hz here since we skip Baro temp readout every 2nd read
                    GroundAltInitialized = true;
                    SonarStatus = 0;
                }
                break;
            }
        }
    }
    else
    {
        if (sensors(SENSOR_SONAR))
        {
            if (SonarStatus) NewVal = sonarAlt;
            switch(SonarStatus)
            {
            case 0:
                SNRavg  = 0.0f;
                IniStep = 0;
                break;
            case 1:
                if (!IniStep)
                {
                    IniStep = 1;
                    SNRavg  = NewVal;
                }
                else SNRavg += 0.2f * (NewVal - SNRavg);                      // Adjust Average during accepttimer (ca. 550ms so ca. 20 cycles)
                SNRcorrect = EstAlt + GroundAlt - SNRavg;                     // Calculate baro/sonar displacement on 1st contact
                break;
            case 2:
                if (newbaroalt) BaroAlt = (SNRcorrect + NewVal) * cfg.snr_cf + BaroAlt * (1 - cfg.snr_cf); // Set weight / make transition smoother
                break;
            }
        }
        EstAlt += vario * ACCDeltaTimeINS;
        if (newbaroalt)
        {
            EstAltBaro     = BaroAlt - GroundAlt;
            VarioTab[Vidx] = constrain((int16_t)(EstAltBaro - LastEstAltBaro), -127, 127);
            Vidx++;
            if (Vidx == VarioTabsize) Vidx = 0;
            LastEstAltBaro = EstAltBaro;
            NewVal = 0;
            for (i = 0; i < VarioTabsize; i++) NewVal += (float)VarioTab[i];
            NewVal = (NewVal * AvgHz)/(float)VarioTabsize;
            vario  = vario  * cfg.accz_vcf + NewVal     * (1.0f - cfg.accz_vcf);
            EstAlt = EstAlt * cfg.accz_acf + EstAltBaro * (1.0f - cfg.accz_acf);
            if (cfg.bar_dbg)
            {
                debug[0] = EstAltBaro * 10;
                debug[1] = EstAlt     * 10;
                debug[2] = NewVal;
                debug[3] = vario;
            }
        }
    }
}

void getAltitudePID(void)                                                     // I put this out of getEstimatedAltitude seems logical
{
    float ThrAngle;
    ThrAngle = constrain(TiltValue * 100.0f, 0, 100.0f);
    if (ThrAngle < 40.0f || UpsideDown)                                       // Don't do BaroPID if copter too tilted
    {
        BaroP = BaroI = BaroD = 0.0f;
    }
    else
    {
        BaroP = (float)cfg.P8[PIDALT] * (AltHold - EstAlt) * 0.005f;
        BaroI = (float)cfg.I8[PIDALT] * vario * 0.02f;                        // BaroI = (int16_t)(((float)cfg.I8[PIDALT] * vario / ACCDeltaTimeINS) * 0.00006f); // That is actually a "D"
        BaroD = (float)cfg.D8[PIDALT] * (100.0f - ThrAngle) * 0.04f;          // That is actually the Tiltcompensation
    }
}
#endif
