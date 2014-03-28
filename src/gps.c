#include "board.h"
#include "mw.h"

#define PhStickCenterTimeout  350         // Defines the time in ms when we consider the sticks really back to center
#define PhSettleTimeout       410         // Defines the time in ms, where actual speed must be below settlespeed, to consider a settled copter

// NAVIGATION & Crosstrack Common Variables
int32_t   target_bearing;                 // target_bearing is where we should be heading
uint32_t  wp_distance;
float     waypoint_speed_gov;             // used for slow speed wind up when start navigation;
int32_t   nav_bearing;                    // This is the angle from the copter to the "next_WP" location  with the addition of Crosstrack error in degrees * 100 // Crosstrack eliminated left here on purpose
int16_t   nav_takeoff_heading;            // saves the heading at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
int32_t   original_target_bearing;        // deg*100, The original angle to the next_WP when the next_WP was set Also used to check when we pass a WP
float     LocError[2];                    // updated after GPS read - 5-10hz Error in cm from target

// PH Variables
static bool PH1stRun;
static bool PHuseGPSWP;
static void GPS_HzSandbox(void);

bool GPS_alltime(void)
{
    bool result;
    if (GPS_FIX && GPS_numSat >= 5)                                             // Do gps stuff with at least 5 Sats
    {
        GPS_calc_velocity();                                                    // Heart of the gps ins, called every time
        GPS_HzSandbox();                                                        // Just in case we want a Hz sandbox here in the future
        nav[LAT] = constrain(nav[LAT], -maxbank100, maxbank100);
        nav[LON] = constrain(nav[LON], -maxbank100, maxbank100);
        result = true;
    }
    else
    {
        GPS_reset_nav();
        f.GPS_HOME_MODE = 0;
        f.GPS_HOLD_MODE = 0;
        f.GPS_AUTO_MODE = 0;
        nav_mode  = NAV_MODE_NONE;
        ph_status = PH_STATUS_NONE;
        wp_status = WP_STATUS_NONE;
        result = false;
    }
    return result;
}

static void GPS_HzSandbox(void)
{
    static uint32_t PosHoldBlindTimer = 0, PhTimer1 = 0;
    static int32_t  AvgSpeed = 0;
    static bool     PHtoofast, PHChange;
    static uint8_t  PHcascade = 0, LastGPSupdateState;
    uint16_t        speed;

    if (!f.ARMED) f.GPS_FIX_HOME = 0;
    if (!f.GPS_FIX_HOME)                                                        // Do relative to home stuff for gui etc
    {
        if (f.ARMED)
        {
            GPS_calc_longitude_scaling(true);
            GPS_home[LAT] = Real_GPS_coord[LAT];
            GPS_home[LON] = Real_GPS_coord[LON];
            nav_takeoff_heading = heading;                                      // save takeoff heading
            f.GPS_FIX_HOME = 1;
        }
        else
        {
            GPS_distanceToHome  = 0;
            GPS_directionToHome = 0;
            GPS_home[LAT]       = 0;
            GPS_home[LON]       = 0;
        }
    }
    else                                                                        // Do dist to Home Stuff here
    {
        GPS_distance_cm_bearing(&GPS_home[LAT], &GPS_home[LON], &GPS_distanceToHome, &GPS_directionToHome);
        GPS_distanceToHome  /= 100;                                             // Back to meters
        GPS_directionToHome /= 100;                                             // Back to degrees
        if (GPS_distanceToHome > cfg.GPS_MaxDistToHome ) cfg.GPS_MaxDistToHome = GPS_distanceToHome; // Stats
        if (GPS_speed > cfg.MAXGPSspeed) cfg.MAXGPSspeed = GPS_speed;
    }

    if (DoingGPS())
    {
        switch (nav_mode)
        {
        case NAV_MODE_POSHOLD:
            if (PH1stRun)
            {
                PHcascade           = 1;                                        // Always Trigger worst case szenario
                PH1stRun            = false;                                    // Don't do this again ...
                PHChange            = false;
                PosHoldBlindTimer   = 0;                                        // Reset Timer
                ph_status           = PH_STATUS_NONE;
                GPS_reset_nav();                                                // Reset nav Speedvector as well
            }

            if (rcCommand[PITCH] || rcCommand[ROLL])                            // Ph Override
            {
                PosHoldBlindTimer = 0;
                PHcascade         = 0;                                          // Reset cascade and WP
                PHChange          = true;
                PH1stRun          = false;
            }
            else                                                                // Sticks are center
            {
                if (PHChange)                                                   // Are we coming from a change? Stick back to neutral, set timer before accepting it
                {
                    if (!PosHoldBlindTimer) PosHoldBlindTimer = currentTimeMS + PhStickCenterTimeout; // Set 300ms timeout
                    else if (currentTimeMS >= PosHoldBlindTimer) PH1stRun = true;// Override done? Re - initialize PH on next run
                }
            }

            LocError[LAT] = LocError[LON] = 0;
            switch(PHcascade)
            {
            case 0:                                                             // Manual oversteer case
                PHtoofast  = true;
                PHuseGPSWP = false;                                             // Forget target Position
                ph_status  = PH_STATUS_NONE;
                GPS_reset_nav();                                                // Reset nav Speedvector as well
                break;
            case 1:                                                             // Figure out current speed & brake
                ph_status = PH_STATUS_WORKING;
                PHtoofast = true;
                PhTimer1  = currentTimeMS + 1000;                               // Gather 1s speed of GPS
                AvgSpeed  = (int32_t)GPS_speed;
                LastGPSupdateState = GPS_update;
                PHcascade++;
            case 2:
                if(LastGPSupdateState != GPS_update)
                {
                    LastGPSupdateState = GPS_update;
                    AvgSpeed += (((int32_t)GPS_speed - AvgSpeed) >> 1);
                    if (currentTimeMS >= PhTimer1)
                    {
                        PhTimer1 = 0;
                        PHcascade++;
                    }
                }
                break;
            case 3:
                if(LastGPSupdateState != GPS_update)
                {
                    LastGPSupdateState = GPS_update;
                    AvgSpeed += (((int32_t)GPS_speed - AvgSpeed) >> 1);        // Keep averaging speed
                    if (AvgSpeed < cfg.gps_ph_settlespeed) PHcascade++;
                    else
                    {
                        if(!PhTimer1) PhTimer1 = currentTimeMS + constrain(((AvgSpeed - cfg.gps_ph_settlespeed) / cfg.gps_ph_brkacc) * 1000, 1, 5000);  // t = v/a limit to 5 seconds
                        else if (currentTimeMS >= PhTimer1) PHcascade++;
                    }
                }
                break;
            case 4:                                                             // Set Waittimer for GPS Catch Up before setting new GPS coords
                PHtoofast = false;
                PhTimer1  = currentTimeMS + (uint32_t)cfg.gps_lag;
                PHcascade++;
            case 5:                                                             // Wait for gps to catch up and set new WP
                if (currentTimeMS >= PhTimer1)                                  // Wait till GPS Lag is done
                {
                    if(!PHuseGPSWP)                                             // We already know our Ph target
                    {
                        GPS_WP[LAT] = Real_GPS_coord[LAT];                      // No? So we define it here
                        GPS_WP[LON] = Real_GPS_coord[LON];
                    }
                    GPS_reset_nav();                                            // Reset nav Speedvector as well
                    PHcascade++;                    
                }
                break;
            case 6:                                                             // Do this forever ?
                PHtoofast = false;
                PHChange  = false;
                GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON]);
                ph_status = PH_STATUS_DONE;
                break;
            }
            if (!PHChange) GPS_calc_posholdCrashpilot(PHtoofast);               // PHtoofast limits the over all tiltangle per axis
            break;

        case NAV_MODE_CIRCLE:
            // *** DO SOME SERIOUS SHIT HERE LATER
            //		GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
            //    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
            break;

        case NAV_MODE_WP:
        case NAV_MODE_RTL:
            GPS_distance_cm_bearing(&GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
            GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON]);

            speed = GPS_calc_desired_speed();
            GPS_calc_nav_rate(speed);                                           // use error as the desired rate towards the target Desired output is in nav_lat and nav_lon where 1deg inclination is 100

            if (cfg.nav_controls_heading && wp_distance > 200)                  // Tail control only update beyond 2 m
            {
                magHold = (float)nav_bearing * 0.01f;                           // tmpflt is in degree now
                if (cfg.nav_tail_first) magHold = wrap_180(magHold - 180.0f);
            }

            if ((wp_distance <= cfg.gps_wp_radius) || check_missed_wp())        // if yes switch to poshold mode
            {
                if (cfg.nav_rtl_lastturn && nav_mode == NAV_MODE_RTL) magHold = nav_takeoff_heading;  // rotates it's head to takeoff direction if wanted
                nav_mode   = NAV_MODE_POSHOLD;
                wp_status  = WP_STATUS_DONE;
                PH1stRun   = true;
                PHuseGPSWP = true;                                              // We want current WP as PH Target
            }
            else wp_status = WP_STATUS_NAVIGATING;
            break;
        }                                                                       // END Switch nav_mode
    }                                                                           // END of gps calcs i.e navigating
}

////////////////////////////////////////////////////////////////////////////////////
// PID based GPS navigation functions
// Author : EOSBandi some functions pimped by Crashpilot
////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
void GPS_set_next_wp(int32_t *lat, int32_t *lon)
{
    float tmp0, tmp1;
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;

    GPS_calc_longitude_scaling(true);
    GPS_distance_cm_bearing(&GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON]);
    nav_bearing = target_bearing;
    original_target_bearing = target_bearing;
    waypoint_speed_gov = (float)cfg.nav_speed_min;
    WP_Target_Alt = 0;
    WP_Desired_Climbrate = 0;

    switch(nav_mode)
    {
    case NAV_MODE_POSHOLD:
        PH1stRun   = true;
        PHuseGPSWP = false;                                                     // We have no special PH WP in mind, let PH find its own
        break;
    case NAV_MODE_RTL:
        WP_Fastcorner = false;                                                  // This means: Slow down when approaching WP
        break;
    case NAV_MODE_WP:
        tmp0 = (float)(WP_Target_Alt - EstAlt);                                 // tmp0 = hightdifference in cm.  + is up
        tmp1 = ((float)wp_distance / (float)cfg.nav_speed_max) * 1.2f;          // tmp1 = Estimated Traveltime + 20% // Div Zero not possible
        if (!tmp1) WP_Desired_Climbrate = 0;                                    // Avoid Div Zero
        else WP_Desired_Climbrate = tmp0 / tmp1;                                // Climbrate in cm/s
        break;
    case NAV_MODE_CIRCLE:                                                       // Set some constants
//        Maybe some shit here later
//        Project a gps point x cm ahead the copter nose will look like this:
//        Project[LON] = Current[LON]+ (int32_t) ((Project_forward_cm * sin_yaw_y) * OneCmTo[LON]);
//        Project[LAT] = Current[LAT]+ (int32_t) ((Project_forward_cm * cos_yaw_x) * OneCmTo[LAT]);
        break;
    }
}
