#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"
#define  WPrxtxTO 500                                 // 500ms timeout for wp request/sending packet flow

static bool mavlink_send_paralist;
static uint16_t ExpectedTotalIncomingWP, ActualExpectedWpIDX, ActualWPFloppyWriteIDX;
static uint32_t ControlAndSensorsPresent = 35843;
static uint32_t TimeStampMSlastWPtxrxAction = 0;
static bool     AllowWPrx = false, AllowWPtx = false, ROIdefined = false;
static uint8_t  system_type;
static uint8_t  SysIDOfPartner;
static uint8_t  CompIDOfPartner;

void baseflight_mavlink_init(void)
{
    switch(cfg.mixerConfiguration)                    // Set system_type here
    {
    case MULTITYPE_TRI:
        system_type = MAV_TYPE_TRICOPTER;
        break;
    case MULTITYPE_QUADP:
    case MULTITYPE_QUADX:
    case MULTITYPE_Y4:
    case MULTITYPE_VTAIL4:
        system_type = MAV_TYPE_QUADROTOR;
        break;
    case MULTITYPE_Y6:
    case MULTITYPE_HEX6:
    case MULTITYPE_HEX6X:
        system_type = MAV_TYPE_HEXAROTOR;
        break;
    case MULTITYPE_OCTOX8:
    case MULTITYPE_OCTOFLATP:
    case MULTITYPE_OCTOFLATX:
        system_type = MAV_TYPE_OCTOROTOR;
        break;
    case MULTITYPE_FLYING_WING:
    case MULTITYPE_AIRPLANE:
        system_type = MAV_TYPE_FIXED_WING;
        break;
    case MULTITYPE_HELI_120_CCPM:
    case MULTITYPE_HELI_90_DEG:
        system_type = MAV_TYPE_HELICOPTER;
        break;
    default:
        system_type = MAV_TYPE_GENERIC;
        break;
    }
/*
    onboard_control_sensors_present Bitmask
    fedcba9876543210
    1000110000000011    For all   = 35843
    0001000000000100    With Mag  = 4100
    0010000000001000    With Baro = 8200
    0100000000100000    With GPS  = 16416
    0000001111111111
*/
    if (sensors(SENSOR_MAG))  ControlAndSensorsPresent |=  4100;
    if (sensors(SENSOR_BARO)) ControlAndSensorsPresent |=  8200;
    if (sensors(SENSOR_GPS))  ControlAndSensorsPresent |= 16416;
    Currentprotocol = PROTOCOL_AUTOSENSE;                     // Set primary Protocol to unknown/autosensing
    reset_mavlink();
}

void reset_mavlink(void)
{
    baseflight_mavlink_send_paramlist(true);                             // Reset parameterlist sending also enables AllowProtocolAutosense
    AllowWPrx                   = false;
    AllowWPtx                   = false;
    ROIdefined                  = false;
    mavlink_send_paralist       = false;
    TimeStampMSlastWPtxrxAction = 0;
    ActualExpectedWpIDX         = 0;
    ActualWPFloppyWriteIDX      = 0;
}

static void RxMissionACKandResetML(uint8_t acktype)
{
    mavlink_message_t m;
    if (ScheduleEEPROMwriteMS && acktype != MAV_MISSION_ACCEPTED)       // Error and a faulty writeaction is scheduled
    {
        ScheduleEEPROMwriteMS = 0;                                      // Abort planned saving
        readEEPROM();                                                   // Reload already stored stuff
    }
    mavlink_msg_mission_ack_pack(MLSystemID, MLComponentID, &m, SysIDOfPartner, CompIDOfPartner, acktype);
    baseflight_mavlink_send_message(&m);  
    reset_mavlink();
}
/*
  Overview over the ACK possibilities
	MAV_MISSION_ACCEPTED=0,           mission accepted OK |
	MAV_MISSION_ERROR=1,              generic error / not accepting mission commands at all right now | 
	MAV_MISSION_UNSUPPORTED_FRAME=2,  coordinate frame is not supported |
	MAV_MISSION_UNSUPPORTED=3,        command is not supported | *
	MAV_MISSION_NO_SPACE=4,           mission item exceeds storage space | *
	MAV_MISSION_INVALID=5,            one of the parameters has an invalid value | *
	MAV_MISSION_INVALID_PARAM1=6,     param1 has an invalid value | *
	MAV_MISSION_INVALID_PARAM2=7,     param2 has an invalid value | *
	MAV_MISSION_INVALID_PARAM3=8,     param3 has an invalid value | *
	MAV_MISSION_INVALID_PARAM4=9,     param4 has an invalid value | *
	MAV_MISSION_INVALID_PARAM5_X=10,  x/param5 has an invalid value | *
	MAV_MISSION_INVALID_PARAM6_Y=11,  y/param6 has an invalid value | *
	MAV_MISSION_INVALID_PARAM7=12,    param7 has an invalid value | *
	MAV_MISSION_INVALID_SEQUENCE=13,  received waypoint out of sequence | *
	MAV_MISSION_DENIED=14,            not accepting any mission commands from this communication partner | *
	MAV_MISSION_RESULT_ENUM_END=15,   *  | *
*/

static void CheckWPrxtxTimeouts(void)
{
    if (!AllowWPrx && !AllowWPtx) return;                                // Skip to keep exec. time low
    if (((millis() - TimeStampMSlastWPtxrxAction) > WPrxtxTO) || (AllowWPrx && AllowWPtx)) reset_mavlink();
}

static bool IniOfWPrxtxPossible(void)
{
    if (AllowWPrx || AllowWPtx || ScheduleEEPROMwriteMS || f.ARMED) return false;
    return true;
}

static void ClearDataset(wp_t *wipe)
{
    uint8_t i;
    for (i = 0; i < sizeof(wp_t); i++) (*wipe).bytes[i] = 0;
}

bool baseflight_mavlink_send_1Hzheartbeat(void)                          // That mother is running at 1Hz and schedules/collects eeprom writes
{
    mavlink_message_t msg2;
    static uint32_t   LastHeartbeat;
    uint8_t           autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;      // uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	  uint8_t           system_mode    = 0;                                // Is set below
	  uint32_t          custom_mode    = 0;
	  uint8_t           system_state   = MAV_STATE_STANDBY;

    CheckWPrxtxTimeouts();                                               // Check for disconnects during WP stuff
    if ((currentTimeMS - LastHeartbeat) < 1000) return false;
    LastHeartbeat = currentTimeMS;

//  Set this here if Automission: MAV_MODE_STABILIZE_DISARMED
    if (f.ANGLE_MODE || f.HORIZON_MODE) system_mode = MAV_MODE_STABILIZE_DISARMED;
     else system_mode = MAV_MODE_MANUAL_DISARMED;
    if(f.ARMED)
    {
        system_mode |= 128;                                              // Set the Armed bit here if necessary
        system_state = MAV_STATE_ACTIVE;
    }
    mavlink_msg_heartbeat_pack(MLSystemID, MLComponentID, &msg2, system_type, autopilot_type, system_mode, custom_mode, system_state);
    baseflight_mavlink_send_message(&msg2);
    return true;
}

void baseflight_mavlink_send_message(mavlink_message_t* msg)
{
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	  uint16_t i;
	  for (i = 0; i < len; i++) uartWrite(buf[i]);
}

bool baseflight_mavlink_receive(char new)
{
    static mavlink_message_t msg;
	  static mavlink_status_t  status;
	  if (mavlink_parse_char(0, new, &msg, &status))
    {
        baseflight_mavlink_handleMessage(&msg);
        return true;
    }
    else return false;
}

void baseflight_mavlink_handleMessage(mavlink_message_t *msg)
{
    mavlink_message_t msg2;
    SysIDOfPartner  = msg->sysid;
    CompIDOfPartner = msg->compid;

    switch (msg->msgid)
    {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        if (!mavlink_send_paralist)
        {
            baseflight_mavlink_send_paramlist(true);           // Just reset function to send from the beginning
            mavlink_send_paralist = true;                      // Only initiate paralist send here, and not already sending
        }
			  break;

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:                  // Prepare sending WP to GCS
        CheckWPrxtxTimeouts();
        if (!IniOfWPrxtxPossible() || !cfg.FDUsedDatasets) break;
        AllowWPtx = true;
        AllowProtocolAutosense = false;                        // Disable Protocolswitching
        mavlink_msg_mission_count_pack(MLSystemID, MLComponentID, &msg2, SysIDOfPartner, CompIDOfPartner, cfg.FDUsedDatasets);
        baseflight_mavlink_send_message(&msg2);
        TimeStampMSlastWPtxrxAction = currentTimeMS;
        break;
        
    case MAVLINK_MSG_ID_MISSION_REQUEST:                       // Sending Data to gui
    {
        mavlink_mission_request_t packet;
        wp_t                      wantedWP;
        uint16_t                  MLcmd = 0;
        float                     param1 = 0, param2 = 0, param3 = 0, param4 = 0, x, y, z;
        
        mavlink_msg_mission_request_decode(msg, &packet);
        
        if(!FloppyReadWP(&packet.seq, &wantedWP) || f.ARMED || !AllowWPtx) break;

        if (wantedWP.WPHead < 0) wantedWP.WPHead += 360;
        x      = (float)wantedWP.WPGPS[LAT] / 1.0e7f;
        y      = (float)wantedWP.WPGPS[LON] / 1.0e7f;
        z      = (float)wantedWP.WPHight;
        param1 = wantedWP.WPTime;                             // Always prefeed some paras here, they will be zero anyway if not needed, or corrected below
        param2 = wantedWP.WPPara1;
        param4 = wantedWP.WPHead;

        switch(wantedWP.WPCMD)                                // Note: Unknown command not possible sorted out on write
        {
        case ML_NAV_WAYPOINT:
            MLcmd  = MAV_CMD_NAV_WAYPOINT;
            break;
        case ML_NAV_SETROI:
            MLcmd  = MAV_CMD_NAV_ROI;
            break;
        case ML_NAV_LOITER_UNLIM:
            MLcmd  = MAV_CMD_NAV_LOITER_UNLIM;
            break;
        case ML_NAV_LOITER_TURNS:
            param1 = wantedWP.WPPara2;
            MLcmd  = MAV_CMD_NAV_LOITER_TURNS;
            break;
        case ML_NAV_LOITER_TIME:
            MLcmd  = MAV_CMD_NAV_LOITER_TIME;
            break;
        case ML_NAV_RETURN_TO_LAUNCH:
            MLcmd = MAV_CMD_NAV_RETURN_TO_LAUNCH;
            break;
        case ML_NAV_LAND:
            MLcmd = MAV_CMD_NAV_LAND;
            break;
        case ML_NAV_TAKEOFF:
            MLcmd = MAV_CMD_NAV_TAKEOFF;
            break;
        }

        mavlink_msg_mission_item_pack(MLSystemID, MLComponentID, &msg2,
            SysIDOfPartner, CompIDOfPartner, packet.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MLcmd,
            0, 1, param1, param2, param3, param4, x, y, z);
        baseflight_mavlink_send_message(&msg2);
        TimeStampMSlastWPtxrxAction = currentTimeMS;
        if (packet.seq == (cfg.FDUsedDatasets - 1)) reset_mavlink();
        break;
    }

// Some thoughts on Transmitted WP/lists that are useless for us and therfor filtered out.
// Missions should consist at least of one WP besides the Homeposition so "Missions" with only one element ("Homepos") are rejected
// WP with unknown instructions("faulty WP") are skipped on save.
// A Mission with 2 Elements Homepos + faulty WP is not accepted and will flush/clear the stored WP list.
// Problem with knife edge waypoints, solution:
// A waypoint with a timevalue/delay (non zero) is pointless because than it's a loiterpoint. But we can use that to define a fast or slow waypoint.
// That means: If timevalue for waypoint is zero, the WP can be approached with full speed without braking (because the next wp is probably in line).
// If the timevalue is non zero (like "1") the wp will be approached with braking in, like needed when some sharp turn follows.
// MAV_CMD_NAV_LOITER_UNLIM can not be followed by another WP a WP list with that MAV_CMD_NAV_LOITER_UNLIM in the middle is considered faulty and is not accepted.
// The same goes for MAV_CMD_NAV_LAND.

		case MAVLINK_MSG_ID_MISSION_COUNT:                           // Always done to initiate wp list reception from GCS
    {
				mavlink_mission_count_t packet;
        mavlink_msg_mission_count_decode(msg, &packet);
        ExpectedTotalIncomingWP = packet.count;
        CheckWPrxtxTimeouts();
        if (!IniOfWPrxtxPossible() || ExpectedTotalIncomingWP < 2) // Error when armed or WP stuff already going on or a write was scheduled and not done or no real wps in pipeline (like a "homepos ony" wplist)
        {
            RxMissionACKandResetML(MAV_MISSION_ERROR);
            break;
        }

        if (ExpectedTotalIncomingWP > (FDByteSize / sizeof(wp_t))) // Too many wp's for floppy
        {
            RxMissionACKandResetML(MAV_MISSION_NO_SPACE);
            break;
        }
        reset_mavlink();
        FloppyClear();                                           // Clear floppy
        AllowWPrx = true;
        AllowProtocolAutosense = false;                          // Disable Protocolswitching
        mavlink_msg_mission_request_pack(MLSystemID, MLComponentID, &msg2, SysIDOfPartner, CompIDOfPartner, ActualExpectedWpIDX);
        baseflight_mavlink_send_message(&msg2);
        TimeStampMSlastWPtxrxAction = currentTimeMS;
        break;
		}
    
    case MAVLINK_MSG_ID_MISSION_ITEM:                            // Getting Data & reasonable plausibility check
    {
        mavlink_mission_item_t packet;
        wp_t                   actualWP;
        uint8_t                MYcmd = 0;
        uint16_t               Limit = ExpectedTotalIncomingWP - 1;
        bool                   Skipdataset = false;
        bool                   CheckIfLastCommand = false;
        bool                   TerminateWithFaultyMission = false;
        bool                   TerminateWhenStandAloneCommandInList = false;
        bool                   CheckHeadingROI = false;
        bool                   CheckHitRadius  = false;
        bool                   CheckIfGPSCoordsAreValid = false;

	  		mavlink_msg_mission_item_decode(msg, &packet);
        if (f.ARMED || !AllowWPrx)                               // Don't do this when armed or wp reception not allowed or Wp sending is in progress
        {
            RxMissionACKandResetML(MAV_MISSION_ERROR);
            break;
        }
        
  			if ((packet.seq > Limit)) TerminateWithFaultyMission = true; // Error, out of bounds

        if (packet.seq != ActualExpectedWpIDX)                    // Do we get what we want?
        {
            RxMissionACKandResetML(MAV_MISSION_INVALID_SEQUENCE);
            break;
        }

        ClearDataset(&actualWP);
        actualWP.WPGPS[LAT] = (int32_t)(packet.x * 1.0e7f);
        actualWP.WPGPS[LON] = (int32_t)(packet.y * 1.0e7f);
        actualWP.WPHight    = (int16_t)constrain(packet.z, -32767, 32767); // In meter
        if(!actualWP.WPHight) actualWP.WPHight = 2;              // Ensure a minimal missionhight of 2m AGL

        switch (packet.command)                                  // Get command here translated to our command set (currently somehow the same... BUT only a byte not a word..)
        {
        case MAV_CMD_NAV_WAYPOINT:                               // Unused: param3
            actualWP.WPTime  = (uint8_t)constrain(packet.param1, 0, 1); // 1 signalize slow turn wp, 0 signalize fast corner
            CheckHeadingROI  = true;
            CheckHitRadius   = true;
            CheckIfGPSCoordsAreValid = true;
            MYcmd = ML_NAV_WAYPOINT;
            break;

        case MAV_CMD_NAV_ROI:                                    // Unused: param1,param2,param3,param4,altitude
            TerminateWhenStandAloneCommandInList = true;         // Command can not exist alone
            if (packet.seq == Limit) TerminateWithFaultyMission = true; // ROI can not be the last command in the list
            if(!actualWP.WPGPS[LAT] || !actualWP.WPGPS[LON]) Skipdataset = true; // If lat or lon skip this and don't set roidefined
            else ROIdefined = true;
            actualWP.WPHight = 0;                                // Note: Altitude is not used because we just point copternose, no gimbal stuff
            MYcmd = ML_NAV_SETROI;
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:                           // Unused: param1,param3
            CheckIfLastCommand = true;                           // Must be last command in a list!
            CheckHeadingROI    = true;
            CheckHitRadius     = true;
            CheckIfGPSCoordsAreValid = true;
            MYcmd = ML_NAV_LOITER_UNLIM;            
            break;

        case MAV_CMD_NAV_LOITER_TURNS:                           // Unused: param3,param4
            actualWP.WPPara2 = (int8_t)constrain(packet.param1, -127, 127); // Number of Turns at least one Turn, otherwise it's pointless
            if(!actualWP.WPPara2) actualWP.WPPara2 = 1;          // Ensure at least one cw turn.
            CheckHitRadius   = true;
            CheckIfGPSCoordsAreValid = true;
            MYcmd = ML_NAV_LOITER_TURNS;
            break;

        case MAV_CMD_NAV_LOITER_TIME:                            // Unused: param3,param4
            actualWP.WPTime  = (uint8_t)constrain(packet.param1, 1, 255); // At least one second
            CheckHeadingROI  = true;
            CheckHitRadius   = true;
            CheckIfGPSCoordsAreValid = true;
            MYcmd = ML_NAV_LOITER_TIME;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:                       // Unused: param1,param2,param3,param4
            TerminateWhenStandAloneCommandInList = true;         // Command can not exist alone
            actualWP.WPGPS[LAT] = actualWP.WPGPS[LON] = 0;       // Use current postition
            MYcmd = ML_NAV_RETURN_TO_LAUNCH;
            break;

        case MAV_CMD_NAV_LAND:
            TerminateWhenStandAloneCommandInList = true;         // Command can not exist alone
            ClearDataset(&actualWP);                             // Use current postition
            CheckIfLastCommand = true;                           // Must be last command in a list!
            MYcmd = ML_NAV_LAND;
            break;

        case MAV_CMD_NAV_TAKEOFF:
            if (packet.seq != 1) TerminateWithFaultyMission = true;// Must be first item (besides "homepoint") and can not be the last command
            TerminateWhenStandAloneCommandInList = true;         // Command can not exist alone
            actualWP.WPGPS[LAT] = actualWP.WPGPS[LON] = 0;       // Use current postition but use defined hight
            actualWP.WPHight = constrain(actualWP.WPHight, 2, 255);// Constrain to reasonable values and rule out negative values
            MYcmd = ML_NAV_TAKEOFF;
            break;

        default:                                                 // Unknown Command skip it, maybe delete list if it was the only command
            Skipdataset = true;
            break;
        }
        
        actualWP.WPCMD = MYcmd;

        if (TerminateWithFaultyMission ||
           (CheckIfLastCommand && packet.seq != Limit) ||
           (TerminateWhenStandAloneCommandInList && ExpectedTotalIncomingWP == 2))
        {
            RxMissionACKandResetML(MAV_MISSION_INVALID);         // That list is not accepted.
            break;                                               // Bail out here completely
        }

        if (CheckIfGPSCoordsAreValid && (!actualWP.WPGPS[LAT] || !actualWP.WPGPS[LON])) Skipdataset = true; // GPS coords are needed but fucked up, skip dataset

        if (CheckHeadingROI)
        {
            actualWP.WPHead = (int16_t)constrain(packet.param4, 0, 1); // Read out param4 into "Heading"
            if(actualWP.WPHead && !ROIdefined) actualWP.WPHead = 0;// Don't use ROI if no ROI was defined
        }

        if (CheckHitRadius)
        {
            actualWP.WPPara1 = (uint8_t)constrain(packet.param2, cfg.gps_wp_radius / 100, 255); // Hitradius in m can not be smaller than preset
        }

        if(!FloppyWriteWP(&ActualWPFloppyWriteIDX, &actualWP))   // Bail Out! This error should never happen here ..
        {
            RxMissionACKandResetML(MAV_MISSION_NO_SPACE);          
            break;
        }

        ActualExpectedWpIDX++;                                   // Increase WP Requestindex
        ActualWPFloppyWriteIDX++;                                // Increase Floppywriteindex

        if (Skipdataset)                                         // Was the (ram)stored WP not suitable for our needs?
        {
            if (ExpectedTotalIncomingWP == 2)                    // If the mission just consits of Homepos and one invalid WP, abort this here
            {
                RxMissionACKandResetML(MAV_MISSION_ACCEPTED);    // Pseudo "ACK" that mission but flush wp list
                FloppyClear();
                ScheduleEEPROMwriteMS = 1;
                break;
            }
            else
            {
                cfg.FDUsedDatasets--;                             // Take back totalcount(was increased by floppywrite)
                ActualWPFloppyWriteIDX--;                         // Take back writeindex
            }
        }

        if (packet.seq == Limit)                                  // End reached. All OK exit.
        {
            RxMissionACKandResetML(MAV_MISSION_ACCEPTED);
            break;
        }

        mavlink_msg_mission_request_pack(MLSystemID, MLComponentID, &msg2, SysIDOfPartner, CompIDOfPartner, ActualExpectedWpIDX);
        baseflight_mavlink_send_message(&msg2);
        TimeStampMSlastWPtxrxAction = currentTimeMS;
        break;
    }
    
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            mavlink_param_request_read_t packet;
            mavlink_msg_param_request_read_decode(msg, &packet);
            baseflight_mavlink_send_singleparam(packet.param_index);
            break;
        }
		case MAVLINK_MSG_ID_PARAM_SET:
        {
            mavlink_param_set_t packet;
			      mavlink_msg_param_set_decode(msg, &packet);
            if(baseflight_mavlink_set_param(&packet)) ScheduleEEPROMwriteMS = currentTimeMS + 500; // Collect some EEPROMWRITES BEFORE ACTUALLY DOING IT
      	    break;
        }
		default:
			  break;
    }
}

void baseflight_mavlink_send_updates(void)                                      // That's a bad mother here :)
{
    static uint32_t   Timer100Hz;
    static uint8_t    HudPackCnt, AttiPackCnt, GPSPackCnt, RCPackCnt, ParaLstCnt, StatuspackCnt, PressPackCnt;
    uint16_t          voltage = 0;
    mavlink_message_t msg2;
    bool              PacketSent;                                               // Avoid too much stuff in one Action
    int16_t           tmp1;

//  NOTE: THE HZ NUMBERS ARE WISHFUL THINKING, BECAUSE IT IS ENSURED THAT ONLY ONE PACKET IS SENT PER RUN
//  SO THE ACTUAL HZ WILL DEGRADE, CHECK WITH GCS FOR REAL DATARATES
    PacketSent = baseflight_mavlink_send_1Hzheartbeat();                        // Does internal 1Hz Timer returns true when done
    if ((currentTime - Timer100Hz) >= 10000)                                    // 100Hz Timebase for mavlink because it's slow anyways
	  {
        Timer100Hz = currentTime;

        if (++PressPackCnt >= 200 && !PacketSent && sensors(SENSOR_BARO))       // 0.5Hz for Pressure Pack
        {
            PressPackCnt = 0;
            PacketSent   = true;
            mavlink_msg_scaled_pressure_pack(
                MLSystemID, MLComponentID, &msg2, currentTimeMS, ActualPressure * 0.01f, 0, telemTemperature1 * 100);
            baseflight_mavlink_send_message(&msg2);
        }

        if (++RCPackCnt >= 47 && !PacketSent)                                   // 2Hz for RC
        {
            RCPackCnt  = 0;
            PacketSent = true;
            mavlink_msg_rc_channels_raw_pack(
                MLSystemID , MLComponentID, &msg2, currentTimeMS, 0, rcData[0], rcData[1], rcData[3],
                rcData[2], rcData[4], rcData[5], rcData[6], rcData[7], rssi);
            baseflight_mavlink_send_message(&msg2);
        }

        if (++StatuspackCnt >= 48 && !PacketSent)                               // 2Hz for Status
        {
            StatuspackCnt = 0;
            PacketSent    = true;
            if (FEATURE_VBAT) voltage = (uint16_t)vbat * 100;                   // in mV
            mavlink_msg_sys_status_pack(
                MLSystemID, MLComponentID, &msg2, ControlAndSensorsPresent, ControlAndSensorsPresent,
                ControlAndSensorsPresent & 1023, 0, voltage, -1, -1, 0, 0, 0, 0, 0, 0);
            baseflight_mavlink_send_message(&msg2);
        }

        if (++GPSPackCnt >= 49 && !PacketSent && sensors(SENSOR_GPS))           // 2Hz for GPS
        {
            GPSPackCnt = 0;
            PacketSent = true;
            if (GPS_FIX) tmp1 = 3;                                              // Report 3Dfix if any fix
             else tmp1 = 0;
	          mavlink_msg_gps_raw_int_pack(
                MLSystemID , MLComponentID, &msg2, currentTime, (uint8_t)tmp1, Real_GPS_coord[LAT], Real_GPS_coord[LON], GPS_altitude * 1000,
                65535, 65535, GPS_speed, constrain(GPS_ground_course * 10, 0, 35999), GPS_numSat);
            baseflight_mavlink_send_message(&msg2);
        }
        
        if (++HudPackCnt >= 10 && !PacketSent)                                  // 10Hz for HUD
        {
            HudPackCnt = 0;
            PacketSent = true;
            if (sensors(SENSOR_MAG))
            {
                tmp1 = heading;
                if (tmp1 < 0) tmp1 += 360;                                      // heading in degrees, in compass units (0..360, 0=north)
            }else tmp1 = 0;
            mavlink_msg_vfr_hud_pack(
                MLSystemID, MLComponentID, &msg2, 0, (float)GPS_speed * 0.01f, tmp1,
                ((int32_t)(rcCommand[THROTTLE] - cfg.esc_min) * 100)/(cfg.esc_max - cfg.esc_min),
                EstAlt * 0.01f, vario * 0.01f);
            baseflight_mavlink_send_message(&msg2);
        }
        
        if (++ParaLstCnt >= 10 && !PacketSent)                                  // 10Hz for Parameterlist transmission
        {
            ParaLstCnt = 0;
            if (mavlink_send_paralist)
            {
                PacketSent = true;
                mavlink_send_paralist = !baseflight_mavlink_send_paramlist(false);// baseflight_mavlink_send_param_lst is true when done
            }
        }

        if (++AttiPackCnt >= 3 && !PacketSent)                                  // 30Hz for Attitude
        {
            AttiPackCnt = 0;
            mavlink_msg_attitude_pack(
                MLSystemID, MLComponentID, &msg2, currentTimeMS, angle[0] * RADX10, -angle[1] * RADX10,
                heading * RADX, 0.0f, 0.0f, 0.0f);
	          baseflight_mavlink_send_message(&msg2);
        }
	  }
}
