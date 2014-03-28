Naze32 Harakiri10 Summer Games pre2.6
=====================================
- Note: Used Hardware: MPU6050 / MS5611 / BMP085 / HMC5883 / GPS-NL-652ETTL / HC-SR04 / MB1200 XL (NAZE v4)
- Probably now compatible with v5. Probably Buzzer working now.
- LONG STANDING MONSTERBUG IN MAINLOOP DELETED.
- Increased Coreclock on Naze4/5 to 80/84Mhz to evade problems with 433Mhz equipement (6th harmonic).
- Added rc_flpsp = x (Range 0 - 3) When enabled and upside down in acro or horizon mode only reduced throttle is applied.
  Also works in combination with althold.
-- rc_flpsp = 0 |Disabled. Default value.
-- rc_flpsp = 1 |1/2 throttle - input is applied when upside down.
-- rc_flpsp = 2 |1/3 throttle - input is applied when upside down.
-- rc_flpsp = 3 |1/4 throttle - input is applied when upside down.
- Probably some MULTITYPE_FLYING_WING stuff fixed in mixer.c see: http://code.google.com/p/afrodevices/source/detail?r=450
- Since we calculate mainpids in float and the mixer is preset in float, I put the mixer to float for the most copter part as well.
- Fixed: http://www.multiwii.com/forum/viewtopic.php?f=8&t=4150#p42211 (Throttle expo array could be out of bounds)
- Removed moving average filter from D calculation. Therefor maincuthz (default 12Hz, Range 1-50Hz) becomes critical for D calculation. Higher frequency lets more ripple pass in D part.
- Put Gyrosmoothing to floats
- Deleted FEATURE_GYRO_SMOOTHING added gy_smrll, gy_smptc, gy_smyw instead. Now you can adjust gy_smyw for tricopter (see below).
- Added MWII 2.3 Yaw algo, for mainpidctrl = 0
- Imu rearrangements:
--Reworked the mwii acc lowpassfilter stuff to proper time dependency i.e frequency filter. Therefor:
-- acc_lpf  = 100 deleted and repaced by acc_lpfhz  = 10.0 (0.536 is equivalent to the former "100" based on 3ms cycletime)
-- acc_ilpf =  10 deleted and repaced by acc_altlpfhz = 15 (6 is equivalent to the former " 10" based on 3ms cycletime)
-- acc_gpslpfhz is separate lpf for GPS ins. Default is 30 Hz for now.
-- Recalibrate acc upon change of gy_lpf or acc type of course.
-- L3G4200D driver fix like here http://code.google.com/p/afrodevices/source/detail?r=403
-- The gyrocalibration data is saved upon acc calibration because if you desperately try to arm your copter on shaky conditions(boats, earthquakes..),
   after ca 20 seconds of frustrane gyro calibration those already saved data will be taken just to be flyable (LED warning flashes).
   Note: Please let the copter warm up a few minutes before triggering ACC Calibration for best results. Cli "status" will give you all details of calib data.
- HMC5883 Driverchanges. With mag_gain(0 default, use 1 on problematic copters) you can adjust the GAUS sensitivity for bootup gaincalculation. Recalibrate compass when changing that.
-- Note: Both tested: mag_gain = 1 gives *slightly* worse PH results, but may be a toilet bowl lifesaver on copters that are on the edge of influencing compass interference.
-- Default Mag orientation is set to 0 in config, and proper aligned for naze 3/4 internally.
- Rework Magcalibration. Removed old calibration and variables: mag_oldctime and mag_oldcalib.
  mag_time (1-6) will set the mag calibration time in minutes now. Default is 1.
  Tested "intelligent mag calibrating" idea but results vary too much between setups/copters that it didn't work out. Idea still in mind, maybe later..
- Reworked Failsafe / cleanup / improved hoverthrottle for fs with barofunction. Since the throttlechannel is not valid in FS situations, a statistic average is taken gathered during the flight.
  If that average is not bigger than rc_minchk + 5% an error is assumed and the predefined fs_rcthr is taken as althold baselinethrottle.
  Thx to Hinkel for pointing out that subject here: http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=280#p41661
- Farted around with ACC & GYRO Calibration to Sphere Algo but but at the end of the day those results were equal, so back to averaging. "gy_stdev" is set to more sensitive. Acc Calibration takes longer now (more values).
- Reworked Temperature readout for MPU6050 & MPU3050 (preparation for temperature compensation). Added some code for correct temperature / alternative MS5611 calculation (both unused and optional - needs recompile).
- Reorganized mainprogram, put throttlechannel/pid attenuation calculation into the rc loop
- Reworked the Copter Timming option. The problem with the original was that on every stickinput an eeprom write was done, that is unnecessary limiting the lifetime. Now you can trim around and only 5 seconds after last
  Triminput the data are saved to eeprom. Note: While an eeprom write is scheduled and not yet done, Arming the copter is not possible.

MAVLINK:
- Possible stack hog deleted.

RC & LED:
- Reworked RC stuff, rc_auxch is limited to 6 so total channels 10 maximal.
- The Gui will show the raw rc values BUT internally they are all limited to 1000 - 2000us, that is also true for feature pass (like esc calibration).
- rc_lowlat = X [X = 0 or 1] Default 1. 1 = lower latency, 0 = normal latency/more filtering (maybe for old rc equip.).
- FEATURE_INFLIGHT_ACC_CAL is gone now and replaced by Air-Trim:
-- Why?
   1. Because the Accelerometer calibration is much more advanced now, and an "in air" acc calibration is not possible without harming all INS functions.
   2. Airtrim is easier to use and of every day usability without enabling a special feature and without harming INS.
-- What does it do?
   It affects the trims in Angle/Horizon mode in the same way you would change them on a disarmed copter (throttle max + nick/roll trim input).
   But this time the offsets/trims are generated in the air.
-- What do I need?
   Your copter needs a working angle/horizon mode - so a calibrated acc.
   You need to have an ARMSWITCH defined in the GUI.
-- What is the exact procedure?
   1. Powerup Copter and let it initialize
   2. Do the Stickcommand (Thr MIN + Yaw MIN("left") + Nick MAX("up") + Roll MAX("right")) and watch the LEDs flicker (confirmation Air-Trim active, current trims cleared).
   3. Now ARM (switch) and lift off using angle or horizon mode.
   4. Once Airborne you can put the Armswitch into disarm position, your copter keeps running (see Killswitch note below).
   The trims are cleared and new trim offsets are generated according to your copter - leveling - stickinput.
   5. Turn on the ARM SWITCH again and the trims are applied (note: Trim offset generation needs just a few seconds).
-- If you are unhappy with the trims you can put the ARMSWITCH to DISARM again to clear trims and trim again.
   6. When the copter is disarmed the trims are saved and Air-Trim function is turned off (LED flicker for confirmation).
-- Notes:
   When you just want to clear your trims just do the stickcommand and then arm/disarm the copter - trims are zeroed and saved.
   When you have the Killswitch active (like rc_killt = 200ms) it is disabled during that Air-Trim flight - BUT MAKE SURE YOU ARE IN Air-Trim MODE SO WATCH THE CONFIRMATION LEDS FLICKER! 
-  FEATURE_MOTOR_STOP is deleted and replaced by rc_motor. It will decide upon the behaviour when throttle is below rc_minchk (throttlestick at low position).
   rc_motor = x [0-2] 0 is Default.
   rc_motor = 0 esc_min will be applied, the pid regulation is disabled to prevent flip on ground.
   rc_motor = 1 esc_min will be applied, the pid regulation will be ACTIVE. That prevents tipovers in the air when thrstick down, but may cause tipovers on the ground.
   rc_motor = 2 esc_moff will be applied, that means motors are off then "motor stop".
   NOTE: Baromode and Autoland. Once copter is flying (decision upon that is based on surpassing esc_nfly) in Baromode, a thrstick down will trigger Autolanding, not motor stop.


- RED LED:
-- In GPS mode the red LED will ALWAYS count up the satcount starting by 5 followed by a 2 sec break. 2 blinks followed by 2sec break = 6 Sats.
-- When not in GPS mode or not enough sats and everything is initialized and the copter is "sharp" and launchable the red led will do a warning strobolight.
-- Red strobolight is off in flight (but green LED on). In flight the red LED will show if in Angle or Horizon mode. Note: GPS satcount will override all other red LED blinkings!!


Tricopter:
- Added: "tri_ydel" Tricopter Yaw Delay in ms [0(disables)-1000]: Purpose: Avoids Yawservomovement when disarmed (sends "tri_yaw_middle" then), when doing yawarm there is a delay in initial yaw servo movement.
  Middlesignal will be send("tri_ymid") when not armed, so it can be used to setup your tri as well. If "tri_ydel" is zero, the yawsignal will be send (like always) even when disarmed.
- gy_smyw has special meaning in tricopter mode, because the servo may need special attention since it is not "so fast" and has an internal control loop to keep its setpoint - so it can lead to resonance
  (in the servo itself, if it is too weak, or in combination with the FC control algo)
  gy_smyw = 0 (default) No filtering for yaw servo
  gy_smyw = 1 JUST moving average filter for yaw servo
  gy_smyw = X and x is greater than 1: moving average filter for yaw servo + lowpassfilter (strength is set with X)

Althold:
- Sonar/Baro code rework/cleanup/Sonardriver. Sonaroffset generation improved with averaging.
- Rework of Barofiltering
- SONAR hardwareproblem info/found:
* Sudden errors with PWM Sonar readout. Keep Sonar pwm signal clear, use a ferrite and/or shielded dataline.
  Check your proper sonar function before first flight if you altered configuration with snr_debg = 1.
* Worst case: A freezing sonar with a valid value in flight will result in an althold doing a rapid climb.
  Looking into a softwareway to check for "in range" but actual invalid data. SOLVED: Disconnect for PWM Sonar is detected in flight!
  So if the sonar readout is dead for 300ms the Sonar sensor is disabled for that session, no matter if a random reconnect happens. You might see a rapid climb during
  that detection period, then everything is back to "normal" (without sonar). I can not try that out because I don't know how to produce a faulty
  on/off connection in flight on purpose (works on the bench). But if you see that behaviour in flight, you can be shure that you have managed to produce a slack joint in your sonar wiring.
  Note: With snr_debg = 1 you will still see the last sonar values before the sensor is shut down, these values will *not* be taken into account for the rest of the flight.
- Added I2C sonars based on mj666 work here http://fpv-treff.de/viewtopic.php?f=18&t=1368&start=3320#p58903 Devantech Ltd. (SRF02, SRF235, SRF08, SRF10), Maxboticx I2CXL (MB1202, MB1212, MB1222, MB1232, MB1242)
-- look in config.c for the details. You may want to build a logic level converter for the 3.3V STM bus. I included a simple (and working) layout I once used with my rusty multiwii and frsky bmp085 BOB. The parts are 1$ or so.
-- mj666 has implemented a switch for I2C speed to reduce it during I2C mxbtx sonar readout. It is reported to work flawless (can not test myself).
- Added, tested, deleted the arducopter groundpressure/groundtemperature stuff for baro like suggested by pm1. Could not compensate for misreadings due to baroheatup, worsens althold from what I've seen in my tests.
  Maybe further investigation with different code (Ms5611 "T2" temp correction) but currently no priority.
- Baro/Sonar Landingrate is now in cm/s (see below "Changed Var Values")
- Engaging Autoland is smoother due to suppression of clearing I.
- Baro Automodes: Autloand and Autostart turn off Horizon and turn on Anglemode automatically, if not already selected.
- Autostart. It is implemented here and turned off by default for future auto missions.
  Usage: When armed and copter not flying and Baromode is on and the throttle is put to the middle (incl. "rc_dbah") the autostart sequence is launched. It can be aborted by moving the throttle out of the middle zone.
  On abortion it will do an althold and wait for throttle recenter for further altholdchanges. Autostart can not be triggered in flight.
  Variables:
  "set as_trgt  = X"  X =  0 - 255 (m) DEFAULT:0. 0 Disables Autostart. Targethight is in m. Note: use 2m or more for reliable (hight) operation. 1m will work but overshoot.
  "set as_lnchr = X   X = 50 - 250 no dimension DEFAULT:200  Autostart launchrate to get off the ground. When as_stdev is exceeded, as_clmbr takes over
  "set as_clmbr = X"  X = 50 - 250cm/s DEFAULT:100. Autostart climbrate in cm/s after liftoff! Autostart Rate in cm/s will be lowered when approaching targethight.
  "set as_stdev = X"  X =  5 - 20 no dimension DEFAULT:10. This is the std. deviation of the variometer when a liftoff is assumed. The higher the more unsensitive.
  It is highly recommended to "set esc_nfly = x" (X = 0 as default) to an appropiate PWM value. It's the PWM value where the copter barely does NOT fly. In my case it's 1300.
  Purpose: The Copter will decide upon it's airborne status, if that PWM value is exceeded once. If it is left at 0 a value of 5% above esc_min is assumed.
  It will affect the initial throttle for Autostart. It will affect the disarm timeout when autolanding. It is used for initial hover throttle plausibility check in failsafe.
  Just for info: Internal function of Autostart: It is done in 3 stages.
  1. Initialize real Targethight, apply esc_nfly as basic throttle.
  2. Increase a virtual Targethight (beyond the real targethight) at desired rate until 50cm/s is achieved or the vario std. deviation is exceeded. So a liftoff is assumed and a short althold is engaged to reset virtual targethight.
     (The virtual targethight works like a rubberband to pull the copter off the ground)
  3. Now the real thing: Proceed with desired rate to actual Targethight.
  WARNING: Autostart is highly *NOT* recommended for indoor use! It will break the ceiling! Stand back on Autostart it *WILL* fly into your face.

MAG:
- Green and Red LED will flash in an alternating manner during calibration because green LED is hard to see on a sunny day.

GPS:
- GPS influence is zeroed on the ground to prevent/reduce tip overs. Autostart: No GPS influence until liftoff is detected. Autoland: If sonar landsupport is enabled, GPS is zeroed on proximity alert.
- In flight GPS disconnect/dead GPS is detected and all GPS functions are disabled then.
- Added Timejitter Filter for 20Hz and 50Hz GPS Datarates (preparation for new skytraq GPS)
- Added rtl_cr [10 - 200cm/s DEFAULT:80] When rtl_mnh (min hight for RTL) is defined this is the climbrate in cm/s that is used.
- Removed "gps_ins_mdl" from cli.
- Reworked GPS "D" calculation (navigation.c "get_D")
- Put PosR I out of order, will be always 0 in GUI. Pos I will work but differently.
- Increased PosR P and PosR D due to better working D part
- gps_ph_abstub & nav_slew_rate deleted and replaced by gps_expo. It's what you think an expocurve [0-99%, 0 disables] for the gps auto steering.
  It's exactly what you would do if you want to fine steer but done here automatically in PH - this eliminates several problems in PH at once:
  Reduces influence of GPS walk.
  Reduces jitter so nav_slew_rate is not needed anymore.
  No threshold needed so gps_ph_abstub deleted
  Note: gps_expo of 20 is very nice on my setup. 10% is too shaky and 30% leads to circeling.
- Reworked GPS logging / WP stuff
-- GPS Logging function is explanted in favour of waypoints for now.
-- With missionplaner/mavlink you can now edit and store currently 146WP without external eeprom on naze. May size it down later if space is an issue.
-- WP Saving/Loading via mavlink added. It is ONLY possible with disarmed copter, so playing around with the WP list when airborne is not possible - for safety.
-- There are timeouts(ca. 1sec) between reading and writing to naze with Missionplaner/GCS.
   So it's useless to go berserk on the read/write buttons. Naze will send an error-ack back to the gui or do no ack (MP does a timeout error then).
   The WP inputstream is watched by naze (like the expected WP number so no wp is lost, skipped or double saved. Out of memory etc), because otherwise errors could occure with big wp lists.
   If an error is detected (list too big, wp sequence wrong, copter armed etc), an error acknowledgement(ack) is send back to GUI.
   A WP list just consisting of an "homepoint" is considered as useless and is not accepted by naze ("error") - it has to have at least one valid WP (a ROI alone will also not do it, since it's no WP).
   A faulty WP has an unknown command (see supported commands below) or has missing LAT/LON coordinates (if required) and is skipped during save.
   A WP "list" with only one invalid WP is pseudo - accepted but will actually delete your current wp list on naze (alternative cli: "wpflush").
   Every entered hight in a mission is relative to the ground level. If you set absolute hights (above sea level) with MP/GCS it will be ignored and treated as relative values.
-- Don't reference to http://copter.ardupilot.com/wiki/planning-an-apmcopter-mission-with-waypoints-and-events/, since this is not Arducopter - some stuff handled differently or is not implemented etc.
-- Currently it is just the implementation of mavlink with no GPS code behind it!
-- These are the commands that are currently under developement and hopefully get working one by one:

   Nomenclature:
   The parameters will be called param1..param4 from left to right, so param4 will be yaw or some heading or something else.
   Latitude will be called LAT and Longitude will be called LON. LAT/LON are only mentioned when they are ignored by the command or something special going on with them or I just want to mention them.
   Altitude is measured in meters and will be called ALT. ALT is only mentioned if there is something special about it.
   Clockwise and counter clockwise will be called CW and CCW. Region of interest is called ROI.
   Missionplanner will be called MP. "MP Requester with some errormessage" will be called ERR.



   MAV_CMD_NAV_WAYPOINT:
   =====================
   Name in MP drop down menue: "WAYPOINT"
   What does it do:
   Trivial, it sets a waypoint, what did you think? BUT there is some non trivial stuff going on here:
   param1 (aka "Delay") will decide upon the speed the copter does that wp.
   param1 = 0 The copter will NOT brake before that wp and look for the next command (that may or may not cause him to brake)
   param1 = 1 The copter will slow down when approaching the WP (that is defined in cli by "nav_approachdiv" and nav_speed_min/max of course)
   Make sure that the corners are not too sharp, in doubt use param1 = 1. Higher values than 1 are taken as non zero and saved as "1".

   param2 (aka "Hit Rad") Is the hitradius in meter. You can not define a smaller hitradius than preset in cli by "gps_wp_radius".
   Note: gps_wp_radius is in CM and param2 is in meter, don't confuse that. Maybe I should redo gps_wp_radius in meter as well maybe later.
   If you don't enter a value, naze will fill in gps_wp_radius.

   param3: Not used, and entered value will not be saved.

   param4 (aka "Yaw Ang") will not be accepted as a real yaw angle. Normally the copter will look directly (nose in) to the waypoint BUT:
   param4 = 1 If a ROI is known it will nose in on the ROI point. To use that define a ROI BEFORE defining a WP. (see ROI)
   param4 = 0 ROI is ignored, copter nose points to the next WP.
   If param4 = 1 is used and no ROI defined, it will be saved as "0".
   
   Valid LAT/LON needed, otherwise the command is skipped upon save.

   ALT in meter will set a new targethight. That hight can be negative, if you want to fly down a canyon, loose GPS signal and crash.
   Note: An ALT of 0 m is not tolerated and will be corrected to 2m upon save. To set an hight of 1 m just enter "1" but "0" is nogo, landing is a different command (see there).
   Reaching the correct altitude (or hightchange between waypoints) is of low priority - that means a Waypoint is considered as done when the LAT/LON position (within the radius)
   is reached. To ensure a correct hight enter a loiterpoint somewhere with the wanted hight and a sufficient time (see there).



   MAV_CMD_NAV_LOITER_UNLIM
   ========================
   Name in MP drop down menue: "LOITER_UNLIM"
   What does it do:
   Loiters (almost) for ever. It can only be the last command in a WP list, otherwise the WP list is not accepted (ERR).
   It is treated as a waypoint that will be approached with braking. A hitradius must be defined or will be preset upon save.

   param2 Is the hitradius in meter. You can not define a smaller hitradius than preset in cli by "gps_wp_radius".

   param4 = 1 If a ROI is known it will nose in on the ROI point.
   param4 = 0 No special yaw. Nose still points in the former flightpath.

   Valid LAT/LON needed, otherwise the command is skipped upon save.



   MAV_CMD_NAV_LOITER_TURNS
   ========================
   Name in MP drop down menue: "LOITER_TURNS"
   What does it do:
   Loiters and yaws/rotates copter.
   It is treated as a waypoint that will be approached with braking. A hitradius must be defined or will be preset upon save.

   param1 (aka "Turns") describes the number of turns done before proceeding. A positive number will cause CW turns, a negative number CCW turn.
   Maximal values: -127..+127. param1 = 0 is not tolerated and is saved as "1" to ensure at least one cw turn.
   I am still unsure how to implement the actual yaw speed, however it will be gps_yaw dependant.
   I guess I will end up with param3 or so setting the time in seconds for one complete turn. Let's see when it comes to this.
   ALT/LAT/LON used as usual.

   param2 Is the hitradius in meter. You can not define a smaller hitradius than preset in cli by "gps_wp_radius".

   Valid LAT/LON needed, otherwise the command is skipped upon save.



   MAV_CMD_NAV_LOITER_TIME
   =======================
   Name in MP drop down menue: "LOITER_TIME"
   What does it do:
   Loiter some time.
   It is treated as a waypoint that will be approached with braking. A hitradius must be defined or will be preset upon save.

   param1 (aka "Time s") Defines the time in seconds. Max 255 sec.
   "0" is not tolerated and is saved as "1" to ensure at least one second loiter.

   param2 Is the hitradius in meter. You can not define a smaller hitradius than preset in cli by "gps_wp_radius".

   param4 (aka "yaw per") Will not be accepted as a real yaw angle.
   param4 = 1 If a ROI is known it will nose in on the ROI point. To use that define a ROI BEFORE. (see ROI)
   param4 = 0 ROI is ignored, copter nose points to the next WP.

   Valid LAT/LON needed, otherwise the command is skipped upon save.



   MAV_CMD_NAV_RETURN_TO_LAUNCH
   ============================
   Name in MP drop down menue: "RETURN_TO_LAUNCH"
   What does it do:
   It will just (nose in return) to the physical launch place (not the MP "homepos"). Landing is a different command (see there).
   LAT/LON are ignored (and saved as "0"), the in flight LAT/LON is taken.
   Don't forget ALT value.
   This command can not stand alone (ERR). That means at least one other WP has to be in the list.


   MAV_CMD_NAV_LAND
   ================
   Name in MP drop down menue: "LAND"
   What does it do:
   It will land at the current position. No other parameter accepted/used.
   It can only be the last command in a WP list (ERR).
   This command can not stand alone (ERR). That means at least one other WP has to be in the list.


   MAV_CMD_NAV_TAKEOFF
   ===================
   Name in MP drop down menue: "TAKEOFF"
   What does it do:
   It does a takeoff at the current position, defined by cli parameters: as_lnchr, as_clmbr, as_stdev and esc_nfly.
   ALT is the targethight of the launch, no matter what is defined in cli as_trgt.
   ALT must be in the range: 2..255m, other values will be constrained to that range.
   It can only be the first command (even a "ROI" has to come later) in a WP list (ERR).
   This command can not stand alone (ERR). That means at least one other WP has to be in the list.
   If the mission is engaged when the copter is already airborne the Takeoff command is skipped for obvious reason, but ALT is taken as minimal hight.
   That means if the mission was engaged airborne but below the specified Takeoff ALT, a climb will be initiated.
   If engaged above that hight it will approach the mission "homepos" at current hight and alter it according to further request.



   MAV_CMD_NAV_ROI
   ===============
   Name in MP drop down menue: "ROI"
   What does it do:
   It defines a ROI for MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LOITER_UNLIM, MAV_CMD_NAV_LOITER_TIME.
   A ROI is just defined by LAT/LON. ALT is not needed since we have no gimbal stuff going on. ALT value will be set to 0.
   In opposite to arducopter you don't have to define a ROI each time before calling an dependent function (like listed above).
   Just make sure a ROI is defined before those functions are called (see there). If you set those functions to use a ROI but ROI
   isn't defined, they will be set to "don't use ROI" upon save.
   You can define multiple ROIs in a WP list but only the previously defined is used for the current (and dependent) entry.
   A WP list with no WP and just a ROI are not saved (ERR). ROI can not be the last command in the WP list (ERR).
   A ROI with LAT or LON = 0 is considered faults and is skipped upon save.

-- Note: If you enter an unknown instruction in the Missionplaner WP list, it will be ignored and skipped on saving.
   You can check what naze makes out of your mission by hitting reload on missionplaner so it will reload the WP like stored in naze.
   For whatever mavlink reason it stores the GPS coordinates not in the precise "int32" manner but in normal "float". That means you will
   see little rounding errors in the GPS coordinates (compare save/load) that is not a fault in the code, but the nature of the mavlink protocol.
- Rearranged the limitcheck of gps tiltangle influence
- Reworked that degree wrapping stuff
- Slight rework of bearing calculation


MSP/Serial:
- Added "OSD SW" for KV_Team_OSD, thanks Hinkel for the input: http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=350#p42658
- Added MSP_STATUS profile 0 like suggested by disq http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=320#p42354
- Decode only one mwii/mavlink command in one cycle when looptime is set low or the code produces timespikes greater than the predefined looptime.
  This is just done now if one fine day the looptime exceeds 3ms so you will not notice that.
- Cycletime out is now smoothed for gui to see average cycletime.
- looptime is limited to minimal 1000, since Sensors will not do more than 1KHZ anyway..


RC:
- Added serial Graupner SumH support from here: http://fpv-treff.de/viewtopic.php?f=18&t=1368&start=3020#p44535 (some changes for integration made)
- Added Devo RSSI output (mwii & mavlink protocol) for details ask Bamfax :)
  here: http://fpv-treff.de/viewtopic.php?f=18&t=2181, http://fpv-community.de/showthread.php?33343-DSM-ohne-Spektrum-mit-SumPPM-12-Kan%E4len-und-RSSI
- Added a cutoff for rssi "set rssicut X" [0-80%][0 Disables/Default] Below that percentage rssi will show zero.
- Did not implement rssi for failsave because if the rssi is reported wrong (cable, devocode etc.) and the user is still in control a fs wouldn't be too great.

CLI / Setup & Motor/ESC:
- Removed bug in cli auxset: removing a not set box would set it. Added a function to clear all auxboxes(auxset --). Improved Auxset code.
- Status Spits out a great deal of calibration data, mag, baro, acc/gyro info, shows unused/free bytes in eeprom. Only onboard sensors, not sonar etc.
- Reworked dump function to make it better human readable.
- Set minimal looptime to 3000
- Added: "wpflush" to clear the WP list, but you will have to type "save" when you are done with cli to make it happen. An "exit" will not do it.
-- You can alternatively clear the wp list in missionplaner by defining a mission with one WP that has lat or lon with "0" value.
- Slight change of feature pass. The passmotor is always reset to 0 (use all motors) on de/activation of feature pass.
- Added Motorstatistics (see MotorStats.gif) to show average values for max 8 Motors. They are NOT stored. After a flight you can see in cli (type "status"):
-- Motornumber (like shown in the BF Manual)
-- The relative load of each motor during flight in percent, absolute throttle not taken into account here (total flightthrottle is taken as 100%).
-- The absolute PWM val for each motor including minthrottle.
-- The relative motorload in percent in relation to the throttlerange, to see how much headroom is left.
  Purpose: On test flights of a new setup you can see if motors are stressed more in an load unbalanced copter.
  Note: Motordata are gathered at 10Hz rate. The percentages are calculated without floatpoint or correct rounding because it's not about 0.x% precision here.
  Note: ESC RPM response maybe not linear to PWM input (that is measured here for display). Depends on ESC.
  Note: I don't know if this is useful at all, since i have only symetric X - shape quadrocopters that are easy to balance. If you own a copter with some spidertype
        frame, please report back if this is useful or not.

Deleted:
- Deleted U_ID_0.
- Deleted BOXCAMTRIG (no hardwarepin or code behind that anyway)
- Deleted FY90Q drv_pwm_fy90q.c, drv_adc_fy90q.c, drv_i2c_soft.c
- Deleted "FEATURE_POWERMETER" because there was/is no code behind it.
- Deleted FEATURE_GYRO_SMOOTHING (replaced)
- Deleted mag_oldctime and mag_oldcalib (replaced by mag_time - cal time in minutes)
- Deleted cli function "aux" because it is replaced by human readable "auxset"
- Deleted gps_ins_mdl alternative model didn't work out, so no gps ins model to choose from.
- Deleted mainpt1cut replaced by maincuthz
- Deleted gpspt1cut replaced by gpscuthz
- Deleted gps_ph_abstub replaced with gps_expo
- Deleted nav_slew_rate replaced with gps_expo
- Deleted gps_phase thought it is useful but not needed...
- Deleted acc_lpf repaced by acc_lpfhz
- Deleted acc_ilpf repaced by acc_altlpfhz
- Deleted FEATURE_INFLIGHT_ACC_CAL repaced by AirTrim
- Deleted newpidimax

Changed Variablenames:
(Purpose:
1: Sort/group by functionality. Where possible, the first 2-3 chars should represent the MAIN purpose / affiliation followed by a "_".
That's the basic idea behind that.
2: Try to stuff into 8 chars for better mavlink- and generaldisplay.
These goals will not be possible with all variables.
Still boring work ahead, but had to be started sometime.)

OLD                          NEW
maxcheck                  -> rc_maxchk
mincheck                  -> rc_minchk
midrc                     -> rc_mid
killswitchtime            -> rc_killt
auxChannels               -> rc_auxch
retarded_arm              -> rc_rllrm (Arming/Disarming with roll is dangerous when doing flips!)
deadband                  -> rc_db
yawdeadband               -> rc_dbyw
alt_hold_throttle_neutral -> rc_dbah
gps_adddb                 -> rc_dbgps
failsafe_delay            -> fs_delay
failsafe_off_delay        -> fs_ofdel
failsafe_throttle         -> fs_rcthr
failsafe_deadpilot        -> fs_ddplt
failsafe_justph           -> fs_jstph
failsafe_ignoreSNR        -> fs_nosnr
snr_debug                 -> snr_dbg
minthrottle               -> esc_min
maxthrottle               -> esc_max
mincommand                -> esc_moff
al_lndthr                 -> esc_nfly (ESC NOFLY, is a copter dependant value greater than esc_min. It is important in autostart, landing disarm and failsafe plausibility throttlecheck. If 0 esc_min+5% is taken)
motor_pwm_rate            -> esc_pwm
servo_pwm_rate            -> srv_pwm
passmotor                 -> pass_mot
mag_declination           -> mag_dec
gyro_cmpf_factor          -> gy_gcmpf
gyro_cmpfm_factor         -> gy_mcmpf
gyro_lpf                  -> gy_lpf
moron_threshold           -> gy_stdev (Allowed Standard Deviation during gyro initialization)
acc_hardware              -> acc_hdw
acc_lpf_factor            -> acc_lpf
acc_ins_lpf               -> acc_ilpf
accz_vel_cf               -> accz_vcf
accz_alt_cf               -> accz_acf
yaw_direction             -> tri_ydir
tri_yaw_middle            -> tri_ymid
tri_yaw_min               -> tri_ymin
tri_yaw_max               -> tri_ymax
triywdel                  -> tri_ydel
barodownscale             -> bar_dscl
baro_lag                  -> bar_lag
baro_debug                -> bar_dbg
gimbal_pitch_gain         -> gbl_pgn
gimbal_roll_gain          -> gbl_rgn
gimbal_flags              -> gbl_flg
gimbal_pitch_min          -> gbl_pmn
gimbal_pitch_max          -> gbl_pmx
gimbal_pitch_mid          -> gbl_pmd
gimbal_roll_min           -> gbl_rmn
gimbal_roll_max           -> gbl_rmx
gimbal_roll_mid           -> gbl_rmd
gps_rtl_minhight          -> rtl_mnh
gps_rtl_mindist           -> rtl_mnd


Changed Var Values:
mainpidctrl = X   0 = Altered Original, 1 = New controller (was 1 & 2 before)
al_barolr & al_snrlr // [10 - 200cm/s DEFAULT:50] Baro/Sonar Landingrate


Naze32 Harakiri10 Summer Games2.5
=================================
- Experimental IMU Changes. Better MPU Gyro usage. All acc scaled to "512" for 1G
- Precision improved ACC/Gyro calibration.
- MPU6050 "single shot" readout like suggested by Sebbi
- Changed MPU Acc setting from 8G to 4G, increasing resolution without observing saturation effect (right now...).
- Experimental IMU Change for GPS
- gyro_cmpf_factor changed to 1000 (from 400)
- Added vario data transmission within MSP_ALTITUDE


Naze32 Harakiri10 Summer Games2.4 WITH EXPERIMENTAL LOGGING
===========================================================
- LLIGHTS & LEDMAX Deleted (Had no use in BF anyway)

// Logging don't expect anything of it yet
// - Introduced some "GPS LOG Box" will record when armed, only one log possible. So restarting logging will delete old log.
// - Introduced gpslog in cli to show dataset(s)
// - GPS Logging for 19,5 Minutes for LAT/LON/ALT/HEADING at 0.5 HZ (every 2 sec new dataset)
// - Logging precision is slightly decreased for compression reasons, one Dataset needs just 4 Bytes, so 2,3KB can store 20Min flight
// - Flightstats couldn't display negative alt, basic stats can be saved now. When doing a general save on eeprom (like save in cli or write in gui, or if logging anyway)
// - Flightstats will be cleared at power up with the default stat_clear = 1. With 0 the last saved stats will be taken into account concerning max speed/hight etc..

Core changes:
- Reintroduced old, (t)rusty MTK parser.
- Removed the fix acc_1G value from multiwii and calculate real acc_1G during calibration. So you will not see that fix "512" for mpu any more. On the first run altitude will show a flatline
  until ACC calibrated (wait a little for save) and FC repowered
- Moved Gyro calibration, angle calculation to float point calculations.
- INS FACTORS WILL HAVE TO BE RETUNED - OMG. gps_ins_vel reduced to 0.6. nav_slew_rate set to 20 now (reducing, increases strength). accz_vel_cf untouched seems to fit.
- gps_tbe_speed deleted was of limited use
- PH/RTL Bug fixed (actual position could be ignored esp. on RTH)
- PosrI put to work (and scaled further down by /100) but just with the velocity error that is calculated from position error (posP). So will hopefully have effect now without circeling. Default 0.Untested.
- Stay away from the feature inflightcalibration it will probably kill INS/ACC functions because it isn't affecting the trims but the real acc calibration. - Outdated mwii code.
- Reworked MsBaroDriver a little (probably slight resolution increase)

Naze32 Harakiri10 Summer Games2.3
=================================
- Little update just concerning ublox parser. Some people reportet uBLox 6M didn't show Lat/Lon but satcount.
- So ublox parser is redone here and it is a mixture of current BF/Mwii and Arducopter driver
- I hope it resolves that issue. Works the same on my rig than the old one
- Reduced the defaultPIDs, because the correct controller is more aggressive

Naze32 Harakiri10 Summer Games2.2
=================================
- Just updated main PID controller (mainpidctrl = 1 (default)) according to BRM's suggestions here: http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=150#p38927
- Relaxed timings on ublox startup configuration
- Minor mavlink changes

Naze32 Harakiri10 Summer Games2.1
=================================
- Just updated the Arducopter "plain earth" bearing calculation to a little more STM like correct, spherical Bearing.
Forumla:http://www.movable-type.co.uk/scripts/latlong.html under "BEARING"
JavaScript: 	
var y = Math.sin(dLon) * Math.cos(lat2);
var x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1)*Math.cos(lat2)*Math.cos(dLon);
var brng = Math.atan2(y, x).toDeg();
I don't understand that but RTL works to the point now :)

Harakiri Summergames2
=====================

- Feature telemetry is gone and replaced by "tele_prot = X"
- Some basic Mavlinksupport. It's beta but there are a few things you can do already.
- Autosensing Mavlink/Multiwii protocols (can change every 4 seconds)
- Due to Autosensing you will see the Mavlink "1Hz Heartbeat - garbage" in cli.
- Entering the CLI requires three times "#" now! ("###"). Alternatively 3 times <RETURN>. CLI entering is only possible when disarmed.
- Flashing requires three times "R" now! ("RRR")
- Reworked PH PosrD Controller. PosrD is scaled down now (factor 30)
- Re-enabled crosstrack support. Crosstrack gain can be adjusted or turned off.


Telemetry / Mavlink
===================
Since "feature telemetry" was just for FRsky it was misleading. Some people might do multiwii/bt telemetry and are mislead by that.
Now you can "set tele_prot = X".
That X will decide, what is done on the usb/telem. port ONLY WHEN ARMED. When disarmed the mavlink/multiwii autodetection will kick in.
So here is what that X stands for:
0 (Dfault) = Keep Multiwii @CurrentUSB Baud (like always)
1          = Frsky @9600Baud Turn to Frsky protocol at 9600 Baud when armed.
2          = Mavlink @CurrentUSB Baud. Some Mavlink 1.0 Packages are sent at low rate (ca <2Kb/s)
3          = Mavlink @57KBaud (like stock minimOSD wants it) Sends the same stuff like in mode "2" but only at 57K

NOTE: If you choose to work just with missionplaner and also want to feed minimosd you might want to set serial_baudrate = 57600 and set
set tele_prot to 2 or 3. So you will not loose MP connection on arming. Or recompile MinimOSD FW with 115KBaud setting.

Current Datastream over mavlink:
Heartbeat      @ 1  Hz: Armed/Disarmed, Copter Type, Stabilized/Acro.
Sys_status     @ 2  Hz: Sensors present and healthstatus. Voltage.
Attitude       @30  Hz: TimeStamp(ms), roll/pitch/yaw(compass heading - RAD)
RC_Channels    @ 2  Hz: RAW, unscaled 8 RC Channels and RSSI (currently unknown to naze)
VFR_HUD        @10  Hz: Speed measured by GPS, scaled Throttle (0-100%), Baro Altitude, Variometer, compass heading (again, this time in Degree)
Scaled_pressure@ 0.5Hz: Gyro Temperature, Airpressure in hPa (mBar)

Note: The Datarates are wishful thinking, because only one Datapacket is send per (100Hz-)cycle to keep the serial rate low. So "Attitude"
is actually send at 25.xHz rate. The real rates are "as high" like the APM sends them. I checked it with GCS. Attitude is 10Hz at Arducopter, so that's faster now.
These Data are send without request, once Mavlink is established. MinimOSD is not tested but should be able to read and display something from the datastream.
Datastreamrequests are currently not handled (with one exception) so for testing you can connect minimOSD only with its' RX pin (same like in arducopter telem mode).
Let me know if minimOSD works with that (set tele_prot = 3).
The only request that is handled is, when Missionplaner requests Parameterlist (won't work with the windows GCS soft I tested).
Missionplaner will load the Parameterlist on start. It will be accessible as advantaged parameter list. You can change and write and compare whole parametersets.
Note: New MP will cry out for missing Arducopterparameters. The new MP doesn't show altitude and some other data on mainscreen - they changed that.
My old and rusty Mission Planner 1.2.38 shows everything and is not so picky. So go for older MP.
Note: Naze parameters will be crippled to 16 chars if necessary. No problem.
WP etc is not supported right now. The main reason for that is that i am having trouble with Naze EEPROM writing more than 1KB data.


Changes in PH
=============
Parameters used:
PosrP (Strength of velocity influence, tries to keep velocity, normally "0" in PH, but might change with PosP - see below)
PosrI (keep it 0, might lead to circeling)
PosrD is rescaled now.

PosP Works in conjunction with PosrP. And defines how much a POSITION error is translated to an VELOCITY error.

Parameters NOT USED: PosI and PosD

gps_ph_brkacc = 40 (Dfault)  // [1 - 500] Is the assumed negative braking acceleration in cm/(s*s) of copter. Value is positive though. It will be a timeout. The lower the Value the longe the Timeout.

gps_ph_abstub = 150 (Dfault) // 0 - 1000cm (150 Dfault, 0 disables) Defines the "bath tub" around current absolute PH Position, where PosP is diminished, reaction gets harder on tubs edge and then goes on linear.
I changed the form of the bathtub -> see attached picture. The bathtub is for absolute position (influence set by PosP).

gps_tbe_speed = 0(Dfault)    // 0 - 1000 (0 disables) Speed in cm/s for TBE detection MUST be greater than gps_ph_settlespeed or it will be disabled
When at the end of the PH chain a movement with that speed (150cm/s) is detected for 2 seconds a toilet bowl is assumed and the PH cascade is redone, keeping the target PH Position in mind.
Thanks "HINKEL" for the idea!
WARNING: gps_tbe_speed Also can disable PH in wind, because if it is carried away with 150cm/s a TBE will be assumed! Set it to 0 to disable it.


Changes in NAVIGATION
=====================
The final stage of a RTH is the PH. If you move the sticks and oversteer, the PH cascade is reset and a new GPS targetpoint is set. It will not land at the once assumed GPS pos anymore thats on purpose.

nav_tiltcomp is reduced to 20  // 0 - 100 (20 TestDefault) Only arducopter really knows. Dfault was 54. This is some kind of a hack of them to reach actual nav_speed_max. 54 was Dfault, 0 disables

nav_ctrkgain = 0.5  // 0 - 10.0 (0.5 TestDefault) That is the "Crosstrackgain" APM Dfault is "1". "0" disables
Re - Introduced Crosstrack. I think it is not neccessary for copters. You can disable it with "0".
See for details: http://diydrones.com/profiles/blogs/705844:BlogPost:43438





