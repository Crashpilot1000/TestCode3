TestCode3
=========

Note: The changes after commit 30 are mostly to get Gtune working. Testing on my 2 units suggests it is very well. Yaw tune is blocked for everything with less than 4 motors. ToDo: The function needs a writeup in the readme.txt. There is no special function for saving the tuned parameters. If after tuning an eeprom write is triggered (like saving trims or hitting save in gui etc.) the tuned P values will be saved along. I know it is dirty but it will be left that way to save ram and not cast an useless eepromwrite. Since you can set a workingrange for the P values you can have it running all the time. It is not tested with TPA (throttle pid attenuation) and it is not supposed to work with it, however gtune can replace TPA.

- A compiled (hex and bin) version including commit 30 is available here
http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&p=54324#p54324

- A compiled (hex and bin) version including commit 26 is available here http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&p=54219#p54219


