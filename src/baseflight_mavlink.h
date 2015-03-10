#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/
   #define inline __inline
#endif

#define MLSystemID 1 
#define MLComponentID 200
#define MAVLINK_H
#define MAVLINK_STX 254
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#define MAVLINK_ALIGNED_FIELDS 1
#define MAVLINK_CRC_EXTRA 1
/*
 * applications can set MAVLINK_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
#define MAVLINK_COMM_NUM_BUFFERS 1
#include "v1.0/common/common.h"

#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/
   #undef inline
#endif

void     reset_mavlink(void);
bool     baseflight_mavlink_receive(char new);
void     baseflight_mavlink_send_message (mavlink_message_t* msg);
void     baseflight_mavlink_handleMessage (mavlink_message_t *msg);
void     baseflight_mavlink_send_updates(void);
bool     baseflight_mavlink_send_paramlist(bool Reset);
bool     baseflight_mavlink_set_param (mavlink_param_set_t *packet);
