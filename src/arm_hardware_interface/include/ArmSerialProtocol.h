#define CONTROL_RATE 60.0
#define COMM_POLL_RATE 1000.0 // idea is to poll serial faster than arm can send messages. We don't wan't to miss any messages.

#define HOME_CMD 'h' // 104
#define HOME_ALL_ID 50

#define ABS_POS_CMD 'P' // 80
#define COMM_CMD 'C'    // 67
#define ABS_VEL_CMD 'V' // 86

#define TEST_LIMITS_CMD 't' // 116

// TODO implement arm abort

//? Limit switch feedback looks like:
//? sprintf(tmpmsg, "Limit Switch %d, is %d.  \n\r\0", i + 1, get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]));
