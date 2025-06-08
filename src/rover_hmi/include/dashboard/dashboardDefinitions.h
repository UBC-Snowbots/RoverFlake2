// May want to use a namespace but works for rough organization

#define NUM_MONITORED_SYSTEMS_CONTROL_BASE 6
#define NUM_MONITORED_SYSTEMS_ONBOARD_NUC 6
#define NUM_MONITORED_SYSTEMS_ONBOARD_JETSON 6

#define RUN 0xA
#define KILL 0xB


//* Process types
#define LAUNCHFILE 0xA
#define NODE 0xB

#define COMPUTER_ONBOARD_NUC 0xA
#define COMPUTER_CONTROL_BASE 0xB
#define COMPUTER_GLOBAL 0x0

// #define COMPUTER_ONBOARD_NUC_STRING "onboard_nuc"
// #define COMPUTER_CONTROL_BASE_STRING "control_base"
// #define COMPUTER_GLOBAL_STRING 0x0