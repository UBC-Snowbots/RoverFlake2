// May want to use a namespace but works for rough organization

#define NUM_MONITORED_SYSTEMS_CONTROL_BASE 6
#define NUM_MONITORED_SYSTEMS_ONBOARD_NUC 6
#define NUM_MONITORED_SYSTEMS_ONBOARD_JETSON 6

#define RUN 0xA
#define KILL 0xB

// Watchdog timeout is a ros parameter

//* Process types
#define LAUNCHFILE 0xA
#define NODE 0xB

#define COMPUTER_ONBOARD_NUC 0xA
#define COMPUTER_CONTROL_BASE 0xB
#define COMPUTER_GLOBAL 0x0

// #define COMPUTER_ONBOARD_NUC_STRING "onboard_nuc"
// #define COMPUTER_CONTROL_BASE_STRING "control_base"
// #define COMPUTER_GLOBAL_STRING 0x0

// Enums

enum computers{
  control_base,
  onboard_nuc,
  onboard_jetson,
};

enum PTZ_BUTTONS {
    TILT_INC,
    TILT_DEC,
    PAN_INC,
    PAN_DEC,
    ZOOM_INC,
    ZOOM_DEC,
    NUM_PTZ_BUTTONS
};

struct SystemProcess{
  int type = LAUNCHFILE;
  std::string pkg;
  std::string exec;
};
struct MonitoredSystem{ 
  std::string name;
  pid_t pid;
  pid_t gpid;
  pid_t sid;
  bool online = false;
  Gtk::Label* status_label;
  Gtk::Label* name_label;
  // std::vector<SubSystemProcess> processes; //! Currently, each subsystem can only run one process (so make it a launch file)
  SystemProcess process;
};
    // This is to monitor the hearts, so one monitor for each device/computer
    struct heart_monitor {
      std::string host_device_name;
      uint32_t time_of_last_heartbeat_ns = NULL;
      uint32_t time_of_last_heartbeat_s = NULL;
      rclcpp::Time time_of_heartbeat;
      std::vector<MonitoredSystem> systems;
    };
      