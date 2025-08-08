 // Terrible fucking way of manageing dependencies. JUst a horrible header.
 
 Gtk::Window* dash_window;
    Gtk::Layout* dash_layout;

      //* Global Messaging System
      Gtk::Label* global_msg_label;

struct ComputerWatchGrid 
{
  Gtk::Widget* line_draw_area;
  Gtk::Label* status_label;
 };
ComputerWatchGrid control_base_watch_grid;
ComputerWatchGrid on_board_nuc_watch_grid;
// ComputerWatchGrid* computer_watch_grids; //TODO change into array or smt

struct PTZCameraControllerGrid
{
  button_pair_increase_decrease_t pan;
  button_pair_increase_decrease_t tilt;
  button_pair_increase_decrease_t zoom;

};
PTZCameraControllerGrid ptz_buttons;

//* Subsystem Status Grid

struct SubSysStatusElement
{
  Gtk::Label* name;
  Gtk::Label* status;
  Gtk::Button* kill_button;
  Gtk::Button* run_button;

};
   //* System Overview
struct SubSysStatusGrid
{
  Gtk::Grid* grid;
  std::vector<SubSysStatusElement> system;
};


struct CamPipe{
  int index = -1;
  RoverHmiCommon::button_pair_increase_decrease_t button;
  Gtk::Label* selected_label;

  std::string selected_topic;


};

  std::vector<std::string> camera_input_topics; // SET VIA PARAM
  std::vector<std::string> camera_output_topics; // SET VIA PARAM

CamPipe campipe_1;
CamPipe campipe_2;