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

