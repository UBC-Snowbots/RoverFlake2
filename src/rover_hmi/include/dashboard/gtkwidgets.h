  Gtk::Window* dash_window;
    Gtk::Layout* dash_layout;

      //* Global Messaging System
      Gtk::Label* global_msg_label;



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

