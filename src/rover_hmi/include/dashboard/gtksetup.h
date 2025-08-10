        //* Glade builder setup
        builder->get_widget("dash_window", dash_window);
        builder->get_widget("dash_top_layout", dash_layout);
            //* subsys Status Grid
            builder->get_widget("control_base_subsys_status_grid", system_health_control_base.grid);
            system_health_control_base.system.resize(NUM_MONITORED_SYSTEMS_CONTROL_BASE);
            monitored_system_names_control_base.resize(NUM_MONITORED_SYSTEMS_CONTROL_BASE);
 
            for (int i = 0; i < NUM_MONITORED_SYSTEMS_CONTROL_BASE; i++) {
              std::string index = std::to_string(i);
              builder->get_widget("control_base_subsys_" + index + "_label", system_health_control_base.system[i].name);
              builder->get_widget("control_base_subsys_" + index + "_status_label", system_health_control_base.system[i].status);
              builder->get_widget("control_base_subsys_" + index + "_run_button", system_health_control_base.system[i].run_button);
              builder->get_widget("control_base_subsys_" + index + "_kill_button", system_health_control_base.system[i].kill_button);
          }

                builder->get_widget("ptz_tilt_increase", ptz_buttons.tilt.inc);
                    ptz_buttons.tilt.inc->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), TILT_INC, true));
                    ptz_buttons.tilt.inc->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), TILT_INC, false));

                builder->get_widget("ptz_tilt_decrease", ptz_buttons.tilt.dec);
                    ptz_buttons.tilt.dec->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), TILT_DEC, true));
                    ptz_buttons.tilt.dec->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), TILT_DEC, false));
                builder->get_widget("ptz_pan_increase", ptz_buttons.pan.inc);      
                    ptz_buttons.pan.inc->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), PAN_INC, true));
                    ptz_buttons.pan.inc->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), PAN_INC, false));

                builder->get_widget("ptz_pan_decrease", ptz_buttons.pan.dec);
                    ptz_buttons.pan.dec->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), PAN_DEC, true));
                    ptz_buttons.pan.dec->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), PAN_DEC, false));

                builder->get_widget("ptz_zoom_increase", ptz_buttons.zoom.inc);
                    ptz_buttons.zoom.inc->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), ZOOM_INC, true));
                    ptz_buttons.zoom.inc->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), ZOOM_INC, false));

                builder->get_widget("ptz_zoom_decrease", ptz_buttons.zoom.dec);
                    ptz_buttons.zoom.dec->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), ZOOM_DEC, true));
                    ptz_buttons.zoom.dec->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::ptzButtonCallback), ZOOM_DEC, false));


            
                for(int i = 0; i < NUM_MONITORED_SYSTEMS_CONTROL_BASE; i++){
                    system_health_control_base.system[i].name->set_label(monitored_system_names_control_base[i]);
                    system_health_control_base.system[i].kill_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names_control_base[i], KILL, COMPUTER_GLOBAL));
                    system_health_control_base.system[i].run_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names_control_base[i], RUN, COMPUTER_GLOBAL));
                    system_health_control_base.system[i].status->set_label("UNKNOWN");
                    monitored_systems_control_base[monitored_system_names_control_base[i]].status_label = system_health_control_base.system[i].status;
                    monitored_systems_control_base[monitored_system_names_control_base[i]].name_label = system_health_control_base.system[i].name;
                    monitored_systems_control_base[monitored_system_names_control_base[i]].status_label->set_label("OFFLINE");
                    Glib::RefPtr<Gtk::StyleContext> context = monitored_systems_control_base[monitored_system_names_control_base[i]].status_label->get_style_context();
                    context->remove_class("dash_status_unknown");
                    context->add_class("subsys_OFFLINE");

                }

                builder->get_widget("control_base_heart_monitor", control_base_watch_grid.line_draw_area);
                builder->get_widget("control_base_status_label", control_base_watch_grid.status_label);
                    control_base_watch_grid.line_draw_area->signal_draw().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::handleSystemStatusGridDraw), control_base_watch_grid, control_base));

                builder->get_widget("on_board_nuc_heart_monitor", on_board_nuc_watch_grid.line_draw_area);
                builder->get_widget("on_board_nuc_status_label", on_board_nuc_watch_grid.status_label);
                    on_board_nuc_watch_grid.line_draw_area->signal_draw().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::handleSystemStatusGridDraw), on_board_nuc_watch_grid, onboard_nuc)); // TODO AHHH ON_BOARD NOT ONBOARD OR MAYBE ONBOARD MEOW

                //* Onboard Nuc
                system_health_onboard_nuc.system.resize(NUM_MONITORED_SYSTEMS_ONBOARD_NUC);
                monitored_system_names_onboard_nuc.resize(NUM_MONITORED_SYSTEMS_ONBOARD_NUC);


                for (int i = 0; i < NUM_MONITORED_SYSTEMS_ONBOARD_NUC; i++) {
                  std::string index = std::to_string(i);
                  builder->get_widget("onboard_nuc_subsys_" + index + "_label", system_health_onboard_nuc.system[i].name);
                  builder->get_widget("onboard_nuc_subsys_" + index + "_status_label", system_health_onboard_nuc.system[i].status);
                  builder->get_widget("onboard_nuc_subsys_" + index + "_kill_button", system_health_onboard_nuc.system[i].kill_button);
                  builder->get_widget("onboard_nuc_subsys_" + index + "_run_button", system_health_onboard_nuc.system[i].run_button);
              }
    
    
                
                    for(int i = 0; i < NUM_MONITORED_SYSTEMS_ONBOARD_NUC; i++){
                        system_health_onboard_nuc.system[i].name->set_label(monitored_system_names_onboard_nuc[i]);
                        system_health_onboard_nuc.system[i].kill_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names_onboard_nuc[i], KILL, COMPUTER_GLOBAL));
                        system_health_onboard_nuc.system[i].run_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names_onboard_nuc[i], RUN, COMPUTER_GLOBAL));
                        system_health_onboard_nuc.system[i].status->set_label("UNKNOWN");
                        monitored_systems_onboard_nuc[monitored_system_names_onboard_nuc[i]].status_label = system_health_onboard_nuc.system[i].status;
                        monitored_systems_onboard_nuc[monitored_system_names_onboard_nuc[i]].name_label = system_health_onboard_nuc.system[i].name;
                        monitored_systems_onboard_nuc[monitored_system_names_onboard_nuc[i]].status_label->set_label("OFFLINE");
                        Glib::RefPtr<Gtk::StyleContext> context = monitored_systems_onboard_nuc[monitored_system_names_onboard_nuc[i]].status_label->get_style_context();
                        context->remove_class("dash_status_unknown");
                        context->add_class("subsys_OFFLINE");
    
                    }





    // GNSS
        builder->get_widget("gnss_save_button", gnss_save_button);
    builder->get_widget("gnss_point_name_entry",  gnss_point_name_entry);

    // connect their signals to your methods
        gnss_save_button->signal_clicked()
                .connect(sigc::mem_fun(*this, &DashboardHMINode::on_gnss_save_button_clicked));

        gnss_point_name_entry->signal_activate()
               .connect(sigc::mem_fun(*this, &DashboardHMINode::on_gnss_point_name_entry_activated));


builder->get_widget("campipe_1_next", campipe_1.button.inc);
builder->get_widget("campipe_1_prev", campipe_1.button.dec);

builder->get_widget("campipe_2_next", campipe_2.button.inc);
builder->get_widget("campipe_2_prev", campipe_2.button.dec);

builder->get_widget("campipe_1_selected_label", campipe_1.selected_label);
builder->get_widget("campipe_2_selected_label", campipe_2.selected_label);

