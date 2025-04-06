        //* Glade builder setup
        builder->get_widget("dash_window", dash_window);
        builder->get_widget("dash_top_layout", dash_layout);
            //* subsys Status Grid
            builder->get_widget("subsys_status_grid", system_health_control_base.grid);
            system_health_control_base.system.resize(NUM_MONITORED_SYSTEMS_CONTROL_BASE);
            for (int i = 0; i < NUM_MONITORED_SYSTEMS_CONTROL_BASE; i++) {
              std::string index = std::to_string(i);
              builder->get_widget("subsys_" + index + "_label", system_health_control_base.system[i].name);
              builder->get_widget("subsys_" + index + "_status_label", system_health_control_base.system[i].status);
              builder->get_widget("subsys_" + index + "_kill_button", system_health_control_base.system[i].kill_button);
              builder->get_widget("subsys_" + index + "_run_button", system_health_control_base.system[i].run_button);
          }


                // builder->get_widget("subsys_0_label", system_health.system[0].name);
                //   builder->get_widget("subsys_0_kill_button", system_health.system[0].kill_button);
                //   builder->get_widget("subsys_0_run_button", system_health.system[0].run_button);
                // builder->get_widget("subsys_1_label", system_health.system[1].name);
                // builder->get_widget("subsys_2_label", system_health.system[2].name);
                // builder->get_widget("subsys_3_label", system_health.system[3].name);
                // builder->get_widget("subsys_4_label", system_health.system[4].name);
                // builder->get_widget("subsys_5_la bel", system_health.system[5].name);
                // builder->get_widget("subsys_0_status_label", system_health.system[0].status);
                // builder->get_widget("subsys_1_status_label", system_health.system[1].status);
                // builder->get_widget("subsys_2_status_label", system_health.system[2].status);
                // builder->get_widget("subsys_3_status_label", system_health.system[3].status);
                // builder->get_widget("subsys_4_status_label", system_health.system[4].status);
                // builder->get_widget("subsys_5_status_label", system_health.system[5].status);

            
                for(int i = 0; i < NUM_MONITORED_SYSTEMS_CONTROL_BASE; i++){
                    system_health_control_base.system[i].name->set_label(monitored_system_names[i]);
                    system_health_control_base.system[i].kill_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names[i], KILL, COMPUTER_GLOBAL));
                    system_health_control_base.system[i].run_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names[i], RUN, COMPUTER_GLOBAL));
                    system_health_control_base.system[i].status->set_label("UNKNOWN");
                    monitored_systems_control_base[monitored_system_names[i]].status_label = system_health_control_base.system[i].status;
                    monitored_systems_control_base[monitored_system_names[i]].name_label = system_health_control_base.system[i].name;
                    monitored_systems_control_base[monitored_system_names[i]].status_label->set_label("OFFLINE");
                    Glib::RefPtr<Gtk::StyleContext> context = monitored_systems_control_base[monitored_system_names[i]].status_label->get_style_context();
                    context->remove_class("dash_status_unknown");
                    context->add_class("subsys_OFFLINE");

                }


                //* Onboard Nuc
                system_health_onboard_nuc.system.resize(NUM_MONITORED_SYSTEMS_ONBOARD_NUC);
                for (int i = 0; i < NUM_MONITORED_SYSTEMS_ONBOARD_NUC; i++) {
                  std::string index = std::to_string(i);
                  builder->get_widget("subsys_" + index + "_label_onboard_nuc", system_health_onboard_nuc.system[i].name);
                  builder->get_widget("subsys_" + index + "_status_label_onboard_nuc", system_health_onboard_nuc.system[i].status);
                  builder->get_widget("subsys_" + index + "_kill_button_onboard_nuc", system_health_onboard_nuc.system[i].kill_button);
                  builder->get_widget("subsys_" + index + "_run_button_onboard_nuc", system_health_onboard_nuc.system[i].run_button);
              }
    
    
                
                    for(int i = 0; i < NUM_MONITORED_SYSTEMS_ONBOARD_NUC; i++){
                        system_health_onboard_nuc.system[i].name->set_label(monitored_system_names[i]);
                        system_health_onboard_nuc.system[i].kill_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names[i], KILL, COMPUTER_GLOBAL));
                        system_health_onboard_nuc.system[i].run_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), monitored_system_names[i], RUN, COMPUTER_GLOBAL));
                        system_health_onboard_nuc.system[i].status->set_label("UNKNOWN");
                        monitored_systems_onboard_nuc[monitored_system_names[i]].status_label = system_health_onboard_nuc.system[i].status;
                        monitored_systems_onboard_nuc[monitored_system_names[i]].name_label = system_health_onboard_nuc.system[i].name;
                        monitored_systems_onboard_nuc[monitored_system_names[i]].status_label->set_label("OFFLINE");
                        Glib::RefPtr<Gtk::StyleContext> context = monitored_systems_onboard_nuc[monitored_system_names[i]].status_label->get_style_context();
                        context->remove_class("dash_status_unknown");
                        context->add_class("subsys_OFFLINE");
    
                    }