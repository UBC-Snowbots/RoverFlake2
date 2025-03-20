        //* Glade builder setup
        builder->get_widget("dash_window", dash_window);
        builder->get_widget("dash_top_layout", dash_layout);
            //* subsys Status Grid
            builder->get_widget("subsys_status_grid", system_health.grid);
                builder->get_widget("subsys_0_label", system_health.system[0].name);
                  builder->get_widget("subsys_0_kill_button", system_health.system[0].kill_button);
                  builder->get_widget("subsys_0_run_button", system_health.system[0].run_button);
                builder->get_widget("subsys_1_label", system_health.system[1].name);
                builder->get_widget("subsys_2_label", system_health.system[2].name);
                builder->get_widget("subsys_3_label", system_health.system[3].name);
                builder->get_widget("subsys_4_label", system_health.system[4].name);
                builder->get_widget("subsys_5_label", system_health.system[5].name);
                builder->get_widget("subsys_0_status_label", system_health.system[0].status);
                builder->get_widget("subsys_1_status_label", system_health.system[1].status);
                builder->get_widget("subsys_2_status_label", system_health.system[2].status);
                builder->get_widget("subsys_3_status_label", system_health.system[3].status);
                builder->get_widget("subsys_4_status_label", system_health.system[4].status);
                builder->get_widget("subsys_5_status_label", system_health.system[5].status);

            
                  system_health.system[0].kill_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), 0, KILL));
                  system_health.system[0].run_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &DashboardHMINode::subsystemRequest), 0, RUN));
                for(int i = 0; i < NUM_MONITORED_SYSTEMS; i++){
                    system_health.system[i].name->set_label(monitored_systems_names[i]);
                    system_health.system[i].status->set_label("UNKNOWN");
                }