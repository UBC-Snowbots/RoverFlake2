#include <arm_panel_node.h>
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto app = Gtk::Application::create(argc, argv, "com.example.GtkAppliation");
    auto node = std::make_shared<ArmPanelNode>();

    // GtkBuilder *builder;
    // GtkWidget *window;
    // GError *error = nullptr;

    // gtk_init(&argc, &argv);
    
//     std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_glade_gui");
//     std::string glade_file_path = package_share_directory + "/glade_files/arm_panel.glade";

//     builder = gtk_builder_new();
//     if (!gtk_builder_add_from_file(builder, glade_file_path.c_str(), &error))
//     {
//         g_critical("Error loading UI file: %s", error->message);
//         g_error_free(error);
//         return 1;
//     }
//     //*Top level window setup, and creates destruction object 
//     window = GTK_WIDGET(gtk_builder_get_object(builder, "arm_panel_top_window"));
//     g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), nullptr);

//     // GtkWidget *sov_1_button = GTK_WIDGET(gtk_builder_get_object(builder, "sov_1_button"));

//     // // GtkButton *sov_1_button = GTK_BUTTON(gtk_builder_get_object(builder, "sov_1_button"));
//     // GtkButton *sov_2_button = GTK_BUTTON(gtk_builder_get_object(builder, "sov_2_button"));
//     // GtkWidget *graph_press_all = GTK_WIDGET(gtk_builder_get_object(builder, "test_drawing"));

//     // g_signal_connect(graph_press_all, "draw", G_CALLBACK(on_draw), node.get());
//     // g_signal_connect(graph_press_all, "button_release_event", G_CALLBACK(on_graph_clicked), node.get());
//     // g_signal_connect(sov_1_button, "clicked", G_CALLBACK(sov_1_clicked), node.get());
       
//     // g_signal_connect(sov_2_button, "clicked", G_CALLBACK(sov_2_clicked), node.get());

//        GtkCssProvider* css_provider;
//     css_provider = gtk_css_provider_new();
//     // gtk_css_provider_load_from_path(css_provider, "styles.css", NULL);

//     GtkStyleContext* context = gtk_widget_get_style_context(main_layout);
//     gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(css_provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
//      std::string style_file_path = package_share_directory + "/glade_files/main_style.css";
//      gtk_css_provider_load_from_path(css_provider, style_file_path.c_str(), NULL);

//    // load_css();
//     gtk_widget_show_all(window);

//     std::thread spinner([&]() {
//         rclcpp::spin(node);
//     });

//     gtk_main();
    node->app = app;
    node->run();
    rclcpp::shutdown();
    return 0;
}

// void load_css(){
//     gtk_css_provider_load_from_path(css_provider, "/glade_files/main_style.css", NULL);
//     // gtk_css_provider_load_from_path(css_provider, "labels.css", NULL); //TODO add more as needed, if we need to better orginize the css files into seperate files
// }