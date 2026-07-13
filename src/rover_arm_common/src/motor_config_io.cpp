// Implements the motor_config.yaml accessors declared in motor_config.h.

#include <rover_arm_common/motor_config.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>

std::string motor_config_yaml_path() {
    try {
        return ament_index_cpp::get_package_share_directory("rover_arm_common")
               + "/config/motor_config.yaml";
    } catch (const std::exception&) {
        return {};
    }
}

std::vector<MotorConfig> get_arm_configuration() {
    try {
        const YAML::Node motors = YAML::LoadFile(motor_config_yaml_path())["motors"];
        std::vector<MotorConfig> axes(NUM_MOTORS);
        for (int i = 0; i < NUM_MOTORS; ++i) {
            const YAML::Node m = motors["motor_" + std::to_string(i + 1)];
            auto& a = axes[i];
            a.kp                        = m["kp"].as<float>();
            a.ki                        = m["ki"].as<float>();
            a.kd                        = m["kd"].as<float>();
            a.max_current_A             = m["max_current_a"].as<float>();
            a.max_velocity              = m["max_velocity"].as<float>();
            a.max_acceleration          = m["max_acceleration"].as<float>();
            a.position_min              = m["position_min"].as<float>();
            a.position_max              = m["position_max"].as<float>();
            a.position_warn_rev_padding = m["position_warn_rev_padding"].as<float>();
            a.max_voltage               = m["max_voltage"].as<float>();
            a.max_power_W               = m["max_power_w"].as<float>();
            a.def_timeout               = m["timeout_s"].as<float>();
            a.gear_reduction            = m["gear_reduction"].as<float>();
        }
        return axes;
    } catch (const std::exception& e) {
        std::cerr << "[motor_config] could not load motor_config.yaml ("
                  << e.what() << ") — using compiled-in fallback values\n";
        return fallback_arm_configuration();
    }
}

bool save_motor_config_value(int motor_idx, const std::string& key, float value) {
    const std::string path = motor_config_yaml_path();
    std::ifstream in(path);
    if (!in.is_open()) return false;
    std::vector<std::string> lines;
    for (std::string l; std::getline(in, l);) lines.push_back(l);
    in.close();

    const std::string motor_tag = "motor_" + std::to_string(motor_idx + 1) + ":";
    const std::string key_tag   = key + ":";
    size_t block_indent = 0;

    for (size_t i = 0, in_block = 0; i < lines.size(); ++i) {
        const size_t indent = lines[i].find_first_not_of(' ');
        if (indent == std::string::npos || lines[i][indent] == '#') continue;
        if (!in_block) {
            if (lines[i].compare(indent, motor_tag.size(), motor_tag) == 0) {
                in_block     = 1;
                block_indent = indent;
            }
            continue;
        }
        if (indent <= block_indent) return false;  // left the block: key not found
        if (lines[i].compare(indent, key_tag.size(), key_tag) != 0) continue;

        char buf[64];
        if (std::isnan(value))
            std::snprintf(buf, sizeof(buf), "%s .nan", key_tag.c_str());
        else
            std::snprintf(buf, sizeof(buf), "%s %.9g", key_tag.c_str(), value);
        lines[i] = lines[i].substr(0, indent) + buf;

        // Rewrite in place so a --symlink-install share path keeps writing
        // through into src/ (a temp-file rename would replace the symlink).
        std::ofstream out(path, std::ios::trunc);
        for (const auto& l : lines) out << l << "\n";
        return out.good();
    }
    return false;
}
