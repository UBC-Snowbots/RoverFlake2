// motor_config_io.cpp
//
// YAML-backed implementation of the motor configuration accessors declared in
// motor_config.h. The values live in the repository at
// arm_hardware_interface/config/motor_config.yaml and are resolved through the
// ament package share directory — with --symlink-install that path is a
// symlink into src/, so save_motor_config_value() writes straight to the repo.

#include <rover_hmi_core/arm/motor_config.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace {

float fieldOr(const YAML::Node& motor, const char* key, float fallback) {
    if (!motor || !motor[key]) return fallback;
    try {
        return motor[key].as<float>();
    } catch (const std::exception&) {
        return fallback;
    }
}

std::string formatValue(float value) {
    if (std::isnan(value)) return ".nan";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g", value);
    return buf;
}

size_t indentOf(const std::string& line) {
    size_t i = 0;
    while (i < line.size() && line[i] == ' ') ++i;
    return i;
}

bool isBlankOrComment(const std::string& line) {
    size_t i = indentOf(line);
    return i >= line.size() || line[i] == '#' || line[i] == '\r';
}

}  // namespace

std::string motor_config_yaml_path() {
    try {
        return ament_index_cpp::get_package_share_directory("arm_hardware_interface")
               + "/config/motor_config.yaml";
    } catch (const std::exception&) {
        return {};
    }
}

bool motor_config_yaml_in_repo() {
    std::error_code ec;
    const std::string path = motor_config_yaml_path();
    return !path.empty() && std::filesystem::is_symlink(path, ec);
}

std::vector<MotorConfig> get_arm_configuration() {
    auto axes = fallback_arm_configuration();
    const std::string path = motor_config_yaml_path();

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::cerr << "[motor_config] WARNING: could not load " << path << " ("
                  << e.what() << ") — using compiled-in fallback values\n";
        return axes;
    }

    const YAML::Node motors = root["motors"];
    if (!motors) {
        std::cerr << "[motor_config] WARNING: no 'motors:' map in " << path
                  << " — using compiled-in fallback values\n";
        return axes;
    }

    for (int i = 0; i < NUM_MOTORS; ++i) {
        const YAML::Node m = motors["motor_" + std::to_string(i + 1)];
        if (!m) {
            std::cerr << "[motor_config] WARNING: motor_" << (i + 1)
                      << " missing from " << path << " — fallback values kept\n";
            continue;
        }
        auto& a = axes[i];
        a.kp                        = fieldOr(m, "kp", a.kp);
        a.ki                        = fieldOr(m, "ki", a.ki);
        a.kd                        = fieldOr(m, "kd", a.kd);
        a.max_current_A             = fieldOr(m, "max_current_a", a.max_current_A);
        a.max_velocity              = fieldOr(m, "max_velocity", a.max_velocity);
        a.max_acceleration          = fieldOr(m, "max_acceleration", a.max_acceleration);
        a.position_min              = fieldOr(m, "position_min", a.position_min);
        a.position_max              = fieldOr(m, "position_max", a.position_max);
        a.position_warn_rev_padding = fieldOr(m, "position_warn_rev_padding",
                                              a.position_warn_rev_padding);
        a.max_voltage               = fieldOr(m, "max_voltage", a.max_voltage);
        a.max_power_W               = fieldOr(m, "max_power_w", a.max_power_W);
        a.def_timeout               = fieldOr(m, "timeout_s", a.def_timeout);
        a.gear_reduction            = fieldOr(m, "gear_reduction", a.gear_reduction);
    }
    return axes;
}

bool save_motor_config_value(int motor_idx, const std::string& key, float value) {
    const std::string path = motor_config_yaml_path();
    if (path.empty()) return false;

    std::ifstream in(path);
    if (!in.is_open()) return false;
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    in.close();

    const std::string motor_tag = "motor_" + std::to_string(motor_idx + 1) + ":";
    const std::string key_tag   = key + ":";

    // Locate the motor block, then the key line within it (the block ends at
    // the first non-blank line indented at or above the block header).
    size_t block_indent = 0;
    bool   in_block     = false;
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        const size_t indent = indentOf(line);

        if (!in_block) {
            if (line.compare(indent, motor_tag.size(), motor_tag) == 0) {
                in_block     = true;
                block_indent = indent;
            }
            continue;
        }

        if (!isBlankOrComment(line) && indent <= block_indent) break;  // block ended
        if (line.compare(indent, key_tag.size(), key_tag) != 0) continue;

        // Preserve any trailing comment on the value line.
        std::string trailing;
        const size_t hash = line.find('#', indent + key_tag.size());
        if (hash != std::string::npos) trailing = "  " + line.substr(hash);

        lines[i] = line.substr(0, indent) + key_tag + " " + formatValue(value) + trailing;

        // Rewrite in place — no temp-file+rename, which would replace the
        // share-dir symlink with a plain file instead of writing through it
        // into the repository.
        std::ofstream out(path, std::ios::trunc);
        if (!out.is_open()) return false;
        for (const auto& l : lines) out << l << "\n";
        return out.good();
    }
    return false;
}
