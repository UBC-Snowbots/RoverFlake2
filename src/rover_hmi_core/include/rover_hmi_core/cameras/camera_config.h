// camera_config.h — loads config/cameras/camera_map.json (in-repo, never ~/.config).
// Names must match cameras.json on the jetson-ffmpeg branch.
#pragma once

#include <QByteArray>
#include <QString>
#include <vector>

namespace rover_hmi_core::camera_config {

struct Camera {
    QString name;   // topic stem, e.g. "logitech"
    QString label;  // human name shown in UI, e.g. "Rear"
};

// Empty vector + err set on any problem (malformed json, missing field, no cameras).
std::vector<Camera> parse(const QByteArray& json, QString* err);

// Reads <package src>/config/cameras/camera_map.json via ROVERFLAKE_ROOT or
// the baked-in source dir, then parse().
std::vector<Camera> load(QString* err);

QString topicFor(const Camera& cam);  // "/<name>/image_raw_decoded"

}  // namespace rover_hmi_core::camera_config
