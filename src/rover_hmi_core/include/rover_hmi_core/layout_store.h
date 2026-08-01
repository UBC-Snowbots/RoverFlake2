// layout_store.h — repo-tracked storage for saved HMI layouts.
// One JSON file per layout in src/rover_hmi_core/config/layouts/, so layouts
// are shared via git instead of diverging in ~/.config across machines.

#pragma once

#include <QJsonObject>
#include <QString>
#include <vector>

class LayoutStore {
public:
    struct Entry {
        QString file_path;
        QString name;
        QString saved_at;   // "yyyy-MM-dd HH:mm"
        QJsonObject json;   // {name, saved_at, tree, visible}
    };

    // Resolves the layouts dir: $ROVERFLAKE_ROOT, else the build machine's
    // source dir baked in by CMake. Unresolved → read-only store.
    LayoutStore();

    bool writable() const { return !dir_.isEmpty(); }
    QString statusMessage() const;  // empty when writable
    QString dir() const { return dir_; }

    std::vector<Entry> list() const;       // sorted by saved_at, oldest first
    bool save(const QJsonObject& layout);  // writes <slug-of-name>.json
    bool remove(const QString& file_path);
    bool rename(const QString& file_path, const QString& new_name);

private:
    QString dir_;  // absolute layouts dir; empty when unresolved
};
