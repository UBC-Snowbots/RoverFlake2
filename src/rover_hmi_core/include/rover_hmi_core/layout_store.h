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

    // ── Walls: one dir per wall, one file per instance ──────────────────────
    struct WallEntry {
        QString name;      // display name ("wall" field of member files)
        QString dir_path;  // absolute walls/<slug>/ directory
        QString saved_at;  // newest member's timestamp
    };
    std::vector<WallEntry> listWalls() const;      // sorted by saved_at
    bool saveWallInstance(const QString& wall_name, const QString& instance,
                          const QJsonObject& layout);
    QJsonObject loadWallInstance(const QString& wall_name,
                                 const QString& instance) const;
    bool removeWall(const QString& wall_name);
    // Rewrites members' "wall" field; dir keeps its slug (name is authoritative).
    bool renameWall(const QString& old_name, const QString& new_name);

private:
    QString dir_;  // absolute layouts dir; empty when unresolved
    QString wallDirFor(const QString& wall_name) const;
};
