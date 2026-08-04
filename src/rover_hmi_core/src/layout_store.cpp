#include <rover_hmi_core/layout_store.h>

#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QSaveFile>
#include <algorithm>
#include <cstdlib>

static const char* LAYOUTS_SUBDIR = "config/layouts";

// First package root containing package.xml wins:
//   1. $ROVERFLAKE_ROOT/src/rover_hmi_core   (explicit override)
//   2. ROVER_HMI_CORE_SOURCE_DIR             (baked in by CMake at build time)
static QString resolvePackageDir() {
    if (const char* root = std::getenv("ROVERFLAKE_ROOT")) {
        QString pkg = QString(root) + "/src/rover_hmi_core";
        if (QFileInfo::exists(pkg + "/package.xml")) return pkg;
    }
#ifdef ROVER_HMI_CORE_SOURCE_DIR
    QString pkg = ROVER_HMI_CORE_SOURCE_DIR;
    if (QFileInfo::exists(pkg + "/package.xml")) return pkg;
#endif
    return {};
}

// "Arm Teleop" → "arm-teleop"; appends -2, -3… on collision (except `keep`)
static QString slugFor(const QString& name, const QDir& dir, const QString& keep = {}) {
    QString slug;
    for (QChar c : name.toLower()) {
        if (c.isLetterOrNumber()) slug += c;
        else if (!slug.endsWith('-')) slug += '-';
    }
    while (slug.endsWith('-')) slug.chop(1);
    while (slug.startsWith('-')) slug.remove(0, 1);
    if (slug.isEmpty()) slug = "layout";

    QString candidate = slug;
    for (int i = 2; dir.exists(candidate + ".json") && candidate + ".json" != keep; ++i)
        candidate = slug + "-" + QString::number(i);
    return candidate;
}

static bool writeJson(const QString& path, const QJsonObject& obj) {
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) return false;
    f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));  // git-diffable
    return f.commit();
}

LayoutStore::LayoutStore() {
    QString pkg = resolvePackageDir();
    if (!pkg.isEmpty()) dir_ = pkg + "/" + LAYOUTS_SUBDIR;
}

QString LayoutStore::statusMessage() const {
    return writable() ? QString()
        : "layouts dir not found — saving disabled (set ROVERFLAKE_ROOT or rebuild)";
}

std::vector<LayoutStore::Entry> LayoutStore::list() const {
    std::vector<Entry> out;
    if (dir_.isEmpty()) return out;
    for (const QFileInfo& fi : QDir(dir_).entryInfoList({"*.json"}, QDir::Files, QDir::Name)) {
        QFile f(fi.absoluteFilePath());
        if (!f.open(QIODevice::ReadOnly)) continue;
        QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        if (!doc.isObject()) continue;
        Entry e;
        e.file_path = fi.absoluteFilePath();
        e.json      = doc.object();
        e.name      = e.json["name"].toString(fi.completeBaseName());
        e.saved_at  = e.json["saved_at"].toString();
        out.push_back(std::move(e));
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const Entry& a, const Entry& b) { return a.saved_at < b.saved_at; });
    return out;
}

bool LayoutStore::save(const QJsonObject& layout) {
    if (!writable() || !QDir().mkpath(dir_)) return false;
    QString slug = slugFor(layout["name"].toString(), QDir(dir_));
    return writeJson(dir_ + "/" + slug + ".json", layout);
}

bool LayoutStore::remove(const QString& file_path) {
    return writable() && QFile::remove(file_path);
}

bool LayoutStore::rename(const QString& file_path, const QString& new_name) {
    if (!writable()) return false;
    QFile f(file_path);
    if (!f.open(QIODevice::ReadOnly)) return false;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    f.close();
    if (!doc.isObject()) return false;

    QJsonObject obj = doc.object();
    obj["name"] = new_name;
    QString old_file = QFileInfo(file_path).fileName();
    QString new_path = dir_ + "/" + slugFor(new_name, QDir(dir_), old_file) + ".json";
    if (!writeJson(new_path, obj)) return false;
    if (new_path != file_path) QFile::remove(file_path);
    return true;
}
