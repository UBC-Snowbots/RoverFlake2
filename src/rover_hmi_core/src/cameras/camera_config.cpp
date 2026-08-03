#include <rover_hmi_core/cameras/camera_config.h>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <cstdlib>

namespace rover_hmi_core::camera_config {
namespace {

// Same two-step resolution as spectro_paths: env var, then baked source dir.
QString packageRoot()
{
    if (const char* root = std::getenv("ROVERFLAKE_ROOT")) {
        QString candidate = QString::fromLocal8Bit(root) + "/src/rover_hmi_core";
        if (QDir(candidate).exists()) return candidate;
    }
#ifdef ROVER_HMI_CORE_SOURCE_DIR
    QString baked = QStringLiteral(ROVER_HMI_CORE_SOURCE_DIR);
    if (QDir(baked).exists()) return baked;
#endif
    return {};
}

}  // namespace

std::vector<Camera> parse(const QByteArray& json, QString* err)
{
    QJsonParseError perr;
    auto doc = QJsonDocument::fromJson(json, &perr);
    if (doc.isNull()) {
        if (err) *err = perr.errorString();
        return {};
    }
    std::vector<Camera> out;
    for (const auto& v : doc.object().value(QStringLiteral("cameras")).toArray()) {
        auto o = v.toObject();
        Camera c{o.value(QStringLiteral("name")).toString(),
                 o.value(QStringLiteral("label")).toString()};
        if (c.name.isEmpty() || c.label.isEmpty()) {
            if (err) *err = QStringLiteral("camera entry needs name, label");
            return {};
        }
        out.push_back(c);
    }
    if (out.empty() && err) *err = QStringLiteral("no cameras defined");
    return out;
}

std::vector<Camera> load(QString* err)
{
    QString root = packageRoot();
    if (root.isEmpty()) {
        if (err) *err = QStringLiteral("package root not found — set ROVERFLAKE_ROOT or rebuild");
        return {};
    }
    QFile f(root + "/config/cameras/camera_map.json");
    if (!f.open(QIODevice::ReadOnly)) {
        if (err) *err = QStringLiteral("cannot read %1").arg(f.fileName());
        return {};
    }
    return parse(f.readAll(), err);
}

QString topicFor(const Camera& cam)
{
    return QStringLiteral("/%1/image_raw_decoded").arg(cam.name);
}

}  // namespace rover_hmi_core::camera_config
