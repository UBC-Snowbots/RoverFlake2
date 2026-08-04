#include <rover_hmi_core/spectro_paths.h>

#include <QDir>
#include <QFileInfo>
#include <cstdlib>

#ifdef Q_OS_UNIX
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rover_hmi_core::spectro_paths {
namespace {

QString g_error;

// Package root, tried in order:
//   1. $ROVERFLAKE_ROOT/src/rover_hmi_core  (set in the container and by rover.sh)
//   2. ROVER_HMI_CORE_SOURCE_DIR            (baked in at build time)
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
    g_error = QStringLiteral("package root not found — set ROVERFLAKE_ROOT or rebuild");
    return {};
}

// The HMI usually runs as root inside the container while the repo belongs to
// the host user. Anything root writes into a *tracked* path then blocks git —
// a branch switch fails with "unable to create file ... Permission denied".
// Hand the data dir and its contents back to whoever owns the repo.
void adoptRepoOwnership(const QString& root, const QString& dir)
{
#ifdef Q_OS_UNIX
    if (::geteuid() != 0) return;  // nothing to fix when not root

    struct stat st;
    if (::stat(root.toLocal8Bit().constData(), &st) != 0) return;
    if (st.st_uid == 0) return;    // repo really is root's; leave it alone

    ::chown(dir.toLocal8Bit().constData(), st.st_uid, st.st_gid);
    for (const QFileInfo& fi :
         QDir(dir).entryInfoList(QDir::Files | QDir::Hidden | QDir::NoDotAndDotDot)) {
        ::chown(fi.absoluteFilePath().toLocal8Bit().constData(),
                st.st_uid, st.st_gid);
    }
#else
    Q_UNUSED(root); Q_UNUSED(dir);
#endif
}

}  // namespace

QString scriptsDir()
{
    QString root = packageRoot();
    if (root.isEmpty()) return {};

    QString dir = root + "/scripts/spectrometer";
    if (!QDir(dir).exists()) {
        g_error = QStringLiteral("scripts dir missing: %1").arg(dir);
        return {};
    }
    return dir;
}

QString script(const QString& filename)
{
    QString dir = scriptsDir();
    if (dir.isEmpty()) return {};

    QString path = dir + "/" + filename;
    if (!QFileInfo::exists(path)) {
        g_error = QStringLiteral("script missing: %1").arg(path);
        return {};
    }
    return path;
}

QString dataDir()
{
    QString root = packageRoot();
    if (root.isEmpty()) return {};

    QString dir = root + "/config/spectrometer";
    if (!QDir().mkpath(dir)) {
        g_error = QStringLiteral("cannot create data dir: %1").arg(dir);
        return {};
    }
    adoptRepoOwnership(root, dir);
    return dir;
}

QString lastError() { return g_error; }

}  // namespace rover_hmi_core::spectro_paths
