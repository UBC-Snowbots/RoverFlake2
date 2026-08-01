// spectro_paths.h — resolves in-repo locations for the spectrometer pipeline.
// Scripts and data stay tracked in the repo, never in ~/.config.
#pragma once

#include <QString>

namespace rover_hmi_core::spectro_paths {

// src/rover_hmi_core/scripts/spectrometer. Empty if unresolved.
QString scriptsDir();

// Absolute path to a vendored script, or empty if it isn't there.
QString script(const QString& filename);

// src/rover_hmi_core/config/spectrometer, created on first use.
// Holds models, captures and results. Empty if unresolved.
QString dataDir();

// Why resolution failed, for surfacing in the UI.
QString lastError();

}  // namespace rover_hmi_core::spectro_paths
